/*
  YT-011 APAL Hestia A2 Demo
  Firmware V3.1  -  Modbus / Satellite  (field arming + data-budget cadence)

  ============================================================================
  WHAT CHANGED IN V3.1  --  FIELD CONTROL YOU CAN USE WITHOUT A LAPTOP
  ============================================================================
  V3.0 fixed the link (CRC-validated Modbus, see below). V3.1 adds the controls
  you need for a field drop you can't reflash on site:

  1) BOOT IS ALWAYS DISARMED.
     Every time the box powers on -- battery, reset, anything -- it starts with
     satellite transmit OFF. It handshakes, reads sensors, shows ONLINE on the
     screen, but sends ZERO satellite messages until YOU arm it. There is no
     saved state, nothing to remember, no file to set. Power-on = safe, always.
     This means your final bench check before walking outside is automatically
     safe: it can't spend your data unless you deliberately arm it.

  2) ARM / DISARM = HOLD THE BUTTON 2 SECONDS.
     - A quick TAP still cycles the 4 LCD screens (unchanged).
     - A 2-SECOND HOLD toggles transmit. While you hold, the screen counts down
       "ARMING IN 2... 1..."; release early to cancel. When it fires, a big
       banner confirms the new state. Hold 2s again to turn it back off.

  3) BURST-THEN-SUSTAIN CADENCE (sized to your 30KB / 3-month budget).
     The moment you arm, it sends a quick BURST (every 45s for 5 min) so points
     land on your dashboard fast for filming -- then it automatically backs off
     to a 5-MINUTE SOAK rate for endurance. At ~100 bytes/message a 3-hour armed
     run costs ~4KB, so 30KB covers many sessions with room for a redo.
     (Heads up: sendUplink() HEX-ENCODES the payload, so ~45 bytes of JSON goes
     out as ~90 bytes. That's why the budget is tighter than it looks. Worth
     asking APAL whether the uplink register accepts BINARY -- that would halve
     bytes/message and double everything.)

  ============================================================================
  THE V3.0 LINK FIX  (unchanged, still the heart of this)
  ============================================================================
  Your RS-485 bus has no fail-safe BIAS RESISTORS, so during the turnaround
  between the Arduino's question and the Hestia's answer the A/B pair floats and
  the UART hears phantom 0x80/0xFF-ish garbage IN FRONT of the real reply. Stock
  ModbusMaster reads that first garbage byte, calls it "wrong slave ID," and
  quits. We instead capture the WHOLE reply and find the frame that starts with
  the slave ID AND ends with a valid CRC, skipping the garbage. Proven on the
  bench: unlock ACK, model, fw (1.41), status (0x7F) all decoded clean.

  Everything else is byte-for-byte your V2.2/V1.2: sensors, 47k/10k + 1.014 cal,
  the 4 LCD screens, the blink, D26, the o0/o1/o2 bench commands. Unchanged.

  HARDWARE / PINS (unchanged):
    A0  battery sense        D22 RS-485 DE+RE (jumpered together)
    D23 button (to GND)      D24 outside DS18B20    D25 box DS18B20
    D26 output LED           Serial1 (D18/D19) -> MAX485 -> Hestia
    I2C LCD at 0x27

  LIBRARIES: LiquidCrystal_I2C, OneWire, DallasTemperature.
             (ModbusMaster is NOT needed -- removed in V3.0.)
  LOAD IT:   Board = Mega 2560, pick the Mega's COM port, Upload, Monitor @115200.
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

// ------------------------------------------------------------
// Pin map  (unchanged from V1.2)
// ------------------------------------------------------------
const uint8_t PIN_VBAT_SENSE    = A0;
const uint8_t PIN_RS485_DE_RE   = 22;
const uint8_t PIN_BUTTON_INPUT  = 23;
const uint8_t PIN_TEMP_OUTSIDE  = 24;
const uint8_t PIN_TEMP_BOX      = 25;
const uint8_t PIN_REMOTE_OUTPUT = 26;   // Blue LED output

// ------------------------------------------------------------
// LCD
// ------------------------------------------------------------
LiquidCrystal_I2C lcd(0x27, 20, 4);     // if blank, try 0x3F

// ------------------------------------------------------------
// Temperature sensors
// ------------------------------------------------------------
OneWire oneWireOutside(PIN_TEMP_OUTSIDE);
OneWire oneWireBox(PIN_TEMP_BOX);
DallasTemperature tempOutsideSensor(&oneWireOutside);
DallasTemperature tempBoxSensor(&oneWireBox);

// ------------------------------------------------------------
// Battery divider / calibration  (unchanged from V1.2)
// ------------------------------------------------------------
const float R_VDIV_TOP       = 47000.0;   // 47k top
const float R_VDIV_BOTTOM    = 10000.0;   // 10k bottom
const float ADC_REF_VOLTAGE  = 5.256;     // measured 5V rail
const float VBAT_CALIBRATION = 1.014;     // multimeter cal

// ------------------------------------------------------------
// Hestia / Modbus
// ------------------------------------------------------------
const uint8_t  HESTIA_ID   = 1;
const uint32_t HESTIA_BAUD = 115200;

// Give-up ceiling for a reply burst. Replies arrive in a few ms; the reader
// returns the instant it finds a CRC-valid frame, so this is just the timeout.
const unsigned long MB_REPLY_WINDOW_MS = 200;

// Hestia Modbus registers (from APAL's sample)
const uint16_t REG_PASSWORD     = 0x0000;  // write 4x 0x0000 to unlock
const uint16_t REG_MODEL        = 0xEA66;  // 5 regs, ASCII
const uint16_t REG_FW           = 0xEA6B;  // 2 regs, ASCII
const uint16_t REG_STATUS       = 0xEA71;  // bitfield
const uint16_t REG_UPLINK_FREE  = 0xEA7D;  // 0 = ready to send
const uint16_t REG_UPLINK_WRITE = 0xC550;  // write payload here
const uint16_t REG_DL_LEN       = 0xEC60;  // downlink length (registers)
const uint16_t REG_DL_DATA      = 0xEC61;  // downlink data

// Status bits (UDP mode)
const uint16_t ST_ALL_READY = 0x1F;        // AT+IP+SIM+netreg+socket -> clear to transmit

// ------------------------------------------------------------
// Timing
// ------------------------------------------------------------
const unsigned long SENSOR_UPDATE_MS   = 2000;
const unsigned long LCD_UPDATE_MS      = 1000;
const unsigned long SERIAL_DEBUG_MS    = 2000;
const unsigned long MODEM_POLL_MS      = 5000;   // poll Hestia status/downlink
const unsigned long BUTTON_DEBOUNCE_MS = 40;
const unsigned long BLINK_INTERVAL_MS  = 500;

// *** SATELLITE SEND CADENCE  (data budget: 30KB / 3 months) ***
// After you ARM, it bursts fast for a few minutes (quick points to film), then
// auto-slows to a long soak rate for endurance. ~100 bytes/msg -> a 3hr armed
// run is ~4KB, so 30KB covers many sessions with margin.
const unsigned long BURST_WINDOW_MS     = 300000;  // fast bursts for 5 min after arming
const unsigned long BURST_INTERVAL_MS   = 45000;   // 45s during the burst
const unsigned long SUSTAIN_INTERVAL_MS = 300000;  // 5 min for the long soak

// ------------------------------------------------------------
// Readings / status
// ------------------------------------------------------------
float vSense = 0.0, vBatt = 0.0;
int   rawAdc = 0;
float tempOutsideF = NAN, tempBoxF = NAN;

// Hestia link state
bool     hestiaLinkOk = false;
char     hestiaModel[16] = "----";
char     hestiaFw[12]    = "----";
uint16_t hestiaStatus    = 0;
bool     netReady        = false;
uint32_t uplinkCount     = 0;
char     lastDownlink[24]= "(none)";
char     lastTxResult[20]= "(idle)";   // last uplink attempt result (COMMS screen)
int      lastOutputCmd   = -1;         // last output command value applied (LED screen)

// Remote output  (unchanged behavior from V1.2)
bool remoteOutputState = false;
bool remoteBlinkMode   = false;
unsigned long lastBlinkToggle = 0;

// UI / timing
unsigned long lastSensorUpdate = 0, lastLcdUpdate = 0, lastSerialDebug = 0;
unsigned long lastModemPoll = 0, lastUplinkTime = 0, lastLinkTry = 0;
const unsigned long LINK_RETRY_MS = 3000;   // re-knock on the dongle until it answers (no reset)
uint16_t hestiaRetries = 0;                 // handshake attempts (shown on SATELLITE screen)
uint8_t screenIndex = 0;
const uint8_t SCREEN_COUNT = 4;
bool lastButtonReading = HIGH, stableButtonState = HIGH;
unsigned long lastButtonChange = 0;
uint16_t uplinkSeq = 0;
String usbBuf;

// ------------------------------------------------------------
// Field arming  (boot DISARMED; 2s button hold toggles transmit)
// ------------------------------------------------------------
bool          txArmed       = false;   // <-- starts false EVERY boot (safe default)
unsigned long armedAt       = 0;       // when we last armed (drives the burst window)
unsigned long btnDownTime   = 0;       // when the current press started
bool          holdFired     = false;   // the 2s toggle already fired this press
int           lastCountShown= -1;      // last countdown number drawn (avoid flicker)
unsigned long bannerUntil   = 0;       // hold normal LCD redraws until this time
const unsigned long HOLD_ARM_MS = 2000;  // hold this long to arm/disarm
const unsigned long TAP_MAX_MS  = 400;   // shorter press = a screen-cycle tap

// ============================================================
// Helpers  (unchanged from V1.2)
// ============================================================
bool tempValid(float t){ if(isnan(t)) return false; if(t<-100.0) return false; if(t>250.0) return false; return true; }
String fmtTempF(float t){ return tempValid(t) ? String(t,1) : String("ERR"); }
String fmtFloat(float v, uint8_t d){ return String(v,d); }
void lcdPrintLine(uint8_t row, String text){ if(text.length()>20) text.remove(20); while(text.length()<20) text+=' '; lcd.setCursor(0,row); lcd.print(text); }
const char* boxThermalStatus(){ if(!tempValid(tempBoxF)) return "SENSOR ERR"; if(tempBoxF>=130.0) return "HOT"; if(tempBoxF>=110.0) return "WARM"; return "OK"; }
const char* batteryStatus(){ if(vBatt<11.5) return "LOW"; if(vBatt<12.0) return "CHECK"; return "OK"; }
const char* sensorStatus(){ if(!tempValid(tempOutsideF)) return "TEMP OUT ERR"; if(!tempValid(tempBoxF)) return "TEMP BOX ERR"; return "OK"; }

int readAnalogAverage(uint8_t pin){ const uint8_t n=16; unsigned long t=0; for(uint8_t i=0;i<n;i++){ t+=analogRead(pin); delay(2);} return t/n; }
void updateVoltage(){
  rawAdc = readAnalogAverage(PIN_VBAT_SENSE);
  vSense = (rawAdc * ADC_REF_VOLTAGE) / 1023.0;
  float m = (R_VDIV_TOP + R_VDIV_BOTTOM) / R_VDIV_BOTTOM;
  vBatt = vSense * m * VBAT_CALIBRATION;
  if(rawAdc <= 1){ vSense=0.0; vBatt=0.0; }
}
void updateTemperatures(){
  tempOutsideSensor.requestTemperatures();
  tempBoxSensor.requestTemperatures();
  tempOutsideF = tempOutsideSensor.getTempFByIndex(0);
  tempBoxF = tempBoxSensor.getTempFByIndex(0);
}
void updateSensors(){ if(millis()-lastSensorUpdate>=SENSOR_UPDATE_MS){ lastSensorUpdate=millis(); updateVoltage(); updateTemperatures(); } }

// Output  (unchanged behavior from V1.2)
void setRemoteOutput(bool state){ remoteOutputState=state; digitalWrite(PIN_REMOTE_OUTPUT, state?HIGH:LOW); }
void handleRemoteBlink(){ if(!remoteBlinkMode) return; if(millis()-lastBlinkToggle>=BLINK_INTERVAL_MS){ lastBlinkToggle=millis(); setRemoteOutput(!remoteOutputState); } }

// Map an output command value (0/1/2) to your existing output logic.
void applyOutputCommand(int val){
  lastOutputCmd = val;
  if(val==0){ remoteBlinkMode=false; setRemoteOutput(false); }
  else if(val==1){ remoteBlinkMode=false; setRemoteOutput(true); }
  else if(val==2){ remoteBlinkMode=true;  setRemoteOutput(true); lastBlinkToggle=millis(); }
}
int currentOutputCode(){ if(remoteBlinkMode) return 2; return remoteOutputState ? 1 : 0; }

int hexNibble(char c){
  if(c>='0'&&c<='9') return c-'0';
  if(c>='a'&&c<='f') return c-'a'+10;
  if(c>='A'&&c<='F') return c-'A'+10;
  return -1;
}

// ------------------------------------------------------------
// Arm banners (forward-declared so handleButton can call them)
// ------------------------------------------------------------
void drawHoldCountdown(int remain);
void drawArmConfirmation();

// ------------------------------------------------------------
// Button: quick TAP cycles screens; 2s HOLD toggles transmit arm.
// ------------------------------------------------------------
void handleButton(){
  bool reading = digitalRead(PIN_BUTTON_INPUT);
  if(reading != lastButtonReading){ lastButtonChange = millis(); lastButtonReading = reading; }

  if((millis()-lastButtonChange) > BUTTON_DEBOUNCE_MS && reading != stableButtonState){
    stableButtonState = reading;
    if(stableButtonState == LOW){            // PRESS (INPUT_PULLUP -> pressed = LOW)
      btnDownTime    = millis();
      holdFired      = false;
      lastCountShown = -1;
    } else {                                 // RELEASE
      unsigned long held = millis() - btnDownTime;
      if(!holdFired){
        if(held < TAP_MAX_MS){               // quick tap -> next screen
          screenIndex++; if(screenIndex>=SCREEN_COUNT) screenIndex=0;
        }
        lcd.clear();                         // clean redraw (covers tap OR cancelled hold)
      }
    }
  }

  // While held and the toggle hasn't fired yet: run the arm countdown.
  if(stableButtonState == LOW && !holdFired){
    unsigned long held = millis() - btnDownTime;
    if(held >= TAP_MAX_MS){
      int remain = (int)((HOLD_ARM_MS - held + 999) / 1000);   // ceil seconds left
      if(remain < 0) remain = 0;
      if(remain != lastCountShown){ lastCountShown = remain; drawHoldCountdown(remain); }
    }
    if(held >= HOLD_ARM_MS){
      holdFired = true;
      txArmed = !txArmed;                    // TOGGLE
      if(txArmed){                           // just armed -> fresh session + fire soon
        armedAt = millis();
        lastUplinkTime = 0;                  // first uplink goes out on the next poll
        uplinkSeq = 0;
        uplinkCount = 0;
      }
      drawArmConfirmation();                 // big unmissable banner
    }
  }
}

// ============================================================
// RAW MODBUS over MAX485  --  CRC-validated, skips idle-bus garbage (V3.0)
// ============================================================
uint16_t modbusCrc(const uint8_t* buf, uint8_t len){
  uint16_t crc = 0xFFFF;
  for(uint8_t i=0;i<len;i++){
    crc ^= buf[i];
    for(uint8_t b=0;b<8;b++){ if(crc & 1){ crc >>= 1; crc ^= 0xA001; } else crc >>= 1; }
  }
  return crc;
}

// Turn the line around and send a frame, then leave the transceiver in RECEIVE.
void mbSend(const uint8_t* tx, uint8_t txLen){
  while(Serial1.available()) Serial1.read();   // clear anything stale before we talk
  UCSR1B &= ~(1 << RXEN1);                      // <-- NEW: deafen our own UART RX during TX
  digitalWrite(PIN_RS485_DE_RE, HIGH);
  delayMicroseconds(50);                       // on-ramp so the driver is awake
  Serial1.write(tx, txLen);
  Serial1.flush();                             // block until the last bit is on the wire
  digitalWrite(PIN_RS485_DE_RE, LOW);          // listen
  UCSR1B |= (1 << RXEN1);                       // <-- NEW: ears back on for the real reply
}

// FC04 read: capture the reply, return the instant a CRC-valid frame is found.
bool mbReadInput(uint16_t addr, uint8_t count, uint16_t* out){
  for(uint8_t r=0;r<count;r++) out[r]=0;

  uint8_t tx[8], n=0;
  tx[n++]=HESTIA_ID; tx[n++]=0x04;
  tx[n++]=addr>>8;   tx[n++]=addr&0xFF;
  tx[n++]=0x00;      tx[n++]=count;
  uint16_t c=modbusCrc(tx,n); tx[n++]=c&0xFF; tx[n++]=c>>8;

  mbSend(tx, n);

  uint8_t rx[128]; uint8_t got=0;
  unsigned long t0=millis();
  while(millis()-t0 < MB_REPLY_WINDOW_MS){
    if(!Serial1.available()) continue;
    uint8_t b=Serial1.read();
    if(got<sizeof(rx)) rx[got++]=b;
    for(int i=0; i+5<=got; i++){
      if(rx[i]!=HESTIA_ID) continue;
      uint8_t fc=rx[i+1];
      if(fc==0x04){
        uint8_t bc=rx[i+2];
        if(bc==0 || i+3+bc+2 > got) continue;
        uint16_t cc=modbusCrc(&rx[i], 3+bc);
        if((cc&0xFF)==rx[i+3+bc] && (cc>>8)==rx[i+4+bc]){
          uint8_t regs=bc/2;
          for(uint8_t r=0; r<regs && r<count; r++)
            out[r] = ((uint16_t)rx[i+3+2*r]<<8) | rx[i+4+2*r];
          return true;
        }
      } else if(fc==0x84){                 // exception reply to FC04
        uint16_t cc=modbusCrc(&rx[i],3);
        if((cc&0xFF)==rx[i+3] && (cc>>8)==rx[i+4]) return false;
      }
    }
  }
  return false;
}

// FC16 write: capture the reply, return the instant a CRC-valid ACK is found.
bool mbWriteRegs(uint16_t addr, uint8_t count, const uint16_t* in){
  uint8_t tx[140]; uint8_t n=0;
  tx[n++]=HESTIA_ID; tx[n++]=0x10;
  tx[n++]=addr>>8;   tx[n++]=addr&0xFF;
  tx[n++]=0x00;      tx[n++]=count;
  tx[n++]=count*2;
  for(uint8_t i=0;i<count;i++){ tx[n++]=in[i]>>8; tx[n++]=in[i]&0xFF; }
  uint16_t c=modbusCrc(tx,n); tx[n++]=c&0xFF; tx[n++]=c>>8;

  mbSend(tx, n);

  uint8_t rx[96]; uint8_t got=0;
  unsigned long t0=millis();
  while(millis()-t0 < MB_REPLY_WINDOW_MS){
    if(!Serial1.available()) continue;
    uint8_t b=Serial1.read();
    if(got<sizeof(rx)) rx[got++]=b;
    for(int i=0; i+5<=got; i++){
      if(rx[i]!=HESTIA_ID) continue;
      uint8_t fc=rx[i+1];
      if(fc==0x10){
        if(i+8 > got) continue;
        uint16_t cc=modbusCrc(&rx[i], 6);
        if((cc&0xFF)==rx[i+6] && (cc>>8)==rx[i+7]) return true;
      } else if(fc==0x90){                // exception reply to FC16
        uint16_t cc=modbusCrc(&rx[i],3);
        if((cc&0xFF)==rx[i+3] && (cc>>8)==rx[i+4]) return false;
      }
    }
  }
  return false;
}

// ------------------------------------------------------------
// Hestia operations built on the raw layer
// ------------------------------------------------------------
String readRegText(uint16_t addr, uint8_t count){
  uint16_t regs[8];
  if(count>8) count=8;
  if(!mbReadInput(addr, count, regs)) return String("");
  String s;
  for(uint8_t i=0;i<count;i++){
    char hi=(char)(regs[i]>>8), lo=(char)(regs[i]&0xFF);
    if(hi>=32 && hi<127) s+=hi;
    if(lo>=32 && lo<127) s+=lo;
  }
  s.trim();
  return s;
}

bool unlockHestia(){
  uint16_t pw[4] = {0,0,0,0};
  return mbWriteRegs(REG_PASSWORD, 4, pw);
}

void hestiaHandshake(){
  hestiaRetries++;
  if(!unlockHestia()){
    hestiaLinkOk = false;
    Serial.print(F("[handshake] try #")); Serial.print(hestiaRetries);
    Serial.println(F("  no valid reply (idle-bus garbage only / link down)"));
    return;
  }
  hestiaLinkOk = true;
  readRegText(REG_MODEL, 5).toCharArray(hestiaModel, sizeof(hestiaModel));
  readRegText(REG_FW, 2).toCharArray(hestiaFw, sizeof(hestiaFw));
  Serial.print(F("[handshake] try #")); Serial.print(hestiaRetries);
  Serial.print(F("  LINKED  model=")); Serial.print(hestiaModel);
  Serial.print(F(" fw=")); Serial.println(hestiaFw);
}

uint16_t readStatusReg(){
  uint16_t st[1];
  if(!mbReadInput(REG_STATUS, 1, st)) return 0;
  return st[0];
}
bool uplinkBufferFree(){
  uint16_t v[1];
  if(!mbReadInput(REG_UPLINK_FREE, 1, v)) return false;
  return v[0] == 0;
}

String buildPayload(){
  String p = "{\"v\":";
  p += String(vBatt, 2);
  p += ",\"tb\":";  p += tempValid(tempBoxF)     ? String(tempBoxF,1)     : String("null");
  p += ",\"to\":";  p += tempValid(tempOutsideF) ? String(tempOutsideF,1) : String("null");
  p += ",\"o\":";   p += String(currentOutputCode());
  p += ",\"n\":";   p += String(uplinkSeq);
  p += "}";
  return p;
}

bool sendUplink(const String& payload){
  static const char H[] = "0123456789abcdef";
  uint16_t regs[64];
  uint8_t n = 0;
  for(int i=0; i<payload.length() && n<62; i++){
    uint8_t b = (uint8_t)payload[i];
    regs[n++] = ((uint16_t)H[(b>>4)&0x0F] << 8) | (uint8_t)H[b&0x0F];
  }
  regs[n++] = 0x0D0A;   // CRLF terminator
  return mbWriteRegs(REG_UPLINK_WRITE, n, regs);
}

void checkDownlink(){
  uint16_t lenReg[1];
  if(!mbReadInput(REG_DL_LEN, 1, lenReg)) return;
  uint16_t n = lenReg[0];
  if(n == 0) return;
  if(n > 40) n = 40;
  uint16_t data[40];
  if(!mbReadInput(REG_DL_DATA, (uint8_t)n, data)) return;
  String payload;
  for(uint8_t i=0;i<n;i++){
    uint16_t w = data[i];
    int h1 = hexNibble((char)(w>>8)), h2 = hexNibble((char)(w&0xFF));
    if(h1>=0 && h2>=0) payload += (char)((h1<<4)|h2);
  }
  if(payload.length()==0) return;
  payload.toCharArray(lastDownlink, sizeof(lastDownlink));
  Serial.print(F("DOWNLINK: ")); Serial.println(payload);
  int oi = payload.indexOf("\"o\"");
  if(oi>=0){ int c=payload.indexOf(':',oi); if(c>=0) applyOutputCommand(payload.substring(c+1).toInt()); }
}

// Current send spacing: fast burst right after arming, then the long soak rate.
unsigned long currentUplinkInterval(){
  if(txArmed && (millis() - armedAt < BURST_WINDOW_MS)) return BURST_INTERVAL_MS;
  return SUSTAIN_INTERVAL_MS;
}

// Poll the Hestia: refresh status, read downlink, uplink if armed + it's time.
void serviceHestia(){
  if(!hestiaLinkOk){
    if(millis()-lastLinkTry >= LINK_RETRY_MS){ lastLinkTry = millis(); hestiaHandshake(); }
    return;
  }
  if(millis()-lastModemPoll < MODEM_POLL_MS) return;
  lastModemPoll = millis();

  hestiaStatus = readStatusReg();
  netReady = (hestiaStatus & ST_ALL_READY) == ST_ALL_READY;

  checkDownlink();   // always listening (inbound; doesn't spend your uplink budget)

  if(millis()-lastUplinkTime >= currentUplinkInterval()){
    lastUplinkTime = millis();
    if(!txArmed){
      snprintf(lastTxResult,sizeof(lastTxResult),"TX OFF (safe)");
      Serial.println(F("(disarmed; not sending)"));
    } else {
      uplinkSeq++;
      String payload = buildPayload();
      if(netReady && uplinkBufferFree()){
        if(sendUplink(payload)){ uplinkCount++; snprintf(lastTxResult,sizeof(lastTxResult),"SENT #%u",(unsigned)uplinkSeq); Serial.print(F("UPLINK SENT: ")); Serial.println(payload); }
        else { snprintf(lastTxResult,sizeof(lastTxResult),"write FAIL"); Serial.println(F("UPLINK write failed")); }
      } else {
        snprintf(lastTxResult,sizeof(lastTxResult),"held:no net");
        Serial.print(F("UPLINK held (network not ready): ")); Serial.println(payload);
      }
    }
  }
}

// USB bench-test: type o0 / o1 / o2 in the Serial Monitor to drive the output.
void readUsbTestCommand(){
  while(Serial.available()){
    char c = Serial.read();
    if(c=='\n' || c=='\r'){
      usbBuf.trim();
      if(usbBuf=="o0"){ applyOutputCommand(0); Serial.println(F("[bench] output -> OFF")); }
      else if(usbBuf=="o1"){ applyOutputCommand(1); Serial.println(F("[bench] output -> ON")); }
      else if(usbBuf=="o2"){ applyOutputCommand(2); Serial.println(F("[bench] output -> BLINK")); }
      else if(usbBuf.length()) Serial.println(F("[bench] commands: o0 o1 o2"));
      usbBuf="";
    } else if(usbBuf.length()<8){ usbBuf += c; }
  }
}

// ============================================================
// LCD screens
// ============================================================
const char* satLinkWord(){
  if(!hestiaLinkOk)           return "NO REPLY";
  if(!(hestiaStatus & 0x01))  return "STARTING";
  if(!(hestiaStatus & 0x04))  return "SIM INIT";
  if(!(hestiaStatus & 0x08))  return "SEARCHING";
  if(!(hestiaStatus & 0x10))  return "REGISTERED";
  return "ONLINE";
}
const char* satCoach(){
  if(!hestiaLinkOk)           return "Check 12V & A/B wire";
  if(!(hestiaStatus & 0x08))  return "Aim at clear S sky";
  if(!(hestiaStatus & 0x10))  return "Linked, connecting..";
  if(!txArmed)                return "Linked - hold to arm";
  return "Linked - SENDING";
}
String satFlagsLine(){
  String sim = (hestiaLinkOk && (hestiaStatus & 0x04)) ? "OK" : "--";
  String net = (hestiaLinkOk && (hestiaStatus & 0x08)) ? "OK" : "--";
  String soc = (hestiaLinkOk && (hestiaStatus & 0x10)) ? "OK" : "--";
  return "SIM:"+sim+" NET:"+net+" SOC:"+soc;
}

// Screen 1 -- sensor readings
void drawReadingsScreen(){
  lcdPrintLine(0,"READINGS         1/4");
  lcdPrintLine(1,"Batt:    "+fmtFloat(vBatt,2)+"V "+batteryStatus());
  lcdPrintLine(2,"Box:     "+fmtTempF(tempBoxF)+" F");
  lcdPrintLine(3,"Outside: "+fmtTempF(tempOutsideF)+" F");
}
// Screen 2 -- satellite attach progress (the field screen)
void drawSatelliteScreen(){
  lcdPrintLine(0,"SATELLITE        2/4");
  lcdPrintLine(1,String("Link: ")+satLinkWord());
  if(!hestiaLinkOk) lcdPrintLine(2,"RS485 retries: "+String(hestiaRetries));
  else              lcdPrintLine(2,satFlagsLine());
  lcdPrintLine(3,satCoach());
}
// Screen 3 -- transmit / receive data
void drawCommsScreen(){
  lcdPrintLine(0,"COMMS            3/4");
  lcdPrintLine(1,String("TX: ")+(txArmed?"ARMED":"OFF")+"  sent:"+String(uplinkCount));
  lcdPrintLine(2,"TX last: "+String(lastTxResult));
  lcdPrintLine(3,"RX last: "+String(lastDownlink));
}
// Screen 4 -- remote LED / output control
void drawRemoteLedScreen(){
  lcdPrintLine(0,"REMOTE LED       4/4");
  lcdPrintLine(1,String("State: ")+(remoteBlinkMode?"BLINK":(remoteOutputState?"ON":"OFF")));
  lcdPrintLine(2,String("Last cmd: ")+(lastOutputCmd<0?String("none"):(String("o=")+lastOutputCmd)));
  lcdPrintLine(3,"Drives D26 blue LED");
}

// --- arm UX banners ---
void drawHoldCountdown(int remain){
  lcdPrintLine(0,"  HOLD TO ARM TX    ");
  lcdPrintLine(1,"                    ");
  lcdPrintLine(2,"    ARMING IN "+String(remain)+"     ");
  lcdPrintLine(3,"  release = cancel  ");
}
void drawArmConfirmation(){
  lcd.clear();
  if(txArmed){
    lcdPrintLine(0,"*** TX IS LIVE! ***");
    lcdPrintLine(1,"  SENDING SAT DATA  ");
    lcdPrintLine(2,"  DON'T WASTE IT!   ");
    lcdPrintLine(3,"  hold 2s to stop   ");
  } else {
    lcdPrintLine(0,"--- TX IS OFF ---   ");
    lcdPrintLine(1,"  disarmed & safe   ");
    lcdPrintLine(2,"  nothing sending   ");
    lcdPrintLine(3,"  hold 2s to arm    ");
  }
  bannerUntil = millis() + 3000;   // hold this on screen ~3s
}

void updateLcd(){
  if(stableButtonState == LOW) return;     // button held: the countdown owns the screen
  if(millis() < bannerUntil)   return;     // an arm/disarm banner is showing
  if(millis()-lastLcdUpdate < LCD_UPDATE_MS) return;
  lastLcdUpdate = millis();
  switch(screenIndex){
    case 0: drawReadingsScreen();  break;
    case 1: drawSatelliteScreen(); break;
    case 2: drawCommsScreen();     break;
    case 3: drawRemoteLedScreen(); break;
    default: screenIndex=0; lcd.clear(); drawReadingsScreen(); break;
  }
}

// ============================================================
// Setup / loop
// ============================================================
void setup(){
  pinMode(PIN_RS485_DE_RE, OUTPUT); digitalWrite(PIN_RS485_DE_RE, LOW);  // start in receive
  pinMode(PIN_BUTTON_INPUT, INPUT_PULLUP);
  pinMode(PIN_REMOTE_OUTPUT, OUTPUT); digitalWrite(PIN_REMOTE_OUTPUT, LOW);

  Serial.begin(115200);
  Serial1.begin(HESTIA_BAUD);
  // ModbusMaster removed in V3.0 -- the raw CRC-validated layer drives Serial1
  // and the DE/RE pin directly (see mbSend / mbReadInput / mbWriteRegs).

  lcd.init(); lcd.backlight(); lcd.clear();
  tempOutsideSensor.begin();
  tempBoxSensor.begin();
  updateVoltage(); updateTemperatures();

  lcdPrintLine(0,"YT-011 BOOTING");
  lcdPrintLine(1,"Firmware V3.1");
  lcdPrintLine(2,"Modbus / Satellite");
  lcdPrintLine(3,"Linking Hestia...");
  delay(400);

  hestiaHandshake();   // unlock + read identity

  lcd.clear();
  lcdPrintLine(0,"YT-011 HESTIA A2");
  lcdPrintLine(1, hestiaLinkOk ? (String("Hestia OK fw ")+hestiaFw) : String("Hestia: NO LINK"));
  lcdPrintLine(2, hestiaLinkOk ? (String("Model ")+hestiaModel)     : String("check wiring"));
  lcdPrintLine(3,"DISARMED - hold 2s");
  delay(1500);
  lcd.clear();

  Serial.println(F("YT-011 Firmware V3.1 (Modbus / Satellite, CRC link + field arming)"));
  Serial.print(F("Hestia link: ")); Serial.println(hestiaLinkOk?"OK":"NO LINK");
  if(hestiaLinkOk){ Serial.print(F("  model=")); Serial.print(hestiaModel); Serial.print(F(" fw=")); Serial.println(hestiaFw); }
  Serial.println(F("Boot state: DISARMED (no satellite TX). HOLD the button 2s to arm."));
  Serial.println(F("Bench test: type o1 (on) / o2 (blink) / o0 (off) + Enter to drive the output."));
}

void loop(){
  updateSensors();
  handleButton();
  handleRemoteBlink();
  serviceHestia();
  readUsbTestCommand();
  printDebugToUsb();
  updateLcd();
}

// ============================================================
// USB debug
// ============================================================
void printDebugToUsb(){
  if(millis()-lastSerialDebug < SERIAL_DEBUG_MS) return;
  lastSerialDebug = millis();
  Serial.print(F("VBAT=")); Serial.print(vBatt,2);
  Serial.print(F("V OUT=")); Serial.print(fmtTempF(tempOutsideF));
  Serial.print(F("F BOX=")); Serial.print(fmtTempF(tempBoxF));
  Serial.print(F(" | Hestia=")); Serial.print(hestiaLinkOk?hestiaModel:"NO LINK");
  Serial.print(F(" fw=")); Serial.print(hestiaFw);
  Serial.print(F(" net=")); Serial.print(netReady?"READY":"not reg");
  Serial.print(F(" TX=")); Serial.print(txArmed?"ARMED":"off");
  Serial.print(F(" uplinks=")); Serial.print(uplinkCount);
  Serial.print(F(" out=")); Serial.println(currentOutputCode());
}

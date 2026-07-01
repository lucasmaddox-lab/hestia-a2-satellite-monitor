/*
  RS-485 RAW PROBE  -  shows the actual bytes coming back from the Hestia.
  Upload this INSTEAD of the main firmware. Serial Monitor @ 115200.
  Sends the exact unlock frame your main code sends, then dumps every
  reply byte in hex. This ends the guessing about what 0xE0 means.
*/
const uint8_t PIN_RS485_DE_RE = 22;   // same pin as your main code

uint16_t crc16(const uint8_t* buf, uint8_t len){
  uint16_t crc = 0xFFFF;
  for(uint8_t i=0;i<len;i++){
    crc ^= buf[i];
    for(uint8_t b=0;b<8;b++){ if(crc & 1){ crc >>= 1; crc ^= 0xA001; } else crc >>= 1; }
  }
  return crc;
}

void setup(){
  pinMode(PIN_RS485_DE_RE, OUTPUT);
  digitalWrite(PIN_RS485_DE_RE, LOW);   // start in receive
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(300);
  Serial.println(F("RS-485 RAW PROBE - unlock frame to slave 1, dumping replies"));
}

void sendUnlockAndListen(){
  uint8_t frame[17]; uint8_t n = 0;
  frame[n++]=0x01; frame[n++]=0x10;             // slave 1, FC16 write-multiple
  frame[n++]=0x00; frame[n++]=0x00;             // start addr 0x0000
  frame[n++]=0x00; frame[n++]=0x04;             // qty = 4 registers
  frame[n++]=0x08;                              // byte count = 8
  for(uint8_t i=0;i<8;i++) frame[n++]=0x00;     // 4x 0x0000
  uint16_t c = crc16(frame, n);
  frame[n++]=(uint8_t)(c & 0xFF); frame[n++]=(uint8_t)(c >> 8);   // CRC lo, hi

  Serial.print(F("TX: "));
  for(uint8_t i=0;i<n;i++){ if(frame[i]<16) Serial.print('0'); Serial.print(frame[i],HEX); Serial.print(' '); }
  Serial.println();

  digitalWrite(PIN_RS485_DE_RE, HIGH);
  delayMicroseconds(50);
  Serial1.write(frame, n);
  Serial1.flush();                              // wait until the last bit is physically out
  digitalWrite(PIN_RS485_DE_RE, LOW);           // back to receive

  Serial.print(F("RX: "));
  unsigned long t0 = millis(); uint16_t got = 0;
  while(millis() - t0 < 250){
    if(Serial1.available()){ uint8_t b = Serial1.read(); if(b<16) Serial.print('0'); Serial.print(b,HEX); Serial.print(' '); got++; }
  }
  if(got==0) Serial.print(F("(nothing)"));
  Serial.print(F("   [")); Serial.print(got); Serial.println(F(" bytes]"));
  Serial.println();
}

void loop(){ sendUnlockAndListen(); delay(3000); }
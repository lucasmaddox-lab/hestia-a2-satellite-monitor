# Satellite IoT Monitoring Box — APAL HESTIA A2 + Arduino Mega

A self-contained, off-grid monitoring box that reads a few sensors and sends the
readings up to a satellite — from places with no cell service and no Wi-Fi. Built
around the **APAL HESTIA A2** satellite IoT dongle, an **Arduino Mega 2560**, and a
**MAX485** RS-485 transceiver talking **Modbus RTU**.

This repo has the firmware, wiring diagrams, and the diagnostic sketch I used to
chase down the one bug that ate most of the project (details below — it's a good one).

**▶ Watch the full build video:** _[YouTube link — paste URL here]_

> ⚠️ **Proof-of-concept, not a product.** This is an educational build for learning and
> experimentation. It is **not** finished, production-ready, or safety-certified. If you
> build on it, do your own testing and validation for your application. Everything here is
> provided as-is, with no warranty.

---

## What it does

- Reads **two DS18B20 temperature sensors** (one outside the enclosure, one inside it)
- Monitors its own **battery voltage** so you know the box is healthy in the field
- Shows everything locally on a **20×4 LCD** (four cycling screens, one button)
- Runs fully **off-grid** on a 12V LiFePO4 battery
- Sends the readings as small packets **up to a satellite** and down to the cloud

It's deliberately simple — temperature and battery — because the point isn't the
sensors, it's proving the pipeline: local data, off-grid power, and a satellite link
carrying it home.

## How it works (the data path)

```
DS18B20 + battery sense  ->  Arduino Mega 2560  ->  MAX485 (RS-485 / Modbus RTU)
   ->  APAL HESTIA A2  ->  satellite (Skylo NTN)  ->  CeresGate (APAL's cloud gateway)
```

The Arduino is the Modbus **master**; the HESTIA A2 is the slave. The Arduino builds a
Modbus frame, hands it to the MAX485 to drive onto the RS-485 pair, and the HESTIA takes
it from there — up to the satellite and back down into CeresGate, where the uplinked data
lands. From CeresGate you can forward it on to your own dashboard / broker (out of scope
for this build).

This is **low-bandwidth and not real-time** by design — small packets on a schedule
(my SIM came with ~30 KB over 180 days), not streaming. It's built to get a little bit of
important data out of somewhere remote, reliably.

---

## Hardware

| Part | Role |
|---|---|
| APAL HESTIA A2 | Satellite IoT dongle (the star of the show) |
| Arduino Mega 2560 | Host controller / Modbus master (chosen for its multiple hardware serial ports) |
| MAX485 module | RS-485 transceiver — translates the Arduino's UART to the RS-485 pair |
| DS18B20 ×2 | Temperature sensors (outside + inside the enclosure) |
| 20×4 I²C LCD (0x27) | Local display |
| 12V LiFePO4 battery | Off-grid power |
| Buck converter (12V→5V) | Steps 12V down to 5V for the logic + components |
| USB-to-RS485 adapter | For talking to the HESTIA directly from a PC (setup + diagnostics) |
| Weatherproof enclosure | Houses the build |
| Status LED | Simple output indicator |

## Wiring / pin map (Arduino Mega)

| Pin | Connection |
|---|---|
| `A0` | Battery voltage sense (via divider) |
| `D18 / D19` (Serial1) | → MAX485 → HESTIA A2 |
| `D22` | RS-485 DE + RE (jumpered together) |
| `D23` | Button (to GND) |
| `D24` | Outside DS18B20 (OneWire) |
| `D25` | Enclosure/box DS18B20 (OneWire) |
| `D26` | Status output LED |
| I²C (SDA/SCL) | 20×4 LCD at address `0x27` |

## Required libraries

Install these through the Arduino IDE Library Manager. **Author matters** — there are
same-named forks that won't behave the same:

- **LiquidCrystal I2C** by *Frank de Brabander* (uses `lcd.init()`)
- **OneWire** by *Paul Stoffregen*
- **DallasTemperature** by *Miles Burton* (the IDE will offer to pull OneWire as a dependency)

`Wire` and `math` are built into the Arduino core — nothing to install.

---

## The bug worth reading about: RS-485 self-echo

Most of this project's pain came from one problem: the HESTIA replies looked like garbage.
Every response came back as a burst of junk bytes followed by the real, valid Modbus frame.

The trap: it was **not** a wiring problem. The scope showed a clean signal and the bus was
biased. The garbage was the **Arduino hearing its own transmission.**

RS-485 here is **half-duplex** — one transceiver that can talk *or* listen, not both. While
the Arduino transmits, the MAX485's receiver output is tri-stated, so the Arduino's RX pin
floats — and its on-chip UART receiver was never disabled, so it framed a mangled echo of
its *own* outgoing message and stacked it in front of the slave's real reply. The dead
giveaway: the garbage burst was always exactly as long as the frame that had just been sent.

**The fix, two layers:**
1. **Mute the Arduino's UART receiver during transmit** (clear `RXEN1` on USART1) so there's
   nothing listening to record the echo, then re-enable it for the reply.
2. **Validate by CRC anyway** — the reader only accepts a frame that starts with the slave ID
   *and* ends with a valid CRC-16, as a backstop against any genuine line noise in the field.

With layer 1 in place, the raw reply dropped from 25 bytes (17 of echo + the real 8) to a
clean 8. If you're fighting the same "wrong slave ID" / garbage-reply wall on a half-duplex
RS-485 link, check whether your own UART is listening while you transmit before you go
chasing wiring.

## Field arming & data budget

You can't reflash the box once it's deployed, so the firmware has field controls:

- **Boots disarmed** — it will not send a single satellite packet until you deliberately
  **hold the button for 2 seconds** to arm it. This protects the tiny data budget from
  accidental transmits.
- Once armed, it sends a short **burst** for quick confirmation, then automatically slows
  to a long, steady cadence to stretch the data allowance.

There are also bench commands over USB serial (`o0` / `o1` / `o2`) to drive the output LED
for testing.

---

## What's in this repo

| File / folder | What it is |
|---|---|
| `YT_011_Firmware_V3_2/` | **Main operational firmware** — this is what runs the box. |
| `hestia_raw_probe/` | **Diagnostic sketch only** — fires the Modbus unlock frame and dumps every raw reply byte in hex. This is the tool that exposed the self-echo bug. **Not** the operational firmware. |
| `Schematic.pdf` | Full wiring schematic |
| `Harness Diagram.pdf` | Wiring harness layout |
| `Block Diagram V1.5.pdf` | System block diagram |
| `COMM Signal Chain.png` | The communication signal chain (Arduino → MAX485 → HESTIA) |
| `Test Harnesses (Schematic).pdf` | The two test harnesses I built (full-system, and HESTIA-alone) |

**A tip that saved me:** build **test harnesses first, not after you're stuck** — one for the
full system and one to talk to the HESTIA alone. When something breaks, you can answer
"is it the device or my integration?" without tearing the whole thing apart.

## A note on the A1/A2 labeling

Heads-up if you get one of these: the unit's housing, FCC ID, and firmware may report as
**"A1"** even though the box and listing say **A2**. I flagged this with APAL and deferred to
their official clarification rather than guessing — see the video description for their
response. Mentioning it here only so you're not thrown if you see the same thing.

## Manufacturer resources (APAL / Creative5)

The device's official tools and sample code live on APAL's GitHub — start here for anything
device-specific (register maps, firmware update tool, sample code):

- Org: https://github.com/CREATIVE5-io
- `Hestia-sample-Python` — Modbus sample for reading the device
- `Hestia-FW-Update-Python` — firmware update tool

---

## Credits

This project was built as part of a sponsored collaboration with **APAL**. The custom build,
firmware, wiring, and troubleshooting in this repo are my own, developed independently for
the demonstration. Product specs and official setup come from APAL's documentation.

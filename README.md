# SmartPod — ESP32 Bluetooth A2DP MP3 Player

A standalone, SD-card-based MP3 player built on the ESP32 that streams audio over Bluetooth A2DP to Bluetooth earbuds or speakers.

SmartPod features an OLED interface, SD-card music storage, smart shuffle, playback history, audio caching, adaptive buffering, and lightweight audio DSP.

Full Demo Link:
https://www.youtube.com/watch?v=pTDd5YzEN70

## Features

- **SD-card MP3 playback** — Music is stored locally on a microSD card.
- **On-demand playlist parsing** — The playlist is read from the SD card as needed for efficient RAM usage.
- **Smart Shuffle** — Intelligent shuffle behavior with persisted shuffle state across power cycles.
- **Playback History** — Chronological Previous/Back navigation independent of shuffle order.
- **Preload Engine + Smart Audio Cache** — Preloads the next track to reduce audible gaps between songs.
- **Adaptive Audio Queue** — Dynamically adjusts the audio queue when underruns occur.
- **Light Audio DSP** — Mild bass enhancement, peak limiting, and fade-in to reduce audible clicks.
- **Bluetooth A2DP** — Streams audio wirelessly to compatible Bluetooth earbuds or speakers.
- **Bluetooth Auto-Reconnect** — Automatically reconnects to the configured Bluetooth device.
- **Runtime Diagnostics** — Serial output provides memory and system diagnostics.
- **Resilient Playback** — Failed tracks are skipped instead of hanging playback.

## Hardware

| Component | Specification |
|---|---|
| Microcontroller | ESP32-WROOM-32 / ESP32 Dev Module |
| Display | SH1106 128x64 OLED |
| Storage | microSD card module |
| Audio Output | Bluetooth A2DP |
| Input | One momentary push button |

## Wiring

### OLED

| OLED | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | D21 |
| SCL | D22 |

### microSD

| SD Module | ESP32 |
|---|---|
| VCC | VIN / 5V |
| GND | GND |
| CS | D33 |
| SCK | D18 |
| MOSI | D23 |
| MISO | D19 |

### Button

One leg → **D13**

Other leg → **GND**

The firmware uses the ESP32 internal pull-up, so no external resistor is required.

## Required Libraries

Install these through the Arduino Library Manager:

- [ESP32-A2DP](https://github.com/pschatzmann/ESP32-A2DP)
- [AudioTools](https://github.com/pschatzmann/arduino-audio-tools)
- [arduino-libhelix](https://github.com/pschatzmann/arduino-libhelix)
- [U8g2](https://github.com/olikraus/u8g2)

## SD Card Setup

Format the microSD card as **FAT32**.

Place your music files and `playlist.txt` in the root directory:

```text
/
├── playlist.txt
├── 0001.mp3
├── 0002.mp3
├── 0003.mp3
└── ...

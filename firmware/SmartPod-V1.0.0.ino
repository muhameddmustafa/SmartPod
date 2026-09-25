/*
  SmartPod - ESP32 Bluetooth A2DP MP3 Player - www.linkedin.com/in/muhameddmustafa - Discord @therealmuhamed
   ---------------------------------------------
  Version: 1.0.0
  License: MIT (see LICENSE in the repository root)
  Board: ESP32-WROOM-32 / ESP32 Dev Module
  You must look at the lines 198 ~ 202 to adjust your earbuds + playlist name.

  Requires Libraries:
    - ESP32-A2DP >= 1.8.6
    - AudioTools
    - arduino-libhelix
    - U8g2

  Control model:
    Touch earbuds (AVRCP) are the primary control. One tap toggles
    play/pause; a configured gesture sends Next/Previous. The physical
    button is an optional backup and is not required for normal use.

  Features:
    - On-demand playlist.txt parsing (no Arduino String, no RAM cap on track count)
    - Smart Shuffle: never repeats the last-played track, steers recent plays away from the front of a new cycle, and survives a power cycle exactly via NVS-persisted seed/anchor/position
    - Real chronological play history for Previous (multi-step back)
    - Preload Engine + Smart Audio Cache: the next track is opened and cache-warmed ahead of time to remove the audible SD.open() gap
    - Adaptive audio queue that grows only when a track shows underruns
    - Mild bass enhancement with a peak limiter
    - Fade-in on track start / pause-resume
    - Bluetooth auto-reconnect, exact-name pairing, remote volume ignored (forced to maximum)
    - Runtime RAM diagnostics over Serial
    - Mutex-protected SD/decoder access between the audio and UI paths
    - Per-track failures no longer hang the device: logged, shown briefly, and playback moves on to the next track

  SD card root:
    playlist.txt
    0001.mp3
    0002.mp3
    ...

  playlist.txt format:
    0001.mp3|Track Title

  Hardware:
    OLED VCC -> 3V3        SD 5V   -> VIN / 5V
    OLED GND -> GND        SD GND  -> GND
    OLED SDA -> D21        SD CS   -> D33
    OLED SCL -> D22        SD SCK  -> D18
                           SD MOSI -> D23
    Button:                SD MISO -> D19
    D13 <-> button <-> GND

  Recommended audio format: MP3, stereo, 44.1 kHz, 192-320 kbps.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <new>

#include "AudioTools.h"
#include "AudioTools/AudioCodecs/CodecMP3Helix.h"
#include "BluetoothA2DPSource.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_avrc_api.h"


// ==================== EMBEDDED SPLASH ====================
// 128x64 XBM bitmap, stored in flash (PROGMEM), not RAM. This is the cat example You can use any image but convert it first into bitmap.
constexpr uint16_t SPLASH_WIDTH = 128;
constexpr uint16_t SPLASH_HEIGHT = 64;

const uint8_t catSplashBitmap[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x0F, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF6, 0x03, 0x20, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x03, 0xF0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x02, 0xD8, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x02, 0xEC, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFE, 0xE7, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x03, 0xFE, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0xF0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0xC0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB8, 0x8F, 0x8D, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x07, 0x87, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x78, 0xC0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x00, 0xE0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0x00, 0x38, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x03, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0xFF, 0x73, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCC, 0x00, 0x18, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE6, 0x01, 0x38, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB3, 0x01, 0x68, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x8F, 0x01, 0xC8, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x87, 0x01, 0x8C, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x83, 0x01, 0x0C, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC7, 0x0F, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 0x0F, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xFF, 0x6F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE7, 0x9F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0xE3, 0x67, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0xC1, 0xC3, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41, 0x86, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x7F, 0x8F, 0x05, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xC1, 0xFE, 0x1F, 0x8E, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x60, 0x83, 0xFD, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE0, 0x81, 0x03, 0x0C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x03, 0x40, 0x00, 0x80, 0x8F, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x08, 0x0C, 0x04, 0xFE, 0xFF, 0xFF, 0x1F, 0x8C, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x0C, 0x06, 0x04, 0xF8, 0x03, 0x3E, 0xF0, 0xC7, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x06, 0x03, 0xFC, 0x00, 0x00, 0x00, 0x80, 0xC3, 0x80, 0x31, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x01, 0xFC, 0x01, 0x00, 0x00, 0x00, 0x61, 0x00, 0x63, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x03, 0x78, 0x60, 0x00, 0x00, 0x8F, 0x31, 0x00, 0x43, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x06, 0x06, 0xE0, 0xFF, 0x0F, 0x7C, 0xFC, 0x1F, 0x00, 0x41, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x1C, 0x1C, 0x00, 0x00, 0xF8, 0xC0, 0x03, 0x0E, 0xC0, 0x61, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x80, 0x1F, 0xE2, 0x03, 0x70, 0x3C, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x10, 0x0C, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static_assert(
  sizeof(catSplashBitmap) == ((SPLASH_WIDTH + 7) / 8) * SPLASH_HEIGHT,
  "Splash bitmap size is incorrect"
);

// ==================== PINS ====================
constexpr uint8_t OLED_SDA = 21;       // D21
constexpr uint8_t OLED_SCL = 22;       // D22

constexpr uint8_t SD_CS   = 33;        // D33
constexpr uint8_t SD_SCK  = 18;        // D18
constexpr uint8_t SD_MISO = 19;        // D19
constexpr uint8_t SD_MOSI = 23;        // D23

constexpr uint8_t BTN_PLAY_PAUSE = 13; // D13

// ==================== SETTINGS ====================
constexpr uint8_t PATH_LEN    = 32;
constexpr uint8_t TITLE_LEN   = 42;
constexpr uint8_t START_VOLUME = 127;

// Smart Shuffle: recently-played tracks kept out of a fresh cycle's front.
constexpr uint8_t SHUFFLE_HISTORY_SIZE = 5;

// Real play-order history for Previous (separate from the shuffle history above).
constexpr uint8_t PLAY_HISTORY_SIZE = 20;

// Sentinel for "no anchor track" in NVS (which stores it as unsigned).
constexpr uint16_t SHUFFLE_ANCHOR_NONE = 0xFFFF;

// Adaptive audio queue: starts small, grows in steps only on underruns.
constexpr size_t AUDIO_QUEUE_MIN_BYTES  = 8192;
constexpr size_t AUDIO_QUEUE_MAX_BYTES  = 16384;
constexpr size_t AUDIO_QUEUE_STEP_BYTES = 2048;
constexpr uint8_t UNDERRUN_GROW_THRESHOLD = 3;

// Preload cache warm-up read size.
constexpr size_t CACHE_WARMUP_BYTES = 512;

constexpr uint32_t SD_SPI_FREQUENCY_HZ = 20000000; // falls back to 4 MHz if unsupported
constexpr uint32_t OLED_I2C_FREQUENCY_HZ = 400000;  // I2C fast-mode

// Fade-in length on track start / pause-resume (~11.6 ms @ 44.1 kHz).
constexpr int32_t FADE_IN_FRAMES = 512;

constexpr uint32_t MEMORY_LOG_INTERVAL_MS = 60000;

// DSP gain settings (Q8 fixed point; 256 = unity gain).
constexpr int32_t MASTER_GAIN_Q8 = 256;
constexpr int32_t BASS_GAIN_Q8   = 4;
constexpr uint8_t BASS_FILTER_SHIFT = 5;

const char PLAYLIST_PATH[] = "/playlist.txt";            
 // VERY IMPORTANT >> THIS IS THE TXT FILE NAME you can change it in the code according to your prefrence if you wish.
const char EXACT_BT_NAME[] = "soundcore R50i NC";     
    // VERY IMPORTANT >> This is the name of my bluetooth earbuds, You SHALL CHANGE IT TO YOUR
    // EXACT Bluetooth device name. Change this to the name of your earbuds/speaker.

// ==================== OLED ====================
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(
  U8G2_R0,
  U8X8_PIN_NONE,
  OLED_SCL,
  OLED_SDA
);

// Small bitmaps kept in flash (PROGMEM), not RAM.
const unsigned char heartIcon[] PROGMEM = {
  0x00, 0x00, 0x9C, 0x03, 0xFE, 0x07, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F,
  0xFE, 0x07, 0xFC, 0x03, 0xF8, 0x01, 0xF0, 0x01, 0xE0, 0x00, 0x40, 0x00,
};

const unsigned char starIcon[] PROGMEM = {
  0x40, 0x00, 0x40, 0x00, 0x60, 0x00, 0xE0, 0x00, 0xFF, 0x0F, 0xFC, 0x03,
  0xF8, 0x01, 0xF8, 0x01, 0xF8, 0x01, 0x1C, 0x03, 0x04, 0x02, 0x00, 0x00,
};

const unsigned char noteIcon[] PROGMEM = {
  0x0C, 0x0E, 0x0A, 0x0A, 0x6A, 0x7A, 0x32, 0x00
};

// ==================== TRACK DATA ====================
// On-demand playlist: tracks are parsed off playlist.txt as needed
// instead of being cached in a RAM array.
uint16_t trackCount = 0;

uint16_t* shuffleOrder = nullptr;
uint16_t shufflePosition = 0;
int32_t currentTrack = -1;
char currentTrackTitle[TITLE_LEN] = "";

// Smart Shuffle: ring buffer of recently-played track indices, used to
// steer a fresh shuffle order away from repeating them near the front.
uint16_t shuffleHistory[SHUFFLE_HISTORY_SIZE];
uint8_t shuffleHistoryCount = 0;
uint8_t shuffleHistoryNext = 0;

// ==================== SHUFFLE CYCLE STATE ====================
// Seed/anchor for the currently active shuffle cycle, persisted to NVS
// so a power cycle can reproduce the exact same order.
uint32_t activeShuffleSeed = 0;
int32_t shuffleAnchorTrack = -1;

// Position loaded from NVS at boot, applied after resumeShuffleCycle()
// rebuilds the order.
uint16_t pendingShufflePosition = 0;

// ==================== PLAY HISTORY ====================
// Chronological record of the last PLAY_HISTORY_SIZE tracks played
// (oldest at [0]); RAM-only, not persisted.
uint16_t playHistory[PLAY_HISTORY_SIZE];
uint8_t playHistoryCount = 0;

// How many Previous presses deep we are; reset on any forward play.
uint8_t historyBackSteps = 0;

// ==================== AUDIO ====================
File songFile;
MP3DecoderHelix mp3Decoder;
EncodedAudioStream decodedAudio(&songFile, &mp3Decoder);
BluetoothA2DPSource a2dpSource;

SemaphoreHandle_t audioMutex = nullptr;

// ==================== PRELOAD ENGINE ====================
// The next shuffled track, already opened and cache-warmed.
File preloadedFile;
int32_t preloadedTrackIndex = -1;
char preloadedTrackTitle[TITLE_LEN] = "";

// ==================== ADAPTIVE QUEUE STATE ====================
size_t activeQueueBytes = AUDIO_QUEUE_MIN_BYTES;
uint8_t underrunsThisTrack = 0;

// Flags written by Bluetooth/audio tasks and consumed by loop().
volatile bool btConnected = false;
volatile bool audioStarted = false;
volatile bool isPaused = false;
volatile bool switchingTrack = false;
volatile bool nextTrackRequested = false;
volatile bool previousTrackRequested = false;
volatile bool togglePlaybackRequested = false;
volatile bool forceMaxVolumeRequested = false;
volatile bool pairedScreenRequested = false;
volatile bool uiDirty = false;

volatile uint8_t consecutiveEmptyReads = 0;

// Counts down from FADE_IN_FRAMES on a fresh PCM onset; 0 = full gain.
volatile int32_t fadeInFramesRemaining = 0;

uint32_t lastMemoryLogMs = 0;

constexpr uint32_t PAIRED_SCREEN_MS = 1400;
bool pairedScreenActive = false;
uint32_t pairedScreenEndsAt = 0;

// One-pole low-frequency filter state (left/right channels).
int32_t bassLowLeft = 0;
int32_t bassLowRight = 0;

// ==================== BUTTON STATE ====================
constexpr uint32_t DEBOUNCE_MS = 60;

bool lastRawButtonState = HIGH;
bool stableButtonState = HIGH;
uint32_t lastButtonChangeMs = 0;

// ==================== MEMORY DIAGNOSTICS ====================
void printResetReason() {
  const esp_reset_reason_t reason = esp_reset_reason();

  Serial.print("[RESET] reason = ");
  Serial.print(static_cast<int>(reason));

  if (reason == ESP_RST_BROWNOUT) {
    Serial.print(" (BROWNOUT / power dip)");
  } else if (reason == ESP_RST_TASK_WDT ||
             reason == ESP_RST_INT_WDT ||
             reason == ESP_RST_WDT) {
    Serial.print(" (WATCHDOG)");
  } else if (reason == ESP_RST_PANIC) {
    Serial.print(" (PANIC / software crash)");
  } else if (reason == ESP_RST_POWERON) {
    Serial.print(" (POWER ON)");
  }

  Serial.println();
}

void printMemory(const char* stage) {
  const size_t freeHeap =
    heap_caps_get_free_size(MALLOC_CAP_8BIT);

  const size_t minimumFreeHeap =
    heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);

  const size_t largestBlock =
    heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

  Serial.print("[RAM] ");
  Serial.print(stage);
  Serial.print(" | free=");
  Serial.print(freeHeap);
  Serial.print(" | minimum=");
  Serial.print(minimumFreeHeap);
  Serial.print(" | largest=");
  Serial.println(largestBlock);

  if (minimumFreeHeap < 30000) {
    Serial.println(
      "[RAM WARNING] Minimum free heap is below 30 KB"
    );
  }
}

// ==================== UI ====================
void showSplash() {
  oled.clearBuffer();

  oled.drawXBMP(
    0,
    0,
    SPLASH_WIDTH,
    SPLASH_HEIGHT,
    catSplashBitmap
  );

  oled.sendBuffer();
}

void drawPairedScreen() {
  oled.clearBuffer();

  oled.setFont(u8g2_font_9x15B_tr);
  const char* pairedText = "Paired";
  const int pairedWidth = oled.getStrWidth(pairedText);
  oled.drawStr(
    max(0, (128 - pairedWidth) / 2),
    18,
    pairedText
  );

  // Large filled heart made from two circles and a triangle.
  oled.drawDisc(53, 36, 10);
  oled.drawDisc(75, 36, 10);
  oled.drawTriangle(
    43, 38,
    85, 38,
    64, 62
  );

  oled.sendBuffer();
}

void makeShortTitle(const char* source,
                    char* output,
                    size_t outputSize) {
  constexpr size_t MAX_VISIBLE = 13;

  if (outputSize == 0) {
    return;
  }

  if (strlen(source) <= MAX_VISIBLE || outputSize < 4) {
    strncpy(output, source, outputSize - 1);
    output[outputSize - 1] = '\0';
    return;
  }

  const size_t copyLength =
    min(MAX_VISIBLE - 3, outputSize - 4);

  memcpy(output, source, copyLength);
  output[copyLength] = '.';
  output[copyLength + 1] = '.';
  output[copyLength + 2] = '.';
  output[copyLength + 3] = '\0';
}

const char* currentTitle() {
  if (currentTrack < 0) {
    return "";
  }

  // Filled once per track switch in openTrack() - no playlist re-read here.
  return currentTrackTitle;
}

// Line-drawn Bluetooth glyph (avoids an extra PROGMEM bitmap).
void drawBtIcon(uint8_t x, uint8_t y, bool connectedSnapshot) {
  if (!connectedSnapshot) {
    return;
  }

  oled.drawLine(x + 2, y,     x + 2, y + 8);
  oled.drawLine(x + 2, y,     x + 6, y + 3);
  oled.drawLine(x + 6, y + 3, x,     y + 5);
  oled.drawLine(x,     y + 5, x + 6, y + 6);
  oled.drawLine(x + 6, y + 6, x + 2, y + 8);
}

// Line-drawn Smart Shuffle glyph.
void drawShuffleIcon(uint8_t x, uint8_t y) {
  oled.drawLine(x, y + 1, x + 6, y + 6);
  oled.drawLine(x, y + 6, x + 6, y + 1);
  oled.drawBox(x + 5, y,     2, 2);
  oled.drawBox(x + 5, y + 5, 2, 2);
}

void drawPlayPauseSymbol(bool pausedSnapshot) {
  // Shows the action a tap performs: Play triangle while paused, Pause bars while playing.
  if (pausedSnapshot) {
    oled.drawTriangle(
      3, 54,
      3, 61,
      12, 57
    );
  } else {
    oled.drawBox(3, 54, 3, 7);
    oled.drawBox(8, 54, 3, 7);
  }
}

void drawPlayerScreen() {
  if (currentTrack < 0) {
    return;
  }

  // Single state snapshot so header, icon and progress bar stay consistent.
  const bool pausedSnapshot = isPaused;
  const bool connectedSnapshot = btConnected;
  const uint8_t backStepsSnapshot = historyBackSteps;
  const uint16_t shufflePositionSnapshot = shufflePosition;

  char shortTitle[24];

  makeShortTitle(
    currentTitle(),
    shortTitle,
    sizeof(shortTitle)
  );

  oled.clearBuffer();

  // ---- Header: shuffle glyph, state label, Bluetooth glyph ----
  drawShuffleIcon(2, 2);

  oled.setFont(u8g2_font_6x13B_tr);
  oled.drawStr(
    13,
    11,
    backStepsSnapshot > 0 ?
      "HISTORY" : (pausedSnapshot ? "PAUSED" : "PLAYING")
  );

  drawBtIcon(116, 1, connectedSnapshot);
  oled.drawHLine(0, 12, 128);

  // ---- Centered song title ----
  oled.setFont(u8g2_font_9x15B_tr);

  const int titleWidth =
    oled.getStrWidth(shortTitle);

  const int titleX =
    max(0, (128 - titleWidth) / 2);

  oled.drawStr(titleX, 30, shortTitle);

  // ---- Subtitle line: which track this is in the library ----
  char subtitle[24];

  snprintf(
    subtitle,
    sizeof(subtitle),
    "Track %u of %u",
    static_cast<unsigned>(currentTrack + 1),
    static_cast<unsigned>(trackCount)
  );

  oled.setFont(u8g2_font_6x12_tr);

  const int subtitleWidth =
    oled.getStrWidth(subtitle);

  oled.drawStr(max(0, (128 - subtitleWidth) / 2), 43, subtitle);

  oled.drawHLine(0, 45, 128);

  // ---- Footer: Smart Shuffle cycle progress bar ----
  oled.drawFrame(4, 47, 120, 6);

  const uint16_t filledPixels =
    trackCount > 0 ?
      static_cast<uint16_t>(
        (118UL * shufflePositionSnapshot) / trackCount
      ) : 0;

  if (filledPixels > 0) {
    oled.drawBox(5, 48, filledPixels, 4);
  }

  drawPlayPauseSymbol(pausedSnapshot);

  oled.setFont(u8g2_font_5x8_tr);

  if (backStepsSnapshot > 0) {
    oled.drawTriangle(
      46, 61,
      46, 55,
      41, 58
    );

    char backLabel[6];
    snprintf(backLabel, sizeof(backLabel), "-%u", backStepsSnapshot);
    oled.drawStr(50, 61, backLabel);
  } else {
    oled.drawStr(42, 61, "SHUFFLE");
  }

  char trackInfo[12];

  snprintf(
    trackInfo,
    sizeof(trackInfo),
    "%u/%u",
    static_cast<unsigned>(currentTrack + 1),
    static_cast<unsigned>(trackCount)
  );

  oled.drawStr(98, 61, trackInfo);

  oled.sendBuffer();
}

void drawSimpleScreen(const char* line1,
                      const char* line2 = "",
                      const char* line3 = "") {
  oled.clearBuffer();

  oled.setFont(u8g2_font_6x13B_tr);
  oled.drawStr(2, 15, line1);

  oled.setFont(u8g2_font_6x12_tr);
  oled.drawStr(2, 37, line2);
  oled.drawStr(2, 58, line3);

  oled.sendBuffer();
}

// ==================== PLAYLIST PARSER ====================
char* skipUtf8Bom(char* line) {
  if (
    strlen(line) >= 3 &&
    static_cast<uint8_t>(line[0]) == 0xEF &&
    static_cast<uint8_t>(line[1]) == 0xBB &&
    static_cast<uint8_t>(line[2]) == 0xBF
  ) {
    return line + 3;
  }

  return line;
}

// Validates one playlist.txt line and optionally fills in its path/title.
// Pass nullptr/0 for pathOut/titleOut to only check validity.
bool parsePlaylistLine(char* originalLine,
                       char* pathOut, size_t pathOutSize,
                       char* titleOut, size_t titleOutSize) {
  char* line = skipUtf8Bom(originalLine);
  char* separator = strchr(line, '|');

  if (!separator) {
    return false;
  }

  *separator = '\0';

  char* fileName = line;
  char* displayTitle = separator + 1;

  while (*fileName == ' ' || *fileName == '\t') {
    ++fileName;
  }

  while (*displayTitle == ' ' || *displayTitle == '\t') {
    ++displayTitle;
  }

  char* end = fileName + strlen(fileName);

  while (
    end > fileName &&
    (
      end[-1] == ' ' ||
      end[-1] == '\t' ||
      end[-1] == '\r'
    )
  ) {
    *--end = '\0';
  }

  end = displayTitle + strlen(displayTitle);

  while (
    end > displayTitle &&
    (
      end[-1] == ' ' ||
      end[-1] == '\t' ||
      end[-1] == '\r'
    )
  ) {
    *--end = '\0';
  }

  if (!*fileName || !*displayTitle) {
    return false;
  }

  char fullPath[PATH_LEN];

  const int written = snprintf(
    fullPath,
    sizeof(fullPath),
    fileName[0] == '/' ? "%s" : "/%s",
    fileName
  );

  if (written < 0 ||
      written >= static_cast<int>(sizeof(fullPath))) {
    Serial.println("Skipped: filename too long");
    return false;
  }

  if (!SD.exists(fullPath)) {
    Serial.print("Missing file: ");
    Serial.println(fullPath);
    return false;
  }

  if (strlen(displayTitle) >= TITLE_LEN) {
    Serial.println("Skipped: title too long");
    return false;
  }

  if (pathOut && pathOutSize > 0) {
    strncpy(pathOut, fullPath, pathOutSize - 1);
    pathOut[pathOutSize - 1] = '\0';
  }

  if (titleOut && titleOutSize > 0) {
    strncpy(titleOut, displayTitle, titleOutSize - 1);
    titleOut[titleOutSize - 1] = '\0';
  }

  return true;
}

// Counts valid entries in playlist.txt without storing them.
uint16_t countPlaylistTracks() {
  File playlistFile =
    SD.open(PLAYLIST_PATH, FILE_READ);

  if (!playlistFile) {
    return 0;
  }

  uint16_t count = 0;
  char line[96];
  uint8_t position = 0;
  bool lineOverflow = false;

  while (playlistFile.available()) {
    const char character =
      static_cast<char>(playlistFile.read());

    if (character == '\n') {
      line[position] = '\0';

      if (!lineOverflow &&
          position > 0 &&
          line[0] != '#') {
        if (parsePlaylistLine(line, nullptr, 0, nullptr, 0)) {
          ++count;
        }
      }

      position = 0;
      lineOverflow = false;
      continue;
    }

    if (position < sizeof(line) - 1) {
      line[position++] = character;
    } else {
      lineOverflow = true;
    }
  }

  if (position > 0 && !lineOverflow) {
    line[position] = '\0';

    if (line[0] != '#') {
      if (parsePlaylistLine(line, nullptr, 0, nullptr, 0)) {
        ++count;
      }
    }
  }

  playlistFile.close();

  Serial.print("Valid playlist tracks: ");
  Serial.println(count);

  return count;
}

// On-demand lookup: re-scans playlist.txt for the Nth valid entry.
bool getTrackByIndex(uint16_t targetIndex,
                     char* pathOut, size_t pathOutSize,
                     char* titleOut, size_t titleOutSize) {
  File playlistFile =
    SD.open(PLAYLIST_PATH, FILE_READ);

  if (!playlistFile) {
    return false;
  }

  uint16_t seen = 0;
  char line[96];
  uint8_t position = 0;
  bool lineOverflow = false;
  bool found = false;

  while (playlistFile.available() && !found) {
    const char character =
      static_cast<char>(playlistFile.read());

    if (character == '\n') {
      line[position] = '\0';

      if (!lineOverflow &&
          position > 0 &&
          line[0] != '#') {
        if (seen == targetIndex) {
          found = parsePlaylistLine(
            line, pathOut, pathOutSize, titleOut, titleOutSize
          );
        } else if (parsePlaylistLine(line, nullptr, 0, nullptr, 0)) {
          ++seen;
        }
      }

      position = 0;
      lineOverflow = false;
      continue;
    }

    if (position < sizeof(line) - 1) {
      line[position++] = character;
    } else {
      lineOverflow = true;
    }
  }

  if (!found && position > 0 && !lineOverflow) {
    line[position] = '\0';

    if (line[0] != '#' && seen == targetIndex) {
      found = parsePlaylistLine(
        line, pathOut, pathOutSize, titleOut, titleOutSize
      );
    }
  }

  playlistFile.close();
  return found;
}

// ==================== SHUFFLE ====================
bool isInShuffleHistory(uint16_t track) {
  for (uint8_t i = 0; i < shuffleHistoryCount; ++i) {
    if (shuffleHistory[i] == track) {
      return true;
    }
  }

  return false;
}

void rememberPlayedTrack(uint16_t track) {
  shuffleHistory[shuffleHistoryNext] = track;
  shuffleHistoryNext =
    (shuffleHistoryNext + 1) % SHUFFLE_HISTORY_SIZE;

  if (shuffleHistoryCount < SHUFFLE_HISTORY_SIZE) {
    ++shuffleHistoryCount;
  }
}

// Fisher-Yates shuffle, then a pass that swaps recently-played tracks
// out of the leading slots so a fresh cycle doesn't immediately repeat them.
void buildShuffleOrder() {
  if (trackCount == 0 || !shuffleOrder) {
    shufflePosition = 0;
    return;
  }

  for (uint16_t i = 0; i < trackCount; ++i) {
    shuffleOrder[i] = i;
  }

  for (int32_t i = static_cast<int32_t>(trackCount) - 1; i > 0; --i) {
    const uint16_t randomIndex =
      random(i + 1);

    const uint16_t temporary =
      shuffleOrder[i];

    shuffleOrder[i] =
      shuffleOrder[randomIndex];

    shuffleOrder[randomIndex] =
      temporary;
  }

  // Leading slots to protect from recently-played tracks.
  const uint16_t guardWindow =
    min(
      static_cast<uint16_t>(shuffleHistoryCount),
      static_cast<uint16_t>(trackCount > 1 ? trackCount - 1 : 0)
    );

  for (uint16_t i = 0; i < guardWindow; ++i) {
    if (!isInShuffleHistory(shuffleOrder[i])) {
      continue;
    }

    // Swap in the closest later slot that isn't a recently-played track.
    for (uint16_t j = i + 1; j < trackCount; ++j) {
      if (!isInShuffleHistory(shuffleOrder[j])) {
        const uint16_t temporary = shuffleOrder[i];
        shuffleOrder[i] = shuffleOrder[j];
        shuffleOrder[j] = temporary;
        break;
      }
    }
  }

  // Never start a new cycle with the track that just finished.
  if (
    trackCount > 1 &&
    shuffleAnchorTrack >= 0 &&
    shuffleOrder[0] ==
      static_cast<uint16_t>(shuffleAnchorTrack)
  ) {
    const uint16_t temporary =
      shuffleOrder[0];

    shuffleOrder[0] =
      shuffleOrder[1];

    shuffleOrder[1] =
      temporary;
  }

  shufflePosition = 0;
}

// Starts a new Smart Shuffle cycle from a fresh random seed.
void beginNewShuffleCycle() {
  activeShuffleSeed = esp_random();
  shuffleAnchorTrack = currentTrack;

  randomSeed(activeShuffleSeed);
  buildShuffleOrder();
}

// Reproduces the shuffle cycle active before the last power-off.
void resumeShuffleCycle() {
  randomSeed(activeShuffleSeed);
  buildShuffleOrder();
}

uint16_t getNextShuffledTrack() {
  if (shufflePosition >= trackCount) {
    beginNewShuffleCycle();
  }

  return shuffleOrder[shufflePosition++];
}

// ==================== PLAY HISTORY ====================
// Records a track as the newest history entry; only forward plays call
// this, so historyBackSteps survives repeated Previous presses correctly.
void pushPlayHistory(uint16_t track) {
  if (playHistoryCount == PLAY_HISTORY_SIZE) {
    memmove(
      playHistory,
      playHistory + 1,
      (PLAY_HISTORY_SIZE - 1) * sizeof(uint16_t)
    );
    --playHistoryCount;
  }

  playHistory[playHistoryCount++] = track;
  historyBackSteps = 0;
}

// ==================== SHUFFLE PERSISTENCE ====================
// Saves what's needed to resume Smart Shuffle exactly after a power
// cycle; ignored if playlist.txt has since changed size.
Preferences shufflePrefs;

void saveShuffleState() {
  if (currentTrack < 0) {
    return;
  }

  shufflePrefs.begin("smartipod", false);
  shufflePrefs.putUShort("lastTrack", static_cast<uint16_t>(currentTrack));
  shufflePrefs.putUShort("trackCount", trackCount);
  shufflePrefs.putUInt("shufSeed", activeShuffleSeed);
  shufflePrefs.putUShort(
    "shufAnchor",
    shuffleAnchorTrack >= 0 ?
      static_cast<uint16_t>(shuffleAnchorTrack) :
      SHUFFLE_ANCHOR_NONE
  );
  shufflePrefs.putUShort("shufPos", shufflePosition);
  shufflePrefs.end();
}

// Returns true when a resumable shuffle cycle was found in NVS.
bool loadShuffleState() {
  shufflePrefs.begin("smartipod", true);

  const uint16_t savedTrackCount =
    shufflePrefs.getUShort("trackCount", 0);

  const uint16_t savedLastTrack =
    shufflePrefs.getUShort("lastTrack", 0);

  const uint32_t savedSeed =
    shufflePrefs.getUInt("shufSeed", 0);

  const uint16_t savedAnchor =
    shufflePrefs.getUShort("shufAnchor", SHUFFLE_ANCHOR_NONE);

  const uint16_t savedPosition =
    shufflePrefs.getUShort("shufPos", 0);

  shufflePrefs.end();

  if (savedTrackCount != trackCount || trackCount == 0) {
    return false; // playlist changed size since the last save - don't trust it
  }

  if (savedLastTrack < trackCount) {
    rememberPlayedTrack(savedLastTrack);
  }

  activeShuffleSeed = savedSeed;

  shuffleAnchorTrack =
    (savedAnchor == SHUFFLE_ANCHOR_NONE || savedAnchor >= trackCount) ?
      -1 : static_cast<int32_t>(savedAnchor);

  pendingShufflePosition =
    (savedPosition <= trackCount) ? savedPosition : 0;

  Serial.println("[SHUFFLE] Restored last shuffle state");
  return true;
}

// ==================== PRELOAD ENGINE ====================
void closePreload() {
  if (preloadedFile) {
    preloadedFile.close();
  }

  preloadedFile = File();
  preloadedTrackIndex = -1;
  preloadedTrackTitle[0] = '\0';
}

// Opens and cache-warms the track getNextShuffledTrack() will hand out
// next, without consuming it. No-ops once something is already preloaded.
void preloadNextTrack() {
  if (
    trackCount == 0 ||
    switchingTrack ||
    preloadedTrackIndex >= 0
  ) {
    return;
  }

  // Rebuild now if the shuffle order is about to wrap, so the preload
  // below still targets the right (freshly-built) next track.
  if (shufflePosition >= trackCount) {
    beginNewShuffleCycle();
  }

  const uint16_t candidate = shuffleOrder[shufflePosition];

  if (xSemaphoreTake(audioMutex, 0) != pdTRUE) {
    return; // playback path is busy right now; try again next loop tick
  }

  // Looked up under audioMutex since it shares the SD bus with provideAudio().
  char candidatePath[PATH_LEN];
  char candidateTitle[TITLE_LEN];

  if (
    getTrackByIndex(
      candidate,
      candidatePath, sizeof(candidatePath),
      candidateTitle, sizeof(candidateTitle)
    )
  ) {
    File file = SD.open(candidatePath, FILE_READ);

    if (file) {
      // Warm the SD cache, then rewind to the real start.
      uint8_t warmupBuffer[CACHE_WARMUP_BYTES];
      file.read(warmupBuffer, sizeof(warmupBuffer));
      file.seek(0);

      preloadedFile = file;
      preloadedTrackIndex = static_cast<int32_t>(candidate);

      strncpy(preloadedTrackTitle, candidateTitle, sizeof(preloadedTrackTitle) - 1);
      preloadedTrackTitle[sizeof(preloadedTrackTitle) - 1] = '\0';
    }
  }

  xSemaphoreGive(audioMutex);
}

// ==================== TRACK CONTROL ====================
bool openTrack(uint16_t requestedTrack,
               bool rememberCurrent = true) {
  if (requestedTrack >= trackCount) {
    return false;
  }

  switchingTrack = true;

  if (
    xSemaphoreTake(
      audioMutex,
      pdMS_TO_TICKS(1500)
    ) != pdTRUE
  ) {
    switchingTrack = false;
    return false;
  }

  File nextFile;
  bool gaplessSwitch = false;
  char requestedTitle[TITLE_LEN];
  requestedTitle[0] = '\0';

  if (
    preloadedTrackIndex >= 0 &&
    static_cast<uint16_t>(preloadedTrackIndex) == requestedTrack &&
    preloadedFile
  ) {
    // Gapless switch: file already open and cache-warmed by the preload.
    nextFile = preloadedFile;
    preloadedFile = File();   // release our handle; nextFile now owns it
    preloadedTrackIndex = -1;
    gaplessSwitch = true;

    strncpy(requestedTitle, preloadedTrackTitle, sizeof(requestedTitle) - 1);
    requestedTitle[sizeof(requestedTitle) - 1] = '\0';
  } else {
    // Preload missed this request - fall back to a synchronous lookup + open.
    char requestedPath[PATH_LEN];

    if (
      getTrackByIndex(
        requestedTrack,
        requestedPath, sizeof(requestedPath),
        requestedTitle, sizeof(requestedTitle)
      )
    ) {
      nextFile = SD.open(requestedPath, FILE_READ);
    }
  }

  // Only a forward play (Next/shuffle/boot) invalidates the preload -
  // Previous never advances shufflePosition, so a pending preload is
  // still correct for the next Next press.
  if (!gaplessSwitch && rememberCurrent) {
    closePreload();
  }

  if (!nextFile) {
    xSemaphoreGive(audioMutex);
    switchingTrack = false;
    return false;
  }

  decodedAudio.end();

  if (songFile) {
    songFile.close();
  }

  songFile = nextFile;

  // Only grow the queue if the previous track actually showed underruns.
  if (
    underrunsThisTrack >= UNDERRUN_GROW_THRESHOLD &&
    activeQueueBytes < AUDIO_QUEUE_MAX_BYTES
  ) {
    activeQueueBytes =
      min(activeQueueBytes + AUDIO_QUEUE_STEP_BYTES, AUDIO_QUEUE_MAX_BYTES);

    Serial.print("[ADAPTIVE QUEUE] Growing to ");
    Serial.println(activeQueueBytes);
  }

  underrunsThisTrack = 0;

  decodedAudio
    .transformationReader()
    .resizeResultQueue(activeQueueBytes);

  if (!decodedAudio.begin()) {
    songFile.close();
    currentTrack = -1;
    currentTrackTitle[0] = '\0';
    uiDirty = true;
    xSemaphoreGive(audioMutex);
    switchingTrack = false;
    return false;
  }

  currentTrack =
    static_cast<int32_t>(requestedTrack);

  strncpy(currentTrackTitle, requestedTitle, sizeof(currentTrackTitle) - 1);
  currentTrackTitle[sizeof(currentTrackTitle) - 1] = '\0';

  rememberPlayedTrack(static_cast<uint16_t>(currentTrack));

  // Only forward plays push a history entry - Previous re-opens an
  // existing entry and must not reset historyBackSteps.
  if (rememberCurrent) {
    pushPlayHistory(static_cast<uint16_t>(currentTrack));
  }

  resetBassFilter();
  consecutiveEmptyReads = 0;
  nextTrackRequested = false;
  audioStarted = false;
  fadeInFramesRemaining = FADE_IN_FRAMES;

  xSemaphoreGive(audioMutex);

  switchingTrack = false;
  uiDirty = true;

  Serial.print("Playing track #");
  Serial.print(currentTrack);
  Serial.print(" | ");
  Serial.println(currentTrackTitle);

  saveShuffleState();

  return true;
}

bool openNextAvailableTrack() {
  for (uint16_t attempt = 0;
       attempt < trackCount;
       ++attempt) {
    if (
      openTrack(
        getNextShuffledTrack(),
        true
      )
    ) {
      return true;
    }
  }

  return false;
}

// Steps backward through real play-order history, one press per step.
bool openPreviousTrack() {
  if (
    playHistoryCount < 2 ||
    historyBackSteps >= static_cast<uint8_t>(playHistoryCount - 1)
  ) {
    return false;
  }

  const uint8_t nextBackSteps =
    static_cast<uint8_t>(historyBackSteps + 1);

  const uint16_t target =
    playHistory[playHistoryCount - 1 - nextBackSteps];

  if (openTrack(target, false)) {
    historyBackSteps = nextBackSteps;
    return true;
  }

  return false;
}

// ==================== AUDIO DSP ====================
int16_t limitPcmSample(int32_t sample) {
  // Normal samples pass unchanged; only very high peaks are compressed.
  constexpr int32_t LIMIT_START = 30000;

  if (sample > LIMIT_START) {
    sample =
      LIMIT_START +
      ((sample - LIMIT_START) / 4);
  } else if (sample < -LIMIT_START) {
    sample =
      -LIMIT_START +
      ((sample + LIMIT_START) / 4);
  }

  if (sample > 32767) {
    return 32767;
  }

  if (sample < -32768) {
    return -32768;
  }

  return static_cast<int16_t>(sample);
}

void resetBassFilter() {
  bassLowLeft = 0;
  bassLowRight = 0;
}

// Ramps gain 0 -> full over FADE_IN_FRAMES after a fresh PCM onset, to
// avoid an audible click on track start / pause-resume.
void applyFadeIn(uint8_t* data, int32_t byteCount) {
  if (fadeInFramesRemaining <= 0) {
    return;
  }

  int16_t* pcm = reinterpret_cast<int16_t*>(data);
  const int32_t frameCount = byteCount / 4; // 16-bit stereo frames

  for (int32_t frame = 0;
       frame < frameCount && fadeInFramesRemaining > 0;
       ++frame) {
    const int32_t gainQ8 =
      ((FADE_IN_FRAMES - fadeInFramesRemaining) * 256) /
      FADE_IN_FRAMES;

    pcm[frame * 2] =
      static_cast<int16_t>((pcm[frame * 2] * gainQ8) >> 8);

    pcm[frame * 2 + 1] =
      static_cast<int16_t>((pcm[frame * 2 + 1] * gainQ8) >> 8);

    --fadeInFramesRemaining;
  }
}

void applyMildBassBoost(uint8_t* data,
                        int32_t byteCount) {
  // Process complete 16-bit stereo PCM frames only.
  const int32_t completeBytes =
    byteCount - (byteCount % 4);

  int16_t* pcm =
    reinterpret_cast<int16_t*>(data);

  const int32_t sampleCount =
    completeBytes / sizeof(int16_t);

  for (int32_t i = 0;
       i + 1 < sampleCount;
       i += 2) {
    const int32_t left = pcm[i];
    const int32_t right = pcm[i + 1];

    bassLowLeft +=
      (left - bassLowLeft) >>
      BASS_FILTER_SHIFT;

    bassLowRight +=
      (right - bassLowRight) >>
      BASS_FILTER_SHIFT;

    const int32_t enhancedLeft =
      ((left * MASTER_GAIN_Q8) +
       (bassLowLeft * BASS_GAIN_Q8)) >>
      8;

    const int32_t enhancedRight =
      ((right * MASTER_GAIN_Q8) +
       (bassLowRight * BASS_GAIN_Q8)) >>
      8;

    pcm[i] =
      limitPcmSample(enhancedLeft);

    pcm[i + 1] =
      limitPcmSample(enhancedRight);
  }
}

// ==================== BLUETOOTH ====================
bool selectBluetoothDevice(const char* name,
                           esp_bd_addr_t address,
                           int rssi) {
  if (!name || !*name) {
    return false;
  }

  Serial.print("BT device: ");
  Serial.println(name);

  // Exact-name matching is safer than matching every device containing R50i.
  return strcmp(name, EXACT_BT_NAME) == 0;
}

void onConnectionStateChanged(
  esp_a2d_connection_state_t state,
  void* object
) {
  const bool wasConnected = btConnected;
  const bool nowConnected =
    state == ESP_A2D_CONNECTION_STATE_CONNECTED;

  btConnected = nowConnected;

  if (nowConnected) {
    forceMaxVolumeRequested = true;

    if (!wasConnected) {
      pairedScreenRequested = true;
    }
  } else {
    pairedScreenRequested = false;
  }

  uiDirty = true;
}

// This callback runs inside a Bluetooth task.
// It must only set flags; never open files or draw the OLED here.
void onRemoteAvrcCommand(uint8_t key,
                         bool isReleased) {
  // A press normally creates PRESSED and RELEASED events.
  // Handle only RELEASED so every gesture runs once.
  if (!isReleased) {
    return;
  }

  switch (key) {
    case ESP_AVRC_PT_CMD_FORWARD:
      nextTrackRequested = true;
      break;

    case ESP_AVRC_PT_CMD_BACKWARD:
      previousTrackRequested = true;
      break;

    case ESP_AVRC_PT_CMD_PLAY:
    case ESP_AVRC_PT_CMD_PAUSE:
    case ESP_AVRC_PT_CMD_STOP:
      // One configured touch toggles the local player state.
      togglePlaybackRequested = true;
      break;

    default:
      // Do not allow remote volume gestures to change the source level.
      // Re-assert maximum volume from loop(), never from this BT callback.
      forceMaxVolumeRequested = true;
      break;
  }
}

int32_t provideAudio(uint8_t* data,
                     int32_t requestedBytes) {
  if (
    currentTrack < 0 ||
    isPaused ||
    switchingTrack ||
    nextTrackRequested ||
    previousTrackRequested
  ) {
    // currentTrack < 0: brief window right after boot if Bluetooth pairs
    // before the first track finishes opening - output silence instead.
    memset(data, 0, requestedBytes);
    return requestedBytes;
  }

  if (
    xSemaphoreTake(audioMutex, 0) != pdTRUE
  ) {
    memset(data, 0, requestedBytes);
    return requestedBytes;
  }

  const int32_t bytesRead =
    decodedAudio.readBytes(
      data,
      requestedBytes
    );

  const bool endOfFile =
    songFile &&
    (
      songFile.position() >= songFile.size() ||
      !songFile.available()
    );

  xSemaphoreGive(audioMutex);

  if (bytesRead > 0) {
    consecutiveEmptyReads = 0;
    audioStarted = true;

    applyMildBassBoost(data, bytesRead);
    applyFadeIn(data, bytesRead);

    if (bytesRead < requestedBytes) {
      memset(
        data + bytesRead,
        0,
        requestedBytes - bytesRead
      );
    }

    return requestedBytes;
  }

  if (endOfFile) {
    // A single empty read is not enough to declare EOF.
    if (++consecutiveEmptyReads >= 3) {
      nextTrackRequested = true;
    }
  } else if (underrunsThisTrack < 255) {
    // Data remains on disk but the decoder had nothing ready - a genuine
    // underrun. The Adaptive Queue uses this count when the next track opens.
    ++underrunsThisTrack;
  }

  memset(data, 0, requestedBytes);
  return requestedBytes;
}

// ==================== PHYSICAL BUTTON ====================
void handlePlayPauseButton() {
  const bool rawState =
    digitalRead(BTN_PLAY_PAUSE);

  if (rawState != lastRawButtonState) {
    lastRawButtonState = rawState;
    lastButtonChangeMs = millis();
  }

  if (
    millis() - lastButtonChangeMs >= DEBOUNCE_MS &&
    rawState != stableButtonState
  ) {
    stableButtonState = rawState;

    if (
      stableButtonState == LOW &&
      currentTrack >= 0 &&
      !switchingTrack
    ) {
      isPaused = !isPaused;
      uiDirty = true;

      if (!isPaused) {
        fadeInFramesRemaining = FADE_IN_FRAMES;
      }
    }
  }
}

// ==================== REMOTE COMMAND PROCESSING ====================
void processRemoteCommands() {
  if (forceMaxVolumeRequested) {
    forceMaxVolumeRequested = false;

    if (btConnected) {
      a2dpSource.set_volume(START_VOLUME);
    }
  }

  if (togglePlaybackRequested) {
    togglePlaybackRequested = false;

    if (currentTrack >= 0 && !switchingTrack) {
      isPaused = !isPaused;
      uiDirty = true;

      if (!isPaused) {
        fadeInFramesRemaining = FADE_IN_FRAMES;
      }
    }
  }

  if (
    previousTrackRequested &&
    !switchingTrack
  ) {
    previousTrackRequested = false;
    isPaused = false;

    if (!openPreviousTrack()) {
      Serial.println(
        "[AVRCP] No previous song available"
      );
    }
  }

  if (
    nextTrackRequested &&
    !switchingTrack
  ) {
    isPaused = false;

    if (!openNextAvailableTrack()) {
      // Every track in the playlist failed to open (e.g. an SD hiccup).
      // Show the problem but keep the device alive instead of hanging
      // forever - AVRCP/the button can still ask for Next again later.
      nextTrackRequested = false;
      isPaused = true;

      Serial.println(
        "[ERROR] No track could be opened - check SD card / playlist.txt"
      );

      drawSimpleScreen(
        "PLAYBACK ERROR",
        "No track opened",
        "Retry: earbud Next"
      );

      uiDirty = false;
    }
  }
}

// ==================== ERROR ====================
void fatal(
  const char* line1,
  const char* line2
) {
  drawSimpleScreen(line1, line2, "");

  while (true) {
    delay(1000);
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(250);

  printResetReason();

  pinMode(
    BTN_PLAY_PAUSE,
    INPUT_PULLUP
  );

  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(OLED_I2C_FREQUENCY_HZ);
  oled.begin();

  // The cat remains visible throughout initialization.
  showSplash();

  audioMutex =
    xSemaphoreCreateMutex();

  if (!audioMutex) {
    fatal("SYSTEM ERROR", "Audio mutex");
  }

  SPI.begin(
    SD_SCK,
    SD_MISO,
    SD_MOSI,
    SD_CS
  );

  if (!SD.begin(SD_CS, SPI, SD_SPI_FREQUENCY_HZ)) {
    // Some cards fail the first init right after power-up; retry once.
    delay(250);
    SD.end();

    if (!SD.begin(SD_CS, SPI, SD_SPI_FREQUENCY_HZ)) {
      // Fall back to the library's default clock if this card can't
      // keep up at SD_SPI_FREQUENCY_HZ.
      SD.end();

      if (!SD.begin(SD_CS, SPI)) {
        fatal("SD ERROR", "Check wiring");
      }

      Serial.println(
        "[SD] Fell back to default SPI clock (card rejected 20 MHz)"
      );
    }
  }

  // Bluetooth starts before the playlist scan so pairing can begin
  // immediately; provideAudio() guards on currentTrack < 0 until the
  // first track finishes opening.
  a2dpSource.set_auto_reconnect(true);
  a2dpSource.set_ssp_enabled(true);

  a2dpSource.set_ssid_callback(
    selectBluetoothDevice
  );

  a2dpSource.set_data_callback(
    provideAudio
  );

  a2dpSource.set_on_connection_state_changed(
    onConnectionStateChanged
  );

  a2dpSource.set_avrc_passthru_command_callback(
    onRemoteAvrcCommand
  );

  a2dpSource.set_volume(START_VOLUME);

  a2dpSource.start();

  printMemory("after Bluetooth start");

  trackCount = countPlaylistTracks();

  if (trackCount == 0) {
    fatal(
      "PLAYLIST ERROR",
      "Check playlist.txt"
    );
  }

  shuffleOrder = new (std::nothrow) uint16_t[trackCount];

  if (!shuffleOrder) {
    fatal("SYSTEM ERROR", "Shuffle memory");
  }

  // Resume Shuffle State: pull in whatever was saved before the last
  // power-off (if the playlist size still matches). When that succeeds,
  // the saved seed/anchor reproduce the exact same shuffle order via
  // resumeShuffleCycle(), and shufflePosition is fast-forwarded to
  // where playback had gotten to - no fresh shuffle, no repeats.
  // Otherwise (first boot, or the playlist changed size) a normal new
  // cycle is started.
  if (loadShuffleState()) {
    resumeShuffleCycle();

    shufflePosition =
      (pendingShufflePosition <= trackCount) ?
        pendingShufflePosition : 0;
  } else {
    beginNewShuffleCycle();
  }

  // openTrack() re-applies this on every switch (Adaptive Queue), but the
  // very first begin() happens before any track has opened, so size it here too.
  decodedAudio
    .transformationReader()
    .resizeResultQueue(activeQueueBytes);

  if (!openNextAvailableTrack()) {
    fatal("FILE ERROR", "No valid MP3");
  }

  // The large cat splash stays visible the whole time regardless of
  // uiDirty - loop()'s "if (!btConnected)" branch forces it back up
  // until a real connection lands - so there's nothing to reset here now
  // that Bluetooth already started before this point.
  printMemory("after first track opened");
}

// ==================== LOOP ====================
void loop() {
  if (millis() - lastMemoryLogMs >= MEMORY_LOG_INTERVAL_MS) {
    lastMemoryLogMs = millis();
    printMemory("periodic");
  }

  handlePlayPauseButton();
  processRemoteCommands();

  // A new real Bluetooth connection gets a short Paired celebration.
  if (
    pairedScreenRequested &&
    btConnected &&
    !switchingTrack
  ) {
    pairedScreenRequested = false;
    pairedScreenActive = true;
    pairedScreenEndsAt =
      millis() + PAIRED_SCREEN_MS;
    uiDirty = false;

    drawPairedScreen();
  }

  // Before pairing, and whenever the earbuds disconnect,
  // the large cat remains on the display.
  if (!btConnected) {
    pairedScreenActive = false;

    if (uiDirty) {
      uiDirty = false;
      showSplash();
    }

    vTaskDelay(pdMS_TO_TICKS(3));
    return;
  }

  // Do not block Bluetooth audio with delay(1400).
  // Keep streaming while the Paired screen is timed non-blockingly.
  if (pairedScreenActive) {
    if (
      static_cast<int32_t>(
        millis() - pairedScreenEndsAt
      ) >= 0
    ) {
      pairedScreenActive = false;
      uiDirty = false;
      drawPlayerScreen();
    }

    vTaskDelay(pdMS_TO_TICKS(3));
    return;
  }

  if (uiDirty && !switchingTrack) {
    uiDirty = false;

    if (currentTrack >= 0) {
      drawPlayerScreen();
    }
  }

  // Preload Engine: top up the next track whenever the audio path is idle. Cheap to call every tick - it no-ops unless something is actually missing.
  if (!switchingTrack && currentTrack >= 0) {
    preloadNextTrack();
  }

  // A short idle delay keeps AVRCP touch commands (play/pause/next/previous from the earbuds) feeling instant without pegging the CPU.
  vTaskDelay(pdMS_TO_TICKS(3));
}

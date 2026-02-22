#pragma once
#include <Arduino.h>
#include <SD.h>

class SDCard {
public:
  SDCard() {}

  // Initialize the SD card (Teensy 4.1: built-in SDIO)
  bool begin(uint32_t csPin = BUILTIN_SDCARD);

  // Create/open new data & event files with incrementing indices.
  // e.g., DATA0001.CSV and EVENT0001.TXT
  bool startLogs(const char* dataPrefix = "DATA",
                 const char* eventPrefix = "EVENT");

  // Buffered CSV logging (call at 100 Hz)
  void logData(float flightTime,
               float ax, float ay, float az,
               float gx, float gy, float gz,
               float roll, float pitch,
               float temperature, float pressure_hPa, float altitude_m,
               int servoYdeg, int servoXdeg, float thrust);

  // Write an event line. If time_s < 0, it will use millis()/1000.
  void logEvent(const char* msg, float time_s = -1.0f);

  // Force flushing of buffered CSV lines to the card (non-blocking most of the time)
  void flushBuffered();

  // Flush the SD files (forces card flush)
  void flushNow();

  // Close files (calls flushBuffered first)
  void close();

  bool isOpen()   const { return dataOpen; }
  const char* dataFilename()  const { return dataName; }
  const char* eventFilename() const { return eventName; }

  // Optional: tune time-based buffered flush interval (ms)
  void setFlushPeriod(uint16_t ms) { flushPeriodMs = ms; }

private:
  // --------- configuration for CSV buffering ----------
  static const size_t  CSV_LINE_MAX   = 160;  // max bytes per CSV line (tune to your header)
  static const size_t  CSV_BUF_LINES  = 40;   // lines to batch before write
  uint16_t             flushPeriodMs  = 250;  // also flush every X ms

  // --------- buffering state ----------
  char     csvBuf[CSV_BUF_LINES][CSV_LINE_MAX];
  uint16_t csvCount     = 0;
  uint32_t lastFlushMs  = 0;

  // --------- SD state ----------
  File dataFile;
  File eventFile;
  bool dataOpen  = false;
  bool eventOpen = false;

  char dataName[32]  = {0};
  char eventName[32] = {0};

  // helpers
  void writeDataHeader_();
  static void makeIndexedName_(char* out, size_t outLen,
                               const char* prefix, uint16_t index,
                               const char* ext);
  static uint16_t findNextIndex_(const char* dataPrefix,
                                 const char* eventPrefix);
};

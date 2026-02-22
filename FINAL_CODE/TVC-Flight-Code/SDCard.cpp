#include "SDCard.h"
#include <cstring>

bool SDCard::begin(uint32_t csPin) {
  // Try begin; return false if card not present or formatted badly
  if (!SD.begin(csPin)) {
    return false;
  }
  return true;
}

void SDCard::makeIndexedName_(char* out, size_t outLen,
                              const char* prefix, uint16_t index,
                              const char* ext) {
  // prefix + 4-digit zero-padded index + '.' + ext
  // e.g., DATA0123.CSV
  snprintf(out, outLen, "%s%04u.%s", prefix, (unsigned)index, ext);
}

// Scan for next free index that does not exist for either data or event files
uint16_t SDCard::findNextIndex_(const char* dataPrefix,
                                const char* eventPrefix) {
  for (uint16_t i = 1; i < 10000; ++i) {
    char d[32]; char e[32];
    makeIndexedName_(d, sizeof(d), dataPrefix, i, "CSV");
    makeIndexedName_(e, sizeof(e), eventPrefix, i, "TXT");
    if (!SD.exists(d) && !SD.exists(e)) {
      return i;
    }
  }
  // Fallback: last slot
  return 9999;
}

bool SDCard::startLogs(const char* dataPrefix, const char* eventPrefix) {
  close();  // close any previous

  // Find next free index
  uint16_t idx = findNextIndex_(dataPrefix, eventPrefix);

  // Build filenames
  makeIndexedName_(dataName,  sizeof(dataName),  dataPrefix,  idx, "CSV");
  makeIndexedName_(eventName, sizeof(eventName), eventPrefix, idx, "TXT");

  // Open files
  dataFile  = SD.open(dataName,  FILE_WRITE);
  eventFile = SD.open(eventName, FILE_WRITE);

  dataOpen  = (bool)dataFile;
  eventOpen = (bool)eventFile;

  if (!dataOpen || !eventOpen) {
    // Clean up partially opened
    if (dataOpen)  { dataFile.close();  dataOpen  = false; }
    if (eventOpen) { eventFile.close(); eventOpen = false; }
    dataName[0] = 0; eventName[0] = 0;
    return false;
  }

  // Write headers once, but do not flush synchronously
  writeDataHeader_();
  eventFile.println(F("Event log:"));
  // Keep timestamps consistent with flight time later
  // No flush here: avoid blocking at arming time

  // reset buffering state
  csvCount    = 0;
  lastFlushMs = millis();

  return true;
}

void SDCard::writeDataHeader_() {
  if (!dataOpen) return;
  dataFile.println(
    F("time_s,ax,ay,az,gx_deg_s,gy_deg_s,gz_deg_s,roll_rad,pitch_rad,"
      "temp_C,press_hPa,alt_m,servoY_deg,servoX_deg,thrust_N"));
}

void SDCard::logData(float flightTime,
                     float ax, float ay, float az,
                     float gx, float gy, float gz,
                     float roll, float pitch,
                     float temperature, float pressure_hPa, float altitude_m,
                     int servoYdeg, int servoXdeg, float thrust)
{
  if (!dataOpen) return;

  // Format one CSV line into the current buffer slot
  int n = snprintf(csvBuf[csvCount], CSV_LINE_MAX,
                   "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.5f,%0.5f,"
                   "%0.2f,%0.2f,%0.2f,%d,%d,%0.2f\n",
                   flightTime, ax, ay, az, gx, gy, gz, roll, pitch,
                   temperature, pressure_hPa, altitude_m,
                   servoYdeg, servoXdeg, thrust);

  if (n <= 0 || n >= (int)CSV_LINE_MAX) {
    // Truncated/format error: drop to keep control real-time
    return;
  }

  csvCount++;

  // Flush when buffer full or after the period
  uint32_t now = millis();
  if (csvCount >= CSV_BUF_LINES || (now - lastFlushMs) >= flushPeriodMs) {
    flushBuffered();
  }
}

void SDCard::flushBuffered() {
  if (!dataOpen || csvCount == 0) return;

  for (uint16_t i = 0; i < csvCount; ++i) {
    dataFile.write(csvBuf[i], strlen(csvBuf[i]));
  }
  csvCount = 0;
  lastFlushMs = millis();
  // We deliberately do not call dataFile.flush() here to avoid blocking
}

void SDCard::flushNow() {
  // Push any buffered CSV lines first
  flushBuffered();

  // Then force the card’s internal buffers to be written
  if (dataOpen)  dataFile.flush();
  if (eventOpen) eventFile.flush();
}

void SDCard::logEvent(const char* msg, float time_s) {
  if (!eventOpen) return;

  if (time_s < 0.0f) {
    time_s = millis() / 1000.0f;
  }
  eventFile.print('[');
  eventFile.print(time_s, 3);
  eventFile.print(F(" s] "));
  eventFile.println(msg);
  // No immediate flush; will be flushed on state transitions or close()
}

void SDCard::close() {
  // Ensure remaining buffered samples are written
  flushBuffered();

  if (dataOpen) {
    dataFile.flush();
    dataFile.close();
    dataOpen = false;
  }
  if (eventOpen) {
    eventFile.flush();
    eventFile.close();
    eventOpen = false;
  }
  dataName[0]  = 0;
  eventName[0] = 0;
}

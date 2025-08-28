#include "ublox_reader.h"
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

// Correctly extracts payload data by using the proper byte offsets.
static int NAV_POSLLH(uint8_t *buffer, classId *gps) {
  // Corrected memcpy offsets for each field within the payload
  memcpy(&gps->iTOW,   buffer + 0, 4);
  memcpy(&gps->lon,    buffer + 4, 4); // Longitude starts at byte 4
  memcpy(&gps->lat,    buffer + 8, 4); // Latitude starts at byte 8
  memcpy(&gps->height, buffer + 12, 4);
  memcpy(&gps->hMSL,   buffer + 16, 4);
  memcpy(&gps->hAcc,   buffer + 20, 4);
  memcpy(&gps->vAcc,   buffer + 24, 4);
  return 0;
}

// Helper function to convert a string of hex values into a vector of bytes.
static vector<uint8_t> hexToBytes(const string &rawHex) {
  vector<uint8_t> bytes;
  stringstream ss(rawHex);
  string token;
  while (ss >> token) {
    bytes.push_back(static_cast<uint8_t>(stoul(token, nullptr, 16)));
  }
  return bytes;
}

// Decodes the UBX frame.
int decodeUBX(uint8_t *buffer, classId *gps) {
  // --- BUG FIX ---
  // The provided hex data in the test files starts directly with the
  // Class and ID, omitting the usual 2-byte sync header.
  // Therefore, we must check for Class (0x01) and ID (0x02) at the very
  // beginning of the buffer (index 0 and 1).
  if (buffer[0] == 0x01 && buffer[1] == 0x02) {
    // The payload starts after the Class (1 byte), ID (1 byte), and Length (2 bytes) fields.
    // So, we must skip the first 4 bytes of the buffer.
    return NAV_POSLLH(buffer + 4, gps);
  }
  return 1; // Return 1 to indicate failure
}

// Converts raw integer data to the final GPS struct with correct units.
GPS gpsFromData(const classId &gps) {
  GPS out;
  // Scale integer values to get degrees and meters
  out.lat = static_cast<double>(gps.lat) * 1e-7;
  out.lon = static_cast<double>(gps.lon) * 1e-7;
  out.height = static_cast<double>(gps.height) / 1000.0;
  return out;
}

// Reads the two-line UBX hex file.
pair<GPS, GPS> readUbloxFile(const string &filename) {
  ifstream file(filename);
  if (!file.is_open()) {
    cerr << "Error: cannot open file " << filename << endl;
    return {{0.0, 0.0}, {0.0, 0.0}};
  }
  string rawStart, rawGoal;
  getline(file, rawStart);
  getline(file, rawGoal);

  cout << "Raw UBX Start: " << rawStart << endl;
  cout << "Raw UBX Goal : " << rawGoal << endl;

  vector<uint8_t> startBytes = hexToBytes(rawStart);
  vector<uint8_t> goalBytes = hexToBytes(rawGoal);

  classId gpsStartData, gpsGoalData;
  decodeUBX(startBytes.data(), &gpsStartData);
  decodeUBX(goalBytes.data(), &gpsGoalData);

  GPS startGPS = gpsFromData(gpsStartData);
  GPS goalGPS = gpsFromData(gpsGoalData);

  file.close();

  return {startGPS, goalGPS};
}

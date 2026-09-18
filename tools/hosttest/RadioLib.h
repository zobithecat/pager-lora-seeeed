#pragma once
#include "Arduino.h"
#include <vector>
#define RADIOLIB_ERR_NONE 0
#define RADIOLIB_NC (0xFFFFFFFFu)
#define RADIOLIB_CHANNEL_FREE (-711)
#define RADIOLIB_LORA_DETECTED (-702)
struct Module { Module(int, int, int, int) {} };
struct TxRec { std::string line; uint32_t at; };
extern std::vector<TxRec> g_tx;
extern std::vector<uint8_t> g_rx_pkt;
extern int g_cad_busy_n;      // next N scans report busy
extern int g_rx_rssi;
struct SX1262 {
  bool resetOnStartup = true;
  SX1262(Module*) {}
  void setRfSwitchPins(uint32_t, uint32_t) {}
  int begin(float, float, int, int, int, int, int, float) { return 0; }
  int setDio2AsRfSwitch(bool) { return 0; }
  int setCRC(int) { return 0; }
  void setPacketReceivedAction(void (*)()) {}
  int startReceive() { return 0; }
  int scanChannel() { if (g_cad_busy_n > 0) { g_cad_busy_n--; return RADIOLIB_LORA_DETECTED; } return RADIOLIB_CHANNEL_FREE; }
  int transmit(String& w) { g_tx.push_back({w.s, g_now}); g_now += 250; return 0; }
  size_t getPacketLength() { return g_rx_pkt.size(); }
  int readData(uint8_t* b, size_t n) { memcpy(b, g_rx_pkt.data(), n); return 0; }
  float getRSSI() { return (float)g_rx_rssi; }
  float getSNR() { return 9.5f; }
};

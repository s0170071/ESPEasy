#ifndef PLUGINSTRUCTS_P182_DATA_STRUCT_H
#define PLUGINSTRUCTS_P182_DATA_STRUCT_H

#include "../../_Plugin_Helper.h"

#ifdef USES_P182

# include <ESPeasySerial.h>

// An ISKRA MT681 / eHZ 681 SML datagram is typically ~400 bytes.
// Meters that also transmit the manufacturer/public-key list can be larger.
# define P182_SML_MSG_BUFFER_SIZE 512

// Drain the hardware FIFO when this much is pending; we are clearly not keeping up
// and the frame in flight is lost anyway.
# define P182_SERIAL_RX_BUFFER    1024
# define P182_SERIAL_OVERFLOW     (P182_SERIAL_RX_BUFFER - 32)

// Upper bound on bytes handled per call, so a burst cannot starve the rest of ESPEasy.
# define P182_MAX_BYTES_PER_LOOP  256

enum class P182_ReceiveState : uint8_t {
  WaitForBegin,
  WaitForEnd,
  GetChecksum,
  CheckCRC
};

struct P182_data_struct : public PluginTaskData_base {
  P182_data_struct(ESPEasySerialPort port,
                   int8_t            rxPin,
                   int8_t            txPin,
                   unsigned long     baudrate);
  virtual ~P182_data_struct();

  P182_data_struct(const P182_data_struct&)            = delete;
  P182_data_struct& operator=(const P182_data_struct&) = delete;

  bool init();

  bool isInitialized() const {
    return nullptr != _easySerial;
  }

  // Feed the state machine with whatever arrived since the last call.
  // Returns true when a complete, CRC-valid datagram was decoded.
  bool loop();

  bool hasValue() const {
    return _haveData;
  }

  double getCurrentPower() const {
    return _currentPower;
  }

  double getSellCounterTotal() const {
    return _sellCounterTotal;
  }

  double getBuyCounterTotal() const {
    return _buyCounterTotal;
  }

private:

  void          emptyBuffers();

  // Decode the SML value that follows the OBIS code starting at pos.
  // Returns false when the buffer does not hold a complete value.
  bool          getSMLValue(size_t  pos,
                            double& result) const;

  // Locate the next OBIS entry at or after pos and store its value.
  // Returns the position to continue scanning from.
  size_t        findValue(size_t pos);

  static uint16_t crcReflectw(uint16_t data);
  static uint8_t  crcReflectb(uint8_t data);
  static uint16_t crcX25(const uint8_t *data,
                         uint16_t       data_len);
  bool            checkCRC(size_t frameEnd) const;

  ESPeasySerial *_easySerial = nullptr;

  const ESPEasySerialPort _port;
  const int8_t            _rxPin;
  const int8_t            _txPin;
  const unsigned long     _baudrate;

  static const uint8_t FrameStart[8];
  static const uint8_t FrameEnd[5];

  uint8_t           _buffer[P182_SML_MSG_BUFFER_SIZE]{};
  size_t            _bufferPtr = 0;
  P182_ReceiveState _RState    = P182_ReceiveState::WaitForBegin;

  double _currentPower     = NAN; // Momentary active power, negative = feeding into the grid
  double _sellCounterTotal = NAN; // Einspeisezaehler, OBIS 2.8.0
  double _buyCounterTotal  = NAN; // Bezugszaehler,    OBIS 1.8.0

  bool _haveData = false;
};

#endif // ifdef USES_P182
#endif // ifndef PLUGINSTRUCTS_P182_DATA_STRUCT_H

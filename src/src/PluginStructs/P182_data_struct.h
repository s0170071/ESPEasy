#ifndef PLUGINSTRUCTS_P182_DATA_STRUCT_H
#define PLUGINSTRUCTS_P182_DATA_STRUCT_H

#include "../../_Plugin_Helper.h"

#ifdef USES_P182

# include <ESPeasySerial.h>

// An ISKRA MT681 / eHZ 681 SML datagram is typically 300..700 bytes, depending on how many
// OBIS entries (and the optional public key) the meter transmits. The SML transport layer
// itself does not limit the datagram size, so leave generous headroom.
# define P182_SML_MSG_BUFFER_SIZE 2048

// Drain the hardware FIFO when this much is pending; we are clearly not keeping up
// and the frame in flight is lost anyway.
# define P182_SERIAL_RX_BUFFER    1024
# define P182_SERIAL_OVERFLOW     (P182_SERIAL_RX_BUFFER - 32)

// Upper bound on bytes handled per call, so a burst cannot starve the rest of ESPEasy.
# define P182_MAX_BYTES_PER_LOOP  256

enum class P182_ReceiveState : uint8_t {
  WaitForBegin, // Looking for the SML transport v1 start sequence 1b1b1b1b 01010101
  WaitForEnd    // Collecting 4-byte aligned words until the end sequence 1b1b1b1b 1a <pad> <crc16>
};

struct P182_data_struct : public PluginTaskData_base {
  P182_data_struct(ESPEasySerialPort port,
                   int8_t            rxPin,
                   int8_t            txPin,
                   unsigned long     baudrate,
                   bool              inverseLogic,
                   bool              rxPullup);
  virtual ~P182_data_struct();

  P182_data_struct(const P182_data_struct&)            = delete;
  P182_data_struct& operator=(const P182_data_struct&) = delete;

  bool init();

  bool isInitialized() const {
    return (nullptr != _easySerial) && static_cast<bool>(*_easySerial);
  }

  uint32_t receivedBytes() const { return _receivedBytes; }
  uint32_t startedFrames() const { return _startedFrames; }
  uint32_t lastFrameLength() const { return _lastFrameLength; }
  uint32_t validFrames() const { return _validFrames; }
  uint32_t crcErrors() const { return _crcErrors; }
  uint32_t droppedFrames() const { return _droppedFrames; }
  String receiveSample() const;

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

  // Decode the SML type-length field at pos. For a list, len is the number of elements,
  // otherwise the number of payload bytes. pos is advanced behind the TL field.
  bool          readTL(size_t& pos,
                       size_t  end,
                       uint8_t& type,
                       size_t& len) const;

  // Skip a complete element (recursing into lists).
  bool          skipElement(size_t& pos,
                            size_t  end,
                            uint8_t depth) const;

  // Read an Integer/Unsigned element of 1..8 bytes.
  bool          readNumber(size_t& pos,
                           size_t  end,
                           double& value,
                           bool    optional) const;

  // Try to decode the SML list entry (SML_ListEntry) at pos.
  // Returns false when the element at pos is not such an entry.
  bool          readListEntry(size_t pos,
                              size_t end);

  // Walk the SML message tree and decode all list entries.
  bool          walk(size_t& pos,
                     size_t  end,
                     uint8_t depth);

  void          storeValue(const uint8_t *obis,
                           uint8_t        unit,
                           double         value);

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
  const bool              _inverseLogic;
  const bool              _rxPullup;

  static const uint8_t FrameStart[8];

  uint8_t           _buffer[P182_SML_MSG_BUFFER_SIZE]{};
  size_t            _bufferPtr = 0;
  bool              _inEscape  = false; // Previous 4-byte word was an escape sequence
  bool              _gotValue  = false; // Set by storeValue()
  P182_ReceiveState _RState    = P182_ReceiveState::WaitForBegin;

  double _currentPower     = NAN; // Momentary active power, negative = feeding into the grid
  double _sellCounterTotal = NAN; // Einspeisezaehler, OBIS 2.8.0
  double _buyCounterTotal  = NAN; // Bezugszaehler,    OBIS 1.8.0

  bool _haveData = false;
  uint32_t _receivedBytes = 0;
  uint32_t _startedFrames = 0;
  uint32_t _lastFrameLength = 0;
  uint32_t _validFrames   = 0;
  uint32_t _crcErrors     = 0;
  uint32_t _droppedFrames = 0;
  uint8_t _receiveSample[64]{};
  size_t _receiveSampleSize = 0;
};

#endif // ifdef USES_P182
#endif // ifndef PLUGINSTRUCTS_P182_DATA_STRUCT_H

#include "../PluginStructs/P182_data_struct.h"

#ifdef USES_P182

# include "../Helpers/ESPEasy_time_calc.h"
# include "../Helpers/StringConverter_Numerical.h"

# ifdef ESP32
#  include <driver/gpio.h>
# endif // ifdef ESP32

const uint8_t P182_data_struct::FrameStart[8] = { 0x1b, 0x1b, 0x1b, 0x1b, 0x01, 0x01, 0x01, 0x01 };

P182_data_struct::P182_data_struct(ESPEasySerialPort port,
                                   int8_t            rxPin,
                                   int8_t            txPin,
                                   unsigned long     baudrate,
                                   bool              inverseLogic,
                                   bool              rxPullup)
  : _port(port), _rxPin(rxPin), _txPin(txPin), _baudrate(baudrate), _inverseLogic(inverseLogic), _rxPullup(rxPullup)
{}

P182_data_struct::~P182_data_struct() {
  delete _easySerial;
  _easySerial = nullptr;
}

String P182_data_struct::receiveSample() const {
  return formatToHex_array(_receiveSample, _receiveSampleSize);
}

bool P182_data_struct::init() {
  delete _easySerial;
  _easySerial = new (std::nothrow) ESPeasySerial(_port, _rxPin, _txPin, _inverseLogic, P182_SERIAL_RX_BUFFER);

  if (nullptr == _easySerial) {
    addLog(LOG_LEVEL_ERROR, F("MT681: Failed to allocate serial port"));
    return false;
  }
  _easySerial->begin(_baudrate, SERIAL_8N1);
  if (!*_easySerial) {
    addLog(LOG_LEVEL_ERROR, F("MT681: Failed to start serial port"));
    delete _easySerial;
    _easySerial = nullptr;
    return false;
  }
# ifdef ESP32

  // Open-collector optical heads need a pull-up on RX.
  if (_rxPullup && (_rxPin >= 0)) {
    gpio_set_pull_mode(static_cast<gpio_num_t>(_rxPin), GPIO_PULLUP_ONLY);
  }
# endif // ifdef ESP32
  emptyBuffers();

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    addLog(LOG_LEVEL_INFO, concat(F("MT681: "), _easySerial->getLogString()));
  }
  return true;
}

void P182_data_struct::emptyBuffers() {
  if (isInitialized()) {
    uint32_t timeout = millis();

    while (_easySerial->available()) {
      _easySerial->read();

      if (timeOutReached(timeout + 200)) { // Feed the watchdog on a very long drain
        delay(1);
        timeout = millis();
      }
    }
  }
  _bufferPtr = 0;
  _inEscape  = false;
  _RState    = P182_ReceiveState::WaitForBegin;
}

/**
 * SML (Smart Message Language) encoding, see BSI TR-03109-1 / SML specification 1.03:
 *
 * Every element starts with a TL (type/length) byte: bit 7 = "more TL bytes follow",
 * bits 6..4 = type (0 octet string, 4 boolean, 5 integer, 6 unsigned, 7 list),
 * bits 3..0 = length (most significant nibble first when the TL spans several bytes).
 * For non-list types the length includes the TL byte(s), for lists it is the element count.
 * 0x01 is an optional field that is not set, 0x00 marks the end of an SML message.
 */
bool P182_data_struct::readTL(size_t& pos, size_t end, uint8_t& type, size_t& len) const {
  if (pos >= end) { return false; }

  uint8_t b = _buffer[pos++];

  if (b == 0x00) { // End of SML message
    type = 0;
    len  = 0;
    return true;
  }
  type = (b >> 4) & 0x07;
  size_t total   = b & 0x0f;
  size_t tlBytes = 1;

  while (b & 0x80) {
    if ((pos >= end) || (tlBytes >= 4)) { return false; }
    b = _buffer[pos++];
    ++tlBytes;
    total = (total << 4) | (b & 0x0f);
  }

  if (type == 7) { // List: total is the number of elements
    len = total;
    return true;
  }

  if ((total < tlBytes) || ((pos + (total - tlBytes)) > end)) { return false; }
  len = total - tlBytes;
  return true;
}

bool P182_data_struct::skipElement(size_t& pos, size_t end, uint8_t depth) const {
  uint8_t type;
  size_t  len;

  if (!readTL(pos, end, type, len)) { return false; }

  if (type != 7) {
    pos += len;
    return true;
  }

  if (depth == 0) { return false; }

  for (size_t i = 0; i < len; ++i) {
    if (!skipElement(pos, end, depth - 1)) { return false; }
  }
  return true;
}

bool P182_data_struct::readNumber(size_t& pos, size_t end, double& value, bool optional) const {
  uint8_t type;
  size_t  len;

  if (!readTL(pos, end, type, len)) { return false; }

  if (optional && (type == 0) && (len == 0)) { // 0x01: field not set
    value = 0;
    return true;
  }

  if (((type != 5) && (type != 6)) || (len < 1) || (len > 8)) {
    pos += (type == 7) ? 0 : len;
    return false;
  }
  uint64_t raw = 0;

  for (size_t i = 0; i < len; ++i) {
    raw = (raw << 8) | _buffer[pos + i];
  }
  pos += len;

  if ((type == 5) && (len < 8) && (raw & (static_cast<uint64_t>(1) << (8 * len - 1)))) {
    raw |= ~static_cast<uint64_t>(0) << (8 * len); // Sign extend
  }
  value = (type == 5) ? static_cast<double>(static_cast<int64_t>(raw)) : static_cast<double>(raw);
  return true;
}

/**
 * SML_ListEntry = list of 7:
 *   objName (octet string, 6 byte OBIS), status, valTime, unit (Unsigned8),
 *   scaler (Integer8), value, valueSignature
 * The physical value is value * 10^scaler in the given unit (27 = W, 30 = Wh).
 */
bool P182_data_struct::readListEntry(size_t pos, size_t end) {
  uint8_t type;
  size_t  len;

  if (!readTL(pos, end, type, len) || (type != 7) || (len != 7)) { return false; }

  if (!readTL(pos, end, type, len) || (type != 0) || (len != 6)) { return false; }
  const uint8_t *obis = &_buffer[pos];
  pos += 6;

  if (!skipElement(pos, end, 2) || // status
      !skipElement(pos, end, 2)) { // valTime (a list when present)
    return false;
  }

  double unit   = 0;
  double scaler = 0;
  double value  = 0;

  if (!readNumber(pos, end, unit, true) || !readNumber(pos, end, scaler, true) || !readNumber(pos, end, value, false)) {
    return false;
  }

  double factor = 1.0;

  const int exponent = static_cast<int8_t>(static_cast<int>(scaler));

  for (int i = 0; i < abs(exponent); ++i) { factor *= 10.0; }
  value = (exponent < 0) ? value / factor : value * factor;
  storeValue(obis, static_cast<uint8_t>(unit), value);
  return true;
}

void P182_data_struct::storeValue(const uint8_t *obis, uint8_t unit, double value) {
  // OBIS A-B:C.D.E*F, only electricity (A=1, B=0) is of interest.
  if ((obis[0] != 0x01) || (obis[1] != 0x00) || (obis[4] != 0x00)) { return; }

  // Counters are reported in Wh, expose them as kWh.
  const double kWh = (unit == 0x1e || unit == 0) ? value / 1000.0 : value;

  if ((obis[2] == 0x01) && (obis[3] == 0x08)) {         // 1.8.0  Bezug (import) total
    _buyCounterTotal = kWh;
    _gotValue        = true;
  } else if ((obis[2] == 0x02) && (obis[3] == 0x08)) {  // 2.8.0  Einspeisung (export) total
    _sellCounterTotal = kWh;
    _gotValue         = true;
  } else if ((obis[2] == 0x10) && (obis[3] == 0x07)) {  // 16.7.0 momentary active power in W
    _currentPower = value;
    _gotValue     = true;
  }
}

bool P182_data_struct::walk(size_t& pos, size_t end, uint8_t depth) {
  if (readListEntry(pos, end)) {
    return skipElement(pos, end, depth); // Continue behind the entry
  }

  uint8_t type;
  size_t  len;

  if (!readTL(pos, end, type, len)) { return false; }

  if (type != 7) {
    pos += len;
    return true;
  }

  if (depth == 0) { return false; }

  for (size_t i = 0; i < len; ++i) {
    if (!walk(pos, end, depth - 1)) { return false; }
  }
  return true;
}

uint16_t P182_data_struct::crcReflectw(uint16_t data) {
  uint16_t ret = data & 0x01;

  for (uint8_t i = 1; i < 16; ++i) {
    data >>= 1;
    ret    = (ret << 1) | (data & 0x01);
  }
  return ret;
}

uint8_t P182_data_struct::crcReflectb(uint8_t data) {
  uint16_t ret = data & 0x01;

  for (uint8_t i = 1; i < 8; ++i) {
    data >>= 1;
    ret    = (ret << 1) | (data & 0x01);
  }
  return static_cast<uint8_t>(ret);
}

// X.25 / CRC-16-CCITT as used in the SML transport layer, seeded the way the
// MT681 expects it.
uint16_t P182_data_struct::crcX25(const uint8_t *data, uint16_t data_len) {
  const uint8_t *d  = data;
  uint16_t crc      = 0x84cf;

  while (data_len--) {
    delay(0); // Feed the watchdog on long frames
    const uint8_t c = crcReflectb(*d++);

    for (uint8_t i = 0; i < 8; ++i) {
      const uint16_t bit = crc & 0x8000;
      crc = static_cast<uint16_t>((crc << 1) | ((c >> (7 - i)) & 0x01));

      if (bit) { crc ^= 0x1021; }
    }
  }

  for (uint8_t i = 0; i < 16; ++i) {
    const uint16_t bit = crc & 0x8000;
    crc = static_cast<uint16_t>(crc << 1);

    if (bit) { crc ^= 0x1021; }
  }
  crc = crcReflectw(crc);
  return static_cast<uint16_t>(crc ^ 0xffff);
}

bool P182_data_struct::checkCRC(size_t frameEnd) const {
  if ((frameEnd + 2) > P182_SML_MSG_BUFFER_SIZE) { return false; }

  const uint16_t computed = crcX25(&_buffer[0], static_cast<uint16_t>(frameEnd));
  const uint16_t received = static_cast<uint16_t>((_buffer[frameEnd + 1] << 8) | _buffer[frameEnd]);

# ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    addLog(LOG_LEVEL_DEBUG, strformat(F("MT681: CRC %s (calc 0x%04x, recv 0x%04x)"),
                                      computed == received ? "OK" : "FAIL", computed, received));
  }
# endif // ifndef BUILD_NO_DEBUG

  return computed == received;
}

bool P182_data_struct::loop() {
  if (!isInitialized()) { return false; }

  if (_easySerial->available() > P182_SERIAL_OVERFLOW) {
    // We fell too far behind; the frame in flight is unusable.
    addLog(LOG_LEVEL_ERROR, F("MT681: RX overflow, discarding"));
    ++_droppedFrames;
    emptyBuffers();
    return false;
  }

  bool     decoded   = false;
  uint16_t processed = 0;

  while (_easySerial->available() && (processed < P182_MAX_BYTES_PER_LOOP)) {
    ++processed;
    ++_receivedBytes;
    _buffer[_bufferPtr++] = static_cast<uint8_t>(_easySerial->read());
    if (_receiveSampleSize < sizeof(_receiveSample)) {
      _receiveSample[_receiveSampleSize++] = _buffer[_bufferPtr - 1];
    }

    switch (_RState) {
      case P182_ReceiveState::WaitForBegin:

        if (_buffer[_bufferPtr - 1] == FrameStart[_bufferPtr - 1]) {
          if (_bufferPtr == sizeof(FrameStart)) {
            _RState   = P182_ReceiveState::WaitForEnd; // Start sequence complete
            _inEscape = false;
            ++_startedFrames;
          }
        } else {
          // Not the start sequence. The byte may be the first byte of the next one.
          const uint8_t b = _buffer[_bufferPtr - 1];
          _bufferPtr = 0;

          if (b == FrameStart[0]) { _buffer[_bufferPtr++] = b; }
        }
        break;

      case P182_ReceiveState::WaitForEnd:

        // The transport layer works on 4-byte aligned words (the start sequence is aligned too):
        //   1b1b1b1b 1b1b1b1b  = escaped literal 1b1b1b1b in the payload
        //   1b1b1b1b 1a PP CC CC = end of datagram, PP = number of padding bytes, CCCC = CRC16
        if ((_bufferPtr % 4) != 0) { break; }
        {
          const uint8_t *w     = &_buffer[_bufferPtr - 4];
          const bool     isEsc = (w[0] == 0x1b) && (w[1] == 0x1b) && (w[2] == 0x1b) && (w[3] == 0x1b);

          if (_inEscape) {
            _inEscape = false;

            if (w[0] == 0x1a) {
              _lastFrameLength = _bufferPtr;

              if (checkCRC(_bufferPtr - 2)) {
                ++_validFrames;
                _gotValue = false;
                size_t       pos = 8;                       // Behind the start sequence
                const size_t end = _bufferPtr - 8;          // Before the end sequence

                while ((pos < end) && walk(pos, end, 8)) {}
                decoded   = _gotValue;
                _haveData = _haveData || _gotValue;
              } else {
                ++_crcErrors;
              }
              emptyBuffers();
            } else if (!isEsc) {
              // Unsupported escape sequence, resynchronize on the next datagram.
              ++_droppedFrames;
              emptyBuffers();
            }
          } else if (isEsc) {
            _inEscape = true;
          }
        }
        break;
    }

    if (_bufferPtr >= P182_SML_MSG_BUFFER_SIZE) {
      // Software buffer full without a valid end-of-frame; drop and resync.
      ++_droppedFrames;
      emptyBuffers();
    }
  }
  return decoded;
}

#endif // ifdef USES_P182

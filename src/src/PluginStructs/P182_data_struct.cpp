#include "../PluginStructs/P182_data_struct.h"

#ifdef USES_P182

# include "../Helpers/ESPEasy_time_calc.h"

const uint8_t P182_data_struct::FrameStart[8] = { 0x1b, 0x1b, 0x1b, 0x1b, 0x01, 0x01, 0x01, 0x01 };
const uint8_t P182_data_struct::FrameEnd[5]   = { 0x1b, 0x1b, 0x1b, 0x1b, 0x1a };

P182_data_struct::P182_data_struct(ESPEasySerialPort port,
                                   int8_t            rxPin,
                                   int8_t            txPin,
                                   unsigned long     baudrate)
  : _port(port), _rxPin(rxPin), _txPin(txPin), _baudrate(baudrate)
{}

P182_data_struct::~P182_data_struct() {
  delete _easySerial;
  _easySerial = nullptr;
}

bool P182_data_struct::init() {
  delete _easySerial;
  _easySerial = new (std::nothrow) ESPeasySerial(_port, _rxPin, _txPin, false, P182_SERIAL_RX_BUFFER);

  if (!isInitialized()) {
    return false;
  }
  _easySerial->begin(_baudrate);
  emptyBuffers();

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    addLog(LOG_LEVEL_INFO, strformat(F("MT681: Init RX:%d TX:%d @ %u baud"), _rxPin, _txPin, _baudrate));
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
  _RState    = P182_ReceiveState::WaitForBegin;
}

/**
 * Decode an SML value directly following an OBIS code.
 *
 * Scans forward from pos for the SML type tag, which is either
 *   0x59 = Integer64 (8 bytes, big endian) preceded by the 0xff unit/scaler byte, or
 *   0x55 = Integer32 (4 bytes, big endian) preceded by 0x00.
 */
bool P182_data_struct::getSMLValue(size_t pos, double& result) const {
  if (pos == 0) { return false; }

  const size_t scanEnd = std::min(pos + 32, static_cast<size_t>(P182_SML_MSG_BUFFER_SIZE));

  while (pos < scanEnd) {
    if (((_buffer[pos] == 0x59) && (_buffer[pos - 1] == 0xff)) ||
        ((_buffer[pos] == 0x55) && (_buffer[pos - 1] == 0x00))) {
      break;
    }
    ++pos;
  }

  if (pos >= scanEnd) { return false; }

  const uint8_t tag = _buffer[pos];
  ++pos;

  if (tag == 0x59) {
    // Integer64, big endian. The counter is reported in 0.0001 kWh units.
    if ((pos + 8) > P182_SML_MSG_BUFFER_SIZE) { return false; }
    int64_t tmp = 0;

    for (uint8_t i = 0; i < 8; ++i) {
      tmp = (tmp << 8) | static_cast<int64_t>(_buffer[pos + i]);
    }
    result = static_cast<double>(tmp) / 10000.0;
    return true;
  }

  // Integer32, big endian. Momentary power in Watt, may be negative.
  if ((pos + 4) > P182_SML_MSG_BUFFER_SIZE) { return false; }
  uint32_t raw = 0;

  for (uint8_t i = 0; i < 4; ++i) {
    raw = (raw << 8) | static_cast<uint32_t>(_buffer[pos + i]);
  }
  result = static_cast<double>(static_cast<int32_t>(raw));
  return true;
}

/**
 * Find the next "list entry with 6-byte OBIS code" marker (0x77 0x07) and,
 * if it carries an OBIS code we care about, decode the value behind it.
 */
size_t P182_data_struct::findValue(size_t pos) {
  while (pos < P182_SML_MSG_BUFFER_SIZE) {
    if ((_buffer[pos] == 0x77) && ((pos + 1) < P182_SML_MSG_BUFFER_SIZE) && (_buffer[pos + 1] == 0x07)) {
      pos += 3;
      break;
    }
    ++pos;
  }

  // Need bytes pos+1 .. pos+3 for the OBIS match.
  if ((pos + 3) >= P182_SML_MSG_BUFFER_SIZE) { return P182_SML_MSG_BUFFER_SIZE; }

  const uint8_t o1 = _buffer[pos + 1];
  const uint8_t o2 = _buffer[pos + 2];
  const uint8_t o3 = _buffer[pos + 3];
  double value{};

  if ((o1 == 0x01) && (o2 == 0x08) && (o3 == 0x01)) {          // 1.8.0  Bezug (import) total
    if (getSMLValue(pos, value)) { _buyCounterTotal = value; }
  } else if ((o1 == 0x02) && (o2 == 0x08) && (o3 == 0x01)) {   // 2.8.0  Einspeisung (export) total
    if (getSMLValue(pos, value)) { _sellCounterTotal = value; }
  } else if ((o1 == 0x10) && (o2 == 0x07) && (o3 == 0x00)) {   // 16.7.0 momentary active power
    if (getSMLValue(pos, value)) { _currentPower = value; }
  }
  return pos;
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
    emptyBuffers();
    return false;
  }

  bool     decoded   = false;
  uint16_t processed = 0;

  while (_easySerial->available() && (processed < P182_MAX_BYTES_PER_LOOP)) {
    ++processed;
    _buffer[_bufferPtr++] = static_cast<uint8_t>(_easySerial->read());

    switch (_RState) {
      case P182_ReceiveState::WaitForBegin:

        if (_buffer[_bufferPtr - 1] == FrameStart[_bufferPtr - 1]) {
          if (_bufferPtr == sizeof(FrameStart)) {
            _RState = P182_ReceiveState::WaitForEnd; // Start sequence complete
          }
        } else {
          _bufferPtr = 0;                            // Not the start sequence, resync
        }
        break;

      case P182_ReceiveState::WaitForEnd:

        // The byte just read is the "number of fill bytes" field; the end signature
        // sits immediately before it, hence the -2 .. -6 offsets.
        if ((_bufferPtr >= 6) &&
            (_buffer[_bufferPtr - 2] == FrameEnd[4]) &&
            (_buffer[_bufferPtr - 3] == FrameEnd[3]) &&
            (_buffer[_bufferPtr - 4] == FrameEnd[2]) &&
            (_buffer[_bufferPtr - 5] == FrameEnd[1]) &&
            (_buffer[_bufferPtr - 6] == FrameEnd[0])) {
          _RState = P182_ReceiveState::GetChecksum;
        }
        break;

      case P182_ReceiveState::GetChecksum:
        // First CRC byte is in the buffer now, the next iteration fetches the second.
        _RState = P182_ReceiveState::CheckCRC;
        break;

      case P182_ReceiveState::CheckCRC:

        if (checkCRC(_bufferPtr - 2)) {
          size_t pos = 0;

          for (uint8_t i = 0; (i < 12) && (pos < P182_SML_MSG_BUFFER_SIZE); ++i) {
            pos = findValue(pos);
          }
          _haveData = true;
          decoded   = true;
        }
        emptyBuffers();
        break;

      default:
        emptyBuffers(); // Should not be reachable
        break;
    }

    if (_bufferPtr >= P182_SML_MSG_BUFFER_SIZE) {
      // Software buffer full without a valid end-of-frame; drop and resync.
      emptyBuffers();
    }
  }
  return decoded;
}

#endif // ifdef USES_P182

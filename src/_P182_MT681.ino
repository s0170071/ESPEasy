#include "_Plugin_Helper.h"
#ifdef USES_P182

// #######################################################################################################
// ######################### Plugin 182: ISKRA MT681 / eHZ 681 smart meter (SML) #########################
// #######################################################################################################

/*
   Reads the SML datagram an ISKRA MT681 (eHZ 681) pushes out of its optical D0 interface
   once per second and decodes the OBIS entries:

     1.8.0  total energy drawn from the grid  -> StromBezug
     2.8.0  total energy fed into the grid    -> StromEinspeisung
     16.7.0 momentary active power            -> WirkleistungTotal (negative = feeding in)

   Wiring: an IR read head on the meter, its data output to the configured RX GPIO.
   The interface is receive-only; leaving TX unassigned is fine and recommended.
   Default line settings of the MT681 are 9600 baud 8N1.
   The selected task port is independent of the UART0 serial console.
   For the existing openHAB Sensors/EHZ topics, enable Controller 1 on this task
   and name the values PowerSell, PowerBuy, CurrentPower (in that order).

   Original plugin by s0170071. Ported to the current ESPEasy API and to ESP32 by
   replacing the ESP8266-only Serial.swap() with a regular ESPEasySerial port selection.
 */

/** Changelog:
 * 2026-10-17: Rewrote the SML decoder to follow the specification: transport layer is parsed in
 *             4-byte aligned words (escape sequences, padding, CRC16 over the whole datagram),
 *             the payload is decoded with a real TL (type/length) parser instead of byte pattern
 *             scanning, honouring unit and scaler. Fixed the OBIS matching (1.8.0/2.8.0 are
 *             ...08 00 ff, the old code matched the tariff 1.8.1/2.8.1) and the 512 byte
 *             datagram limit.
 * 2026-10-17: Validate UART startup, add optional RX inversion and receive diagnostics.
 *             Document the existing openHAB value-name mapping.
 * 2026-09-16: Ported to current plugin API (ESPEasySerial, PluginTaskData, UserVar setters).
 *             Fixed the Integer64 decode, which shifted past the width of the accumulator,
 *             and the unbounded forward scan in getSMLValue().
 *             Dropped the custom /<taskname> large-font web page; it relied on the old
 *             TXBuffer streaming API and on plugins being able to register URL handlers.
 * 2023-12-26: Original version.
 */

# define PLUGIN_182
# define PLUGIN_ID_182         182
# define PLUGIN_NAME_182       "Energy (SML) - ISKRA MT681 [TESTING]"
# define PLUGIN_VALUENAME1_182 "StromEinspeisung"
# define PLUGIN_VALUENAME2_182 "StromBezug"
# define PLUGIN_VALUENAME3_182 "WirkleistungTotal"

# include "src/PluginStructs/P182_data_struct.h"

# define P182_BAUDRATE       PCONFIG_LONG(0)
# define P182_BAUDRATE_LABEL F("p182_baud")
# define P182_INVERT_RX      PCONFIG(0)
# define P182_INVERT_LABEL   F("p182_invert")
# define P182_RX_PULLUP      PCONFIG(1)
# define P182_PULLUP_LABEL   F("p182_pullup")


boolean Plugin_182(uint8_t function, struct EventStruct *event, String& string)
{
  boolean success = false;

  switch (function)
  {
    case PLUGIN_DEVICE_ADD:
    {
      auto& dev = Device[++deviceCount];
      dev.Number         = PLUGIN_ID_182;
      dev.Type           = DEVICE_TYPE_SERIAL;
      dev.VType          = Sensor_VType::SENSOR_TYPE_TRIPLE;
      dev.FormulaOption  = true;
      dev.ValueCount     = 3;
      dev.SendDataOption = true;
      dev.TimerOption    = true;
      dev.PluginStats    = true;
      break;
    }

    case PLUGIN_GET_DEVICENAME:
    {
      string = F(PLUGIN_NAME_182);
      break;
    }

    case PLUGIN_GET_DEVICEVALUENAMES:
    {
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_182));
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[1], PSTR(PLUGIN_VALUENAME2_182));
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[2], PSTR(PLUGIN_VALUENAME3_182));
      break;
    }

    case PLUGIN_GET_DEVICEGPIONAMES:
    {
      serialHelper_getGpioNames(event, false, true); // TX is optional, the meter only sends
      break;
    }

    case PLUGIN_SET_DEFAULTS:
    {
      P182_BAUDRATE = 9600;
      P182_INVERT_RX = 0;
      P182_RX_PULLUP = 1;
      success       = true;
      break;
    }

    case PLUGIN_WEBFORM_SHOW_CONFIG:
    {
      string += serialHelper_getSerialTypeLabel(event);
      success = true;
      break;
    }

    case PLUGIN_WEBFORM_LOAD:
    {
      addFormNumericBox(F("Baud Rate"), P182_BAUDRATE_LABEL, P182_BAUDRATE, 300, 115200);
      addUnit(F("baud"));
      addFormNote(F("The MT681 optical interface uses 9600 baud, 8N1."));
      addFormCheckBox(F("Invert RX"), P182_INVERT_LABEL, P182_INVERT_RX);
      addFormNote(F("Enable for optical read heads with an inverted UART output."));
      addFormCheckBox(F("RX pull-up"), P182_PULLUP_LABEL, P182_RX_PULLUP);
      addFormNote(F("Enable the internal pull-up on RX, for read heads with an open-collector output."));
      const P182_data_struct *data =
        static_cast<P182_data_struct *>(getPluginTaskData(event->TaskIndex));
      if ((nullptr != data) && data->isInitialized()) {
        addFormNote(strformat(F("SML RX bytes: %u; frames started: %u; valid frames: %u; CRC errors: %u; dropped frames: %u; last frame length: %u"),
                              data->receivedBytes(), data->startedFrames(), data->validFrames(),
                              data->crcErrors(), data->droppedFrames(), data->lastFrameLength()));
        addFormNote(concat(F("First RX bytes: "), data->receiveSample()));
      } else {
        addFormNote(F("SML serial port is not initialized."));
      }
      success = true;
      break;
    }

    case PLUGIN_WEBFORM_SAVE:
    {
      P182_BAUDRATE = getFormItemInt(P182_BAUDRATE_LABEL);
      P182_INVERT_RX = isFormItemChecked(P182_INVERT_LABEL);
      P182_RX_PULLUP = isFormItemChecked(P182_PULLUP_LABEL);
      success       = true;
      break;
    }

    case PLUGIN_INIT:
    {
      if (P182_BAUDRATE <= 0) { P182_BAUDRATE = 9600; }

      const ESPEasySerialPort port = static_cast<ESPEasySerialPort>(CONFIG_PORT);

      initPluginTaskData(event->TaskIndex,
                         new (std::nothrow) P182_data_struct(port,
                                                             CONFIG_PIN1,
                                                             CONFIG_PIN2,
                                                             static_cast<unsigned long>(P182_BAUDRATE),
                                                             P182_INVERT_RX != 0,
                                                             P182_RX_PULLUP != 0));
      P182_data_struct *P182_data =
        static_cast<P182_data_struct *>(getPluginTaskData(event->TaskIndex));

      success = (nullptr != P182_data) && P182_data->init();
      break;
    }

    case PLUGIN_TEN_PER_SECOND:
    {
      P182_data_struct *P182_data =
        static_cast<P182_data_struct *>(getPluginTaskData(event->TaskIndex));

      if ((nullptr == P182_data) || !P182_data->isInitialized()) {
        break;
      }

      if (P182_data->loop() && (Settings.TaskDeviceTimer[event->TaskIndex] == 0)) {
        // No interval configured, so publish as soon as a datagram is decoded.
        Plugin_182_setValues(event);
        sendData(event);
      }
      success = true;
      break;
    }

    case PLUGIN_READ:
    {
      P182_data_struct *P182_data =
        static_cast<P182_data_struct *>(getPluginTaskData(event->TaskIndex));

      if ((nullptr == P182_data) || !P182_data->hasValue()) {
        break;
      }

      Plugin_182_setValues(event);

      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        addLog(LOG_LEVEL_INFO, strformat(F("MT681: P %.0f W, in %.4f, out %.4f"),
                                         P182_data->getCurrentPower(),
                                         P182_data->getBuyCounterTotal(),
                                         P182_data->getSellCounterTotal()));
      }
      success = true;
      break;
    }
  }

  return success;
}

// Takes only an EventStruct: the Arduino .ino preprocessor hoists a prototype for this
// function above the #include of P182_data_struct.h, so the signature must not mention
// any plugin-specific type.
void Plugin_182_setValues(struct EventStruct *event) {
  P182_data_struct *P182_data =
    static_cast<P182_data_struct *>(getPluginTaskData(event->TaskIndex));

  if (nullptr == P182_data) { return; }

  UserVar.setFloat(event->TaskIndex, 0, static_cast<float>(P182_data->getSellCounterTotal()));
  UserVar.setFloat(event->TaskIndex, 1, static_cast<float>(P182_data->getBuyCounterTotal()));
  UserVar.setFloat(event->TaskIndex, 2, static_cast<float>(P182_data->getCurrentPower()));
}

#endif // USES_P182

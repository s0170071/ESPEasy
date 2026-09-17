# ESPEasy — instructions for coding agents

ESPEasy is Arduino-framework firmware for ESP8266 / ESP82xx and the whole ESP32 family
(classic, solo1, S2, S3, C2, C3, C5, C6, C61, P4). It turns a bare module into a
configurable sensor/actuator node with a built-in web GUI, rules engine, and a set of
controllers that push values to MQTT, Domoticz, Home Assistant, ThingSpeak, and more.

Target branch for new work is `mega`. `v2.0` receives bug fixes only.

## Repository layout

| Path | Contents |
| --- | --- |
| `src/ESPEasy.ino` | Entry point (`setup()` / `loop()`). |
| `src/_P###_Name.ino` | Plugins — one file per sensor/actuator (~168 of them). |
| `src/_C###.cpp` | Controllers (MQTT, Domoticz, HTTP, …). |
| `src/_N00#_Name.cpp` | Notifiers (e-mail, buzzer). |
| `src/src/PluginStructs/P###_data_struct.{h,cpp}` | Per-plugin state and logic; the `.ino` stays a thin dispatcher. |
| `src/src/CustomBuild/define_plugin_sets.h` | The build-configuration brain — resolves every `PLUGIN_BUILD_*`, `USES_*`, `FEATURE_*` and `WEBSERVER_*` macro. |
| `src/src/WebServer/` | Web GUI pages. |
| `src/src/Static/WebStaticData.{h,cpp}` | CSS/JS/favicon blobs compiled into the firmware. |
| `src/src/{Helpers,Globals,DataStructs,DataTypes,Commands,ESPEasyCore}/` | Shared runtime code. |
| `static/` | Source CSS/JS; the minified+gzipped forms are embedded via `WebStaticData.h`. |
| `boards/`, `boards/partitions/` | Custom board JSONs and partition tables. |
| `tools/pio/` | PlatformIO pre/post build scripts. |
| `lib/` | Vendored libraries (`lib_ldf_mode = chain`, `lib_compat_mode = strict`). |
| `docs/` | Sphinx documentation sources. |

## Build system

PlatformIO, driven by `platformio.ini` plus a per-chip family set of `extra_configs`
(`platformio_esp32_envs.ini`, `platformio_esp82xx_envs.ini`, …). There are ~184
environments. Two settings in `[platformio]` matter a lot:

- `core_dir = .platformio` — the toolchain lives **inside the repo**, not in `~/.platformio`.
  Always invoke PlatformIO from the repo root so it picks this up.
- `build_cache_dir = .cache`.

### Environment naming

`<pluginset>_<chip>_<flashlayout>`, e.g. `normal_ESP32_4M316k`,
`collection_C_ESP8266_4M1M`, `max_ESP32_16M8M`, `custom_ESP32c3_4M316k`.
`4M316k` = 4 MB flash, 316 kB filesystem; the rest is app + OTA.

### Common commands

```bash
# Build the env named in default_envs
pio run

# Build / upload / monitor a specific env
pio run -e normal_ESP32_4M316k
pio run -e normal_ESP32_4M316k -t upload --upload-port COM7
pio device monitor -e normal_ESP32_4M316k --port COM7

# Inspect how an env actually resolves (very useful before a long build)
pio project config --json-output
```

Do **not** hard-code a COM port or `/dev/ttyUSB*` into the `.ini` files — port numbers are
per-machine. Pass `--upload-port` / `--port` on the command line instead.

Builds are slow (several minutes for a full tree). Prefer changing one env and reusing
the build cache over switching envs back and forth.

## Build configuration model

Everything is resolved by the preprocessor in `define_plugin_sets.h`. The layers:

1. **Plugin sets** — `PLUGIN_BUILD_NORMAL`, `PLUGIN_BUILD_COLLECTION_A..H`,
   `PLUGIN_BUILD_TESTING`, `PLUGIN_BUILD_DEV`, `PLUGIN_BUILD_MAX`, `PLUGIN_BUILD_CUSTOM`,
   `PLUGIN_BUILD_MINIMAL_OTA`. Sets are cumulative: `DEV` pulls in `COLLECTION`, which
   pulls in `NORMAL`.
2. **Individual plugins/controllers** — `USES_P122`, `USES_C016`, `USES_N001`, …
3. **Features** — `FEATURE_ETHERNET`, `FEATURE_MQTT_TLS`, `FEATURE_PLUGIN_STATS`,
   `FEATURE_CHART_JS`, … Most are `0`/`1` valued, a few are bare `#define`s.
4. **Web GUI pages** — `WEBSERVER_DEVICES`, `WEBSERVER_RULES`, `WEBSERVER_TOOLS`, …
   All default to *on* unless `WEBSERVER_CUSTOM_BUILD_DEFINED` is set.

### Size-reduction switches (important)

`LIMIT_BUILD_SIZE` and `PLUGIN_BUILD_MINIMAL_OTA` silently `#undef` things to make a
build fit. Among them: `WEBSERVER_TIMINGSTATS`, `WEBSERVER_CSS`, `WEBSERVER_INCLUDE_JS`
and `EMBED_ESPEASY_DEFAULT_MIN_CSS`, and they force `WEBSERVER_USE_CDN_JS_CSS`.

`WEBSERVER_USE_CDN_JS_CSS` means the GUI's CSS/JS are **not** in the firmware and are
fetched from a CDN at runtime. If you need a GUI that works on an isolated network with
an empty filesystem, the build must avoid `LIMIT_BUILD_SIZE` and keep `WEBSERVER_CSS`,
`WEBSERVER_INCLUDE_JS` and `EMBED_ESPEASY_DEFAULT_MIN_CSS` defined. See the shared
`[all_ui_pages]` section in `platformio.ini` for the full flag list (the older
`[all_ui_sht2x]` is the same list plus `-DUSES_P122`).

Note that the CodeMirror rules editor (`FEATURE_RULES_EASY_COLOR_CODE`) is *always*
CDN- or filesystem-served; it is never embedded.

### Custom builds

Envs with `custom` in the name run `tools/pio/pre_custom_*.py`. That script checks for
`src/Custom.h`:

- **present** → only `-DUSE_CUSTOM_H` is added and `src/Custom.h` decides everything.
  Copy `src/Custom-sample.h` to start. This file is **gitignored** — never commit it and
  never assume it exists.
- **absent** → the script injects a hard-coded default define list (see the script).

Those script-injected defines go into `CXXFLAGS`, while `.ini` `build_flags` go into
`CPPDEFINES`. Both reach C++ translation units, but only `build_flags` reach `.c` files.

## Coding conventions

- Plugins are `.ino` files but are compiled as C++. Keep the `.ino` a dispatcher over the
  `PLUGIN_*` cases and put real logic in `src/src/PluginStructs/P###_data_struct.*`.
- Guard every plugin file with `#ifdef USES_P###`.
- Use `F()` / `PSTR()` / `strformat()` for literals — flash is the scarce resource.
- Register new value names through `PLUGIN_GET_DEVICEVALUENAMES` and keep
  `dev.ValueCount` and `dev.VType` in sync in `PLUGIN_DEVICE_ADD`.
- `uncrustify.cfg` in the repo root defines the formatting style.
- **Warnings are treated as errors in CI.** A build that compiles locally with warnings
  will fail the PR check.

### Adding a plugin

1. Create `src/_P###_Name.ino` (+ a data struct under `src/src/PluginStructs/`).
2. Add `USES_P###` to the right set in `define_plugin_sets.h`.
3. New and unproven → wrap in `PLUGIN_BUILD_DEV` and put `[DEVELOPMENT]` in the plugin
   name. Working but not widely tested → `PLUGIN_BUILD_TESTING` and `[TESTING]`.
   Only a maintainer promotes a plugin to `COLLECTION` or `NORMAL`.
4. Document it under `docs/source/`.

## Gotchas

- **ESP8266 concatenation.** ESP8266 envs run `tools/pio/concat_cpp_files.py`, which
  concatenates sources into `src/src/*_tmp/` directories to cut build time. Those are
  generated — edit the real sources, not the `_tmp` copies.
- **Flash budget.** A 4 MB ESP32 using `esp32_4M` gets a 1810 kB app partition
  (`boards/partitions/esp32_partition_app1810k_spiffs316k.csv`). Enabling many plugins
  plus the full GUI plus TLS will overflow it. Check the size report at the end of a build.
- **Don't casually flip `default_envs`** in `platformio.ini` — it is a tracked file and
  other people's workflows read it. Use `pio run -e <env>` for one-off builds.
- **`.ino` prototype hoisting.** PlatformIO runs the Arduino preprocessor over `.ino`
  files: it emits forward declarations for every free function at the *top* of the
  translation unit, above your `#include` of the plugin's data struct. A helper whose
  signature mentions a plugin-specific type therefore fails with
  `'P###_data_struct' has not been declared`. Keep free functions in an `.ino` limited to
  types the core already knows (`EventStruct`, `String`, …) and look the task data up
  inside the function body, or move the helper into the `.cpp`.
- **`PLUGIN_BUILD_CUSTOM` suppresses `PLUGIN_BUILD_NORMAL`** and also skips the block of
  `FEATURE_*` defaults near the top of `define_plugin_sets.h` (`FEATURE_I2CMULTIPLEXER`,
  `FEATURE_TIMING_STATS`, `FEATURE_SSDP`, …). A custom build must set the features it
  wants explicitly. To pick an exact plugin set, extend `esp32_common_LittleFS_ETH`
  rather than `esp32_custom_base_LittleFS` — the latter attaches
  `tools/pio/pre_custom_esp32.py`, which injects ~29 plugin defines of its own when
  `src/Custom.h` is absent. See `[env:energy_climate_ESP32_4M316k]` for a worked example.
- **P140 (CardKB) is force-enabled on every ESP32 build** (`define_plugin_sets.h:3058`).
  Opt out with `-DUN_USES_P140` if you are counting bytes or plugin slots.
- **`dev.Type = DEVICE_TYPE_I2C` is load-bearing.** Core keys a lot of behaviour off it,
  and changing an I2C plugin to some other `DEVICE_TYPE_*` (e.g. `DEVICE_TYPE_DUAL`, to
  get per-task SDA/SCL pins) silently removes all of it, with no compile error:
  - `devicePage_show_I2C_config()` is only called for `DEVICE_TYPE_I2C`
    (`WebServer/DevicesPage.cpp:1243`), so the task page loses the *I2C Bus* selector and
    the *Multiplexer channels* / *Connected to* selectors.
  - `prepare_I2C_by_taskIndex()` and `post_I2C_by_taskIndex()` return early for other
    types (`Globals/Plugins.cpp:214`), so the multiplexer channel is never actually
    switched at runtime and the slow/fast clock-speed handling is skipped.

  Don't hand-roll per-task pins: `FEATURE_I2C_MULTIPLE` is already on by default for
  ESP32 (`define_plugin_sets.h:4303`) and gives two independent buses, each with its own
  SDA/SCL *and its own multiplexer*. Calling `I2CBegin()` from inside a plugin is also
  actively harmful next to a multiplexer — it re-initialises the bus after core has
  already selected the channel.
- **I2C multiplexer settings live on `/interfaces_i2c`** (Interfaces → I²C sub-tab), not
  on the Hardware page where older versions kept them. The per-task channel selector only
  renders once *both* a type and an address are set — `isI2CMultiplexerEnabled()` in
  `Helpers/Hardware_I2C.cpp` requires `type != I2C_MULTIPLEXER_NONE && addr != -1`. An
  absent selector usually means unconfigured hardware, not a missing build flag.
- **Don't mutate `EventStruct` members** passed to a `PLUGIN_WRITE` handler.
- Keep PRs focused: one feature or fix per PR (see `.github/CONTRIBUTING.md`).

### Windows toolchain traps

These two bite on Windows and look like code failures when they are not:

- **Console encoding.** esptool 5.x prints Unicode progress bars. Under the default
  cp1252 code page, PlatformIO dies mid-flash with
  `UnicodeEncodeError: 'charmap' codec can't encode characters`, leaving the board
  half-written. Export `PYTHONIOENCODING=utf-8` (and `PYTHONUTF8=1`) before any
  `pio run`, or set the console to `chcp 65001`. The same problem is what suppresses the
  "Firmware metrics can not be shown" table.
- **Broken esptool in the project venv.** The espressif32 platform installs esptool into
  `.platformio/penv` as an *editable* install. The real source lives in
  `.platformio/tools/tool-esptoolpy/`, while `.platformio/packages/tool-esptoolpy/`
  holds only `package.json` + `tools.json`. If the editable link ends up pointing at the
  `packages/` stub you get `ModuleNotFoundError: No module named 'esptool'` when
  `bootloader.bin` is generated. Repair with:

  ```bash
  .platformio/penv/Scripts/python.exe -m pip uninstall -y esptool
  .platformio/penv/Scripts/python.exe -m pip install .platformio/tools/tool-esptoolpy
  .platformio/penv/Scripts/esptool.exe version   # should print 5.x
  ```

- **Port held open.** A `pio device monitor` (including the VS Code PlatformIO monitor)
  keeps the COM port locked, and a cancelled upload can leave an orphan `esptool.exe`
  holding both the port and the `.bin`. Close the monitor and kill stragglers before
  re-flashing, otherwise you get
  `The process cannot access the file because it is being used by another process`.
- **Uploads that hang without output.** An `esptool.exe` that sits on the port
  accumulating no CPU time is wedged, not working — kill the `pio` / `cmd` / `esptool`
  chain and retry; the retry normally succeeds. Distinguish it from simple buffering:
  piping `pio run` into `tail`/`head` can withhold *all* output until exit, so redirect
  to a file (`> upload.log 2>&1`) with `PYTHONUNBUFFERED=1` when you need to watch
  progress. Check liveness with
  `Get-Process esptool | Select-Object Id,StartTime,CPU`.
- **Benign noise.** `ERROR: MSys/Mingw is not supported`, `*** Error 1`,
  `tool-esptoolpy does not appear to be a Python project` and `'uv' is not recognized`
  appear in successful builds. Judge the build by the final `[SUCCESS]` line.

## Verifying a change on real hardware

```bash
# Identify the attached board first — chip revision and flash size decide the env
python .platformio/packages/tool-esptoolpy/esptool.py --port COM7 flash_id

pio run -e <env> -t upload --upload-port COM7
pio device monitor -e <env> --port COM7    # 115200 baud
```

`pio run -t upload` writes only the bootloader (0x1000), partition table (0x8000),
boot_app0 (0xe000) and the app (0x10000). It does **not** touch the filesystem
partition, so the unit's settings — including WiFi credentials in `security.dat` —
survive a reflash as long as the partition layout is unchanged. Back it up anyway
before flashing an unfamiliar unit; for the `esp32_4M` layout the filesystem lives at
`0x3B1000`, size `0x4F000` (see `boards/partitions/`):

```bash
esptool --port COM7 read-flash  0x3B1000 0x4F000 backup/spiffs_settings.bin
esptool --port COM7 write-flash 0x3B1000          backup/spiffs_settings.bin  # restore
```

### Inspecting a running unit without damaging its config

Read-only endpoints: `/json` (task list and current values), `/csv`, `/sysinfo`, and
`/download` (fetches `config.dat`). Use these to check state.

**Config pages are not read-only, even over GET.** ESPEasy makes no distinction between
a GET and a form submit: `handle_devices()` reads `TDNUM` with
`getFormItemInt(F("TDNUM"), 0)`, and a *missing* argument therefore reads as `0` =
"device: None". Requesting `/devices?index=1&edit=1` to have a look at task 1 hits the
"device changed" branch at `WebServer/DevicesPage.cpp:151`, wipes the task and saves
settings. The same pattern applies to `/interfaces_i2c`, `/hardware` and friends: a
partial query string clears every field you left out.

If you must render a config page, send every field the form contains — or, for a task,
at minimum the correct `TDNUM` — and prefer `/json` whenever it will answer the question.

A healthy boot prints, at `LOG_LEVEL_INFO`:

```
INIT : Booting version: <binary name>, (<origin>) <git build> (<core lib>)
INIT : Free RAM:<n>
INIT : <boot cause> #<bootCounter>
INIT : Normal boot #<n>
```

followed by WiFi/AP setup lines. A boot loop shows a repeating `rst cause` / backtrace
instead, and `bootFailedCount` climbing. For readable ESP32 backtraces, temporarily
switch `monitor_filters` from `direct` to `esp32_exception_decoder` in the chip's `.ini`.

Some USB-serial bridges (CH9102, CP210x) reset the board when the monitor asserts
DTR/RTS; `[common]` sets `monitor_dtr = 0` and `monitor_rts = 0` to avoid that.

## Further reading

- `.github/CONTRIBUTING.md`
- <https://espeasy.readthedocs.io/>
- <https://www.letscontrolit.com/wiki/index.php/ESPEasyDevelopmentGuidelines>

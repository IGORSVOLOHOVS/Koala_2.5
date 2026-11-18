# Software Requirements Specification: Koala 2.5 library (libkoala)

Reverse-engineered on 2026-11-18 from commit `dc18adb`. Every requirement states only
behaviour present in the code and cites its source. Hub document:
[docs/README.md](docs/README.md).

<!-- jira-sync:start -->
Jira: KAN-123

## Tasks

This list is one ticket in Jira project KAN, [KAN-123](https://igorsvolohovs.atlassian.net/browse/KAN-123). Each line is a checkbox there: tick it in Jira and the sync ticks it here; add a checkbox there and it is appended here. Done items are the features described in section 3; open items are what the project declares but the code does not do yet.

- [x] Initialisation and shutdown (section 3.1) <!-- spec-initialisation-and-shutdown -->
- [x] Serial transport (section 3.2) <!-- spec-serial-transport -->
- [x] Command protocol (common behaviour) (section 3.3) <!-- spec-command-protocol-common-behaviour -->
- [x] Configuration (section 3.4) <!-- spec-configuration -->
- [x] Motors (section 3.5) <!-- spec-motors -->
- [x] Sensors (section 3.6) <!-- spec-sensors -->
- [x] I2C, IO and power (section 3.7) <!-- spec-i2c-io-and-power -->
- [x] Auto-monitoring reception (section 3.8) <!-- spec-auto-monitoring-reception -->
- [x] Console utilities (section 3.9) <!-- spec-console-utilities -->
- [x] Build (section 3.10) <!-- spec-build -->
- [x] Example programs (section 3.11) <!-- spec-example-programs -->
- [ ] `examples/koala_auto_mode_test.c` (owner's refactoring) (declared, not implemented) <!-- spec-examples-koalaautomodetest-c-owner-s-refactoring -->
- [ ] `test_sound()`, reached by `koala_test` input starting `so` (declared, not implemented) <!-- spec-testsound-reached-by-koalatest-input-starting-so -->
- [ ] `-d` / `--debug` option (declared, not implemented) <!-- spec-d-debug-option -->
- [ ] `KOALA_RS232_MESSAGE_ERROR_CHAR` (`'#'`) (declared, not implemented) <!-- spec-koalars232messageerrorchar -->
- [ ] `koala_getcommand_line_nowait()` in the public API (declared, not implemented) <!-- spec-koalagetcommandlinenowait-in-the-public-api -->
- [ ] Battery, US, GPS, I2C, PID and accelerometer demos in `koala_example` (declared, not implemented) <!-- spec-battery-us-gps-i2c-pid-and-accelerometer-demos-in- -->
- [ ] Drive-mode auto-stop after 100 ms without a key (declared, not implemented) <!-- spec-drive-mode-auto-stop-after-100-ms-without-a-key -->
- [ ] `src/robot` (IR proximity, buzzer, LEDs, switches via I2C registers) (declared, not implemented) <!-- spec-src-robot-ir-proximity-buzzer-leds-switches-via-i2 -->
- [ ] Manual's `template/` and `doc/` directories and top-level `make examples` (declared, not implemented) <!-- spec-manual-s-template-and-doc-directories-and-top-leve -->
- [ ] README "Example" section (declared, not implemented) <!-- spec-readme-example-section -->

<!-- jira-sync:end -->

## 1. Purpose and scope

The system is a C library, `libkoala`, through which a program running on the Koala 2.5
robot's onboard computer commands the robot's microcontroller over a serial line, plus
example programs that use it.

In scope:

- library sources `src/*.c`, `src/*.h` and `src/Makefile`;
- example programs `examples/*.c` and `examples/Makefile`;
- committed build output in `build-x86/` and `examples/`.

Out of scope:

- the microcontroller firmware (not in the repository);
- the robot's operating system and hardware;
- `src/robot`, a text fragment of another API that is not compiled (see 3.12);
- the two copies of the K-Team user manual, used here only as a reference for the serial protocol.

## 2. Context

| Item | Description | Source |
|---|---|---|
| Actor: application programmer | Writes C code that includes `<koala/koala.h>` and links `-lkoala` | `README.md:16-43`, `examples/Makefile:20-23` |
| Actor: operator | Runs the example programs on the robot console and drives them from the keyboard | `examples/koala_test.c:664-699` |
| External system: Koala 2.5 microcontroller | Answers ASCII commands on `/dev/ttyS1`, 115200 baud | `src/koala_robot.h:45-46`; manual chapter "Serial communication protocol" |
| Peripherals reached through it | 2 motors with encoders, 9 ultrasonic sensors, accelerometer, gyroscope, magnetometer, GPS, external I2C bus, 4 IO, 4 power outputs, 2 digital inputs, 2 AD inputs, battery monitor | `src/koala_robot.h:53-158`, `src/koala_robot.c:868-941` |
| Runtime environment | Linux with termios and `getopt_long`; the manual names an onboard Pico-ITX (Intel Atom N2800) with 64-bit Ubuntu | `src/koala_serial.h:37`, `src/koala_init.c:38`; `Koala2.5 User Manual 1.3(EDIT).docx`, "Pico-ITX", "Programming" |
| Committed binaries | ELF 64-bit x86-64; the two executables are marked "for GNU/Linux 3.2.0" (output of `file`); all carry the compiler tag `GCC: (Ubuntu 9.4.0-1ubuntu1~20.04.1) 9.4.0`, and the library's embedded `__DATE__` is `Jul 18 2023` | `build-x86/lib/libkoala.so.1.0`, `examples/koala_test`, `examples/koala_example` |

## 3. Functional requirements

"Manual check" names the `koala_test` menu option that exercises the function
(`examples/koala_test.c:664-692`). There are no automated tests.

### 3.1 Initialisation and shutdown

- **FR-1** The system shall, in `koala_init()`, parse the program's `argv` with `getopt_long` and, for `-v` or `--version`, print `libkoala v1.1 [<build date>]`.
  Source: `src/koala_init.c:79-91`, `src/koala.h:38-39`. Verification: no test.
- **FR-2** The system shall, on the first `koala_init()` call, open `/dev/ttyS1` at B115200. It shall return -1 if the device cannot be opened, 0 on success, and 1 on any call after a successful one (until `koala_exit()`) without reopening.
  Source: `src/koala_init.c:72-111`, `src/koala_robot.c:98-111`, `src/koala_robot.h:45-46`. Verification: no test (every example checks `< 0`).
- **FR-3** The system shall register `koala_exit()` with `atexit()`. At process exit, `koala_exit()` closes and frees the serial descriptor.
  Source: `src/koala_init.c:104,121-129`, `src/koala_robot.c:124-128`, `src/koala_serial.c:106-116`. Verification: no test.

### 3.2 Serial transport

- **FR-4** The system shall open the serial device in read-write mode with `O_NOCTTY` and then switch it to blocking reads. It shall set 8 data bits, no parity, one stop bit, `CLOCAL|CREAD`, `IGNPAR`, raw local mode, `VMIN=1` and `VTIME=1`, and it shall flush pending input. A baud rate of 0 shall become B115200. On open failure it shall return NULL.
  Source: `src/koala_serial.c:63-97`. Verification: no test.
- **FR-5** The system shall read a reply line by waiting up to 500 ms for the first byte, then reading single bytes until CR, LF or 255 bytes. It shall NUL-terminate the line and return its length without the terminator. It shall return -1 for an invalid descriptor, -2 on timeout or `select` error, and -3 if the first `read` returns no byte.
  Source: `src/koala_serial.c:49,210-254`. Verification: no test.
- **FR-6** The system shall provide the same line read with a zero timeout (`koala_rs232_readLine_nowait`, wrapped by `koala_getcommand_line_nowait`).
  Source: `src/koala_serial.c:152-198`, `src/koala_robot.c:179-187`. Verification: manual check `rm`.
- **FR-7** The system shall pass raw reads and writes through to `read()`/`write()` and return -1 for an invalid descriptor.
  Source: `src/koala_serial.c:132-139,271-277`, `src/koala_robot.c:145-152,204-211`. Verification: no test.

### 3.3 Command protocol (common behaviour)

- **FR-8** For each command, the system shall send one frame and then read one reply line. The frame is an upper-case command character, optional comma-separated decimal parameters and a terminating CR. The system shall return -1 when the reply's first character is not the expected acknowledgement: the lower-case command letter, `@` for `&`, or `!` for `?` (table 4.2).
  Source: for example `src/koala_robot.c:237-247,466-473,1115-1122`. Verification: manual check, any option.
- **FR-9** The system shall parse up to `nb_arg` comma-separated integers from a reply. It shall return -1 on a token that is not an integer, and 0 otherwise, including when fewer than `nb_arg` values are present.
  Source: `src/koala_robot.c:61-80`. Verification: no test.

### 3.4 Configuration

- **FR-10** The system shall read the firmware version by sending `B\r` and reading exactly 7 raw bytes. It shall return byte 2 as the version character and `atoi` of bytes 4 onward as the revision. Error codes: -1 for a NULL version pointer, -2 for a NULL revision pointer, -3 for a reply not starting with `b`, -4 for a read count other than 7.
  Source: `src/koala_robot.c:258-286`. Verification: manual check, start-up of `koala_test`.
- **FR-11** The system shall send the auto-monitoring mask `A,<mask>`. Mask bits (`src/koala_robot.h:53-63`): 1 US, 2 speed, 4 position, 8 current, 16 accelerometer, 32 gyroscope, 64 GPS, 128 NMEA, 256 magnetometer; `0x01FF` all, 0 none.
  Source: `src/koala_robot.c:232-247`. Verification: manual check `am`.
- **FR-12** The system shall send `C,<us_mask>,<io_dir>`. The US mask has 9 bits, 511 for all (`src/koala_robot.h:67-77`). The IO direction uses two bits per IO: 00 output, 01 input, 10 PWM servo (`src/koala_robot.h:85-97`).
  Source: `src/koala_robot.c:316-331`. Verification: manual check `pa`.
- **FR-13** The system shall send the PID gains as `H,kp,ki,kd`.
  Source: `src/koala_robot.c:491-503`. Verification: manual check `pi`.
- **FR-14** The system shall send the speed profile as `J,acc_inc,acc_div,min_speed,cst_speed,pos_margin,max_current`.
  Source: `src/koala_robot.c:553-565`. Verification: manual check `sp`.
- **FR-15** The system shall set the encoder values with `I,left,right`.
  Source: `src/koala_robot.c:521-533`. Verification: manual check `re`.

### 3.5 Motors

- **FR-16** The system shall set the closed-loop speed with `D,left,right`, in pulses per 10 ms.
  Source: `src/koala_robot.c:343-358`, `src/koala_robot.h:138`. Verification: manual check `ms`.
- **FR-17** The system shall set the speed with an acceleration ramp with `K,left,right`.
  Source: `src/koala_robot.c:582-594`. Verification: manual check `msp`.
- **FR-18** The system shall set the open-loop PWM with `L,left,right`.
  Source: `src/koala_robot.c:610-622`. Verification: manual check `mso`.
- **FR-19** The system shall set the target position in pulses with `F,left,right`.
  Source: `src/koala_robot.c:398-410`. Verification: manual check `mp`.
- **FR-20** The system shall read the motor speed (`E`), the current (`O`, 0.1 A) and the position (`P`) as two integers each, and return -2 if two integers cannot be parsed.
  Source: `src/koala_robot.c:369-387,693-708,722-737`. Verification: manual check `g`.
- **FR-21** The system shall read the motor control status with `Y` as four integers: the control type 0-6 for each motor and the on-target flag for each motor. It shall return -2 if four integers cannot be parsed.
  Source: `src/koala_robot.c:1031-1069`. Verification: manual check `mst`.

### 3.6 Sensors

- **FR-22** The system shall read the 9 ultrasonic values with `G` into an array. Each value is 0 (object nearer than 25 cm), 25-250 cm, 1000 (no object) or 2000 (sensor disabled). It shall return -2 on a bad token.
  Source: `src/koala_robot.c:415-444`, `src/koala_robot.h:142-144`. Verification: manual check `u`.
- **FR-23** The system shall read 30 accelerometer values (`M`) and 30 gyroscope values (`N`): 10 samples of x, y, z with the newest first.
  Source: `src/koala_robot.c:627-679`, `src/koala_robot.h:113-123`. Verification: manual check `ag`.
- **FR-24** The system shall read the magnetometer by sending `&` and parsing three values from a reply starting with `@`.
  Source: `src/koala_robot.c:459-476`. Verification: manual check `ma`.
- **FR-25** The system shall read GPS data with `Q`, parsing 14 fields: validity flag, satellite count, latitude, N/S, longitude, E/W, `hh:mm:ss`, `dd.mm.yy`, speed in knots and altitude in metres. Time and date shall be written into `struct tm` fields as received. It shall return -2 if 14 fields are not parsed.
  Source: `src/koala_robot.c:743-777`. Verification: manual check `gp`.
- **FR-26** The system shall forward a GPS command as `X,<cmd>`. It shall return -1 when the command is longer than 1020 characters and -2 when the reply does not start with `x`.
  Source: `src/koala_robot.c:1012-1026`. Verification: manual check `gs`.

### 3.7 I2C, IO and power

- **FR-27** The system shall read `n` bytes from the external I2C bus with `R,addr,reg,n`. It shall return -2 on a bad token and -3 when the number of values parsed differs from `n`; parsing stops at 256 values.
  Source: `src/koala_robot.c:795-828`, `src/koala_robot.h:127`. Verification: manual check `ir`.
- **FR-28** The system shall write to the external I2C bus with `W,addr,reg,d1,...,dn`. It shall return -1 when `n` exceeds 253 (`(1024-10)/4`) and -2 when the reply does not start with `w`.
  Source: `src/koala_robot.c:973-998`. Verification: manual check `iw`.
- **FR-29** The system shall scan the external I2C bus with `?` and parse the reply `!,count,addr...`. Error codes: -2 or -3 for a missing or invalid count, -4 for an invalid address, -5 when the number of addresses differs from the count. Reading stops after 128 addresses.
  Source: `src/koala_robot.c:1105-1149`. Verification: manual check `is`.
- **FR-30** The system shall set the four power outputs and the four IO with `S,pwr,io0,io1,io2,io3`.
  Source: `src/koala_robot.c:834-863`, `src/koala_robot.h:101-108`. Verification: manual check `po`.
- **FR-31** The system shall read the IO and input states (`T`) and the two AD inputs (`U`), each as two integers. It shall read the battery (`V`) as four integers: type (0 Li-ION, 1 NiMH, 2 not initialised), voltage in 0.1 V, current in 0.1 A and charge current in 10 mA.
  Source: `src/koala_robot.c:868-957`. Verification: manual checks `io`, `ad`, `b`.
- **FR-32** The system shall reset the microcontroller with `Z`.
  Source: `src/koala_robot.c:1081-1092`. Verification: manual check `rm`.

### 3.8 Auto-monitoring reception

- **FR-33** `koala_get_from_auto_mode()` shall read one line without waiting and return -1 if none is available. Otherwise it shall store the first character in `data->mode` and parse the frame into `koala_auto_data_t` by type: `e`, `g`, `m`, `n`, `o`, `p`, `q`, `$` (raw line copied to `gps_raw`) or `@`. It shall return -2 to -9 on parse errors and -10 for an unknown type.
  Source: `src/koala_robot.c:1164-1222`. Verification: manual check `am`.

### 3.9 Console utilities

- **FR-34** `koala_kbhit()` shall return the number of bytes waiting on stdin (`FIONREAD`, with non-canonical mode set and then restored), or -1 on error.
  Source: `src/koala_utils.c:52-73`. Verification: no test.
- **FR-35** `koala_change_term_mode(1)` shall turn off `ICANON` and `ECHO` on stdin. `koala_change_term_mode(0)` shall restore the saved settings, but only after a previous call with 1.
  Source: `src/koala_utils.c:85-104`. Verification: no test.
- **FR-36** The system shall provide ANSI escape helpers to clear the screen, erase a line and move the cursor to a column, a line or both (origin 1,1).
  Source: `src/koala_utils.c:111-159`. Verification: no test.
- **FR-37** `koala_timeval_diff()` shall return `end - start` in microseconds with `tv_usec` normalised, and store the difference when a non-NULL pointer is given.
  Source: `src/koala_utils.c:162-200`. Verification: no test.

### 3.10 Build

- **FR-38** `make` in `src/` shall compile every `src/*.c` with `-fPIC` and produce `../build-x86/lib/libkoala.so.1.0` and `libkoala.a`. It shall copy `src/*.h` to `../build-x86/include/koala` and create the link `libkoala.so -> libkoala.so.1.0`.
  Source: `src/Makefile:1-9,24,33-50`. Verification: no test.
- **FR-39** `make install` shall copy `libkoala.so.1.0` to `/usr/lib` and the headers to `/usr/include/koala`, and create `/usr/lib/libkoala.so`.
  Source: `src/Makefile:56-65`. Verification: no test.
- **FR-40** `make` in `examples/` shall build each program in `TARGETS` against `../build-x86` with `-lkoala -lm`.
  Source: `examples/Makefile:11-27,39-50,76-78`. Verification: no test (fails at `koala_auto_mode_test`, see 3.12).

### 3.11 Example programs

- **FR-41** `koala_test` shall first initialise the library and read the version, retrying once. It shall then disable auto-monitoring and apply the default PID (10, 3, 1) and the default speed profile (5, 1, 10, 200, 10, 10). It shall install a SIGINT handler that stops the motors in open loop and restores the terminal. Finally it shall loop over the menu until `q`.
  Source: `examples/koala_test.c:52-59,589-660,1360-1362`. Verification: no test.
- **FR-42** `koala_test` drive mode shall map the arrow keys and numpad digits 1-4 and 6-9 to motion with the speed ramp (digit 3 uses closed-loop speed without the ramp). PgUp/PgDn shall change the speed by 1 and Home/End by 25, within 1-500 (default 200). `s`, space or `5` shall stop the motors and `q` shall return to the menu.
  Source: `examples/koala_test.c:128-140,152-376`. Verification: no test.
- **FR-43** `koala_test` option `br` shall run Braitenberg avoidance until a key is pressed:
  - auto-monitoring of the US sensors;
  - distances clipped to the range 25-60;
  - fixed per-sensor weights and a base speed of 200;
  - a battery printout every 30 s.

  Source: `examples/koala_test.c:384-529`. Verification: no test.
- **FR-44** `koala_example` shall run in two phases:
  - calibration: rotate in place at +-50 until the left encoder count corresponds to 2.5 turns of the robot, recording the magnetometer x/y extremes; a key press ends the program;
  - heading control: compute and display the heading, and accept `q`, the left/right arrows, `s`/space, `n` (north) and `a` (an angle in the range 0-360).

  Source: `examples/koala_example.c:246-509`. Verification: no test.

### 3.12 Declared but not implemented

| Item | State | Source |
|---|---|---|
| `examples/koala_auto_mode_test.c` (owner's refactoring) | Does not compile: the macro has a default parameter; `version` and `revision` are undeclared | `examples/koala_auto_mode_test.c:13,173-179`; macro added in `8258133`, declaration removed in `fc06d99` |
| `test_sound()`, reached by `koala_test` input starting `so` | Empty stub returning 0, not in the menu text | `examples/koala_test.c:539-542,1399-1401` |
| `-d` / `--debug` option | Parsed, no effect | `src/koala_init.c:85-86` |
| `KOALA_RS232_MESSAGE_ERROR_CHAR` (`'#'`) | Defined, never used | `src/koala_robot.h:164` |
| `koala_getcommand_line_nowait()` in the public API | Defined and exported, missing from the header | `src/koala_robot.c:179`, `src/koala_robot.h:216-218` |
| Battery, US, GPS, I2C, PID and accelerometer demos in `koala_example` | Commented out | `examples/koala_example.c:92-244` |
| Drive-mode auto-stop after 100 ms without a key | Commented out | `examples/koala_test.c:138,354-366` |
| `src/robot` (IR proximity, buzzer, LEDs, switches via I2C registers) | Not compiled; refers to undefined `koala_I2C_REG_ADDRESS_*` | `src/robot:13-293`, `src/Makefile:1` (`*.c` only) |
| Manual's `template/` and `doc/` directories and top-level `make examples` | Absent from the repository | manual, chapter "Programming" |
| README "Example" section | Placeholder body | `README.md:51-64` |

## 4. Interfaces

### 4.1 Command line (any program that calls `koala_init`)

| Option | Effect | Source |
|---|---|---|
| `-v`, `--version` | Print library version and build date | `src/koala_init.c:48-54,89-91` |
| `-d <arg>`, `--debug` | No effect | `src/koala_init.c:48-54,85-86` |

### 4.2 Serial protocol as implemented

All frames end with CR (`\r`). Replies are read up to CR or LF.

| Command sent | Expected reply | Values parsed | Function | Source line |
|---|---|---|---|---|
| `A,mask` | `a` | - | `koala_configure_auto_monitoring_mode` | `src/koala_robot.c:237` |
| `B` | `b` + 5 bytes + CR (7 bytes raw; manual example `b,A-02`) | version, revision | `koala_get_firmware_version_revision` | `:263` |
| `C,us_mask,io_dir` | `c` | - | `koala_configure_us_io` | `:321` |
| `D,l,r` | `d` | - | `koala_set_motor_speed` | `:348` |
| `E` | `e,l,r` | 2 | `koala_read_motor_speed` | `:374` |
| `F,l,r` | `f` | - | `koala_set_motor_target_position` | `:403` |
| `G` | `g,v1..v9` | 9 | `koala_read_us_sensors` | `:434` |
| `H,p,i,d` | `h` | - | `koala_configure_pid` | `:496` |
| `I,l,r` | `i` | - | `koala_set_position_encoders` | `:526` |
| `J,inc,div,min,cst,margin,maxcur` | `j` | - | `koala_set_speed_profile` | `:558` |
| `K,l,r` | `k` | - | `koala_set_motor_speed_accel` | `:587` |
| `L,l,r` | `l` | - | `koala_set_motor_speed_open_loop` | `:615` |
| `M` | `m,30 values` | 30 | `koala_read_accelerometer` | `:640` |
| `N` | `n,30 values` | 30 | `koala_read_gyroscope` | `:669` |
| `O` | `o,l,r` | 2 | `koala_read_motor_current` | `:698` |
| `P` | `p,l,r` | 2 | `koala_read_motor_position` | `:727` |
| `Q` | `q,...` | 14 | `koala_gps_data` | `:767` |
| `R,addr,reg,n` | `r,v1..vn` | n | `koala_read_i2c` | `:802` |
| `S,pwr,io0,io1,io2,io3` | `s` | - | `koala_set_pwr_io_output` | `:856` |
| `T` | `t,io,in` | 2 | `koala_read_io` | `:886` |
| `U` | `u,ad0,ad1` | 2 | `koala_read_ad` | `:915` |
| `V` | `v,type,volt,cur,chrg` | 4 | `koala_get_battery_data` | `:947` |
| `W,addr,reg,d1..dn` | `w` | - | `koala_write_i2c` | `:983` |
| `X,text` | `x` | - | `koala_send_gps_cmd` | `:1019` |
| `Y` | `y,ctl_l,ctl_r,trg_l,trg_r` | 4 | `koala_get_motor_status` | `:1059` |
| `Z` | `z` | - | `koala_reset_microcontroller` | `:1085` |
| `?` | `!,count,addr...` | 1+count | `koala_scan_i2c` | `:1115` |
| `&` | `@,x,y,z` | 3 | `koala_read_magnetometer` | `:466` |
| (unsolicited) | `e g m n o p q $ @` | per type | `koala_get_from_auto_mode` | `:1180-1219` |

### 4.3 C API return convention

Every command function returns 0 on success and a negative value on error
(`src/koala_robot.c`, `\return` comments). Getters write through pointer arguments.

### 4.4 Make targets

| Directory | Targets | Source |
|---|---|---|
| `src/` | `all` (default), `docs`/`doc`, `install`, `clean`, `depend` | `src/Makefile:24-78` |
| `examples/` | `all` (default), `clean`, `depend` | `examples/Makefile:39-80` |

## 5. Data

| Entity | Content | Source |
|---|---|---|
| `koala_rs232_t` | File descriptor and saved `termios` | `src/koala_serial.h:39-48` |
| `gps_data_t` | Validity flag, satellite count, latitude/longitude with hemisphere characters, `struct tm` time, speed (knots), altitude (m) | `src/koala_robot.h:172-183` |
| `koala_auto_data_t` | Last auto-mode frame: speeds, positions, currents, `us[9]`, `accel[30]`, `gyro[30]`, `magne[3]`, `gps_raw[1024]`, `mode`, `gps` | `src/koala_robot.h:186-202` |
| `KOALA_US_SENSOR_NAMES[9]` | Sensor names from "Left rear" to "Back left" | `src/koala_robot.c:85` |
| Unit factors | `KOALA_PULSE_TO_MM` = pi * 82.5 / 23400 mm; `KOALA_SPEED_TO_MM_S` = that / 0.01 s; `KOALA_ACCEL_G` = 1/16384 g; `KOALA_GYRO_DEG_S` = 0.066 deg/s; `KOALA_AD_TO_V` = 3.3/1024 V; wheel distance 250 mm | `src/koala_robot.h:112-158` |

No data is persisted. Build artefacts are files: `build-x86/lib/libkoala.so.1.0`,
`libkoala.a`, the link `libkoala.so`, and `build-x86/include/koala/*.h`, which are
identical to `src/*.h` when compared with `diff -q --strip-trailing-cr`.

## 6. Non-functional requirements

- **NFR-1** The library shall run on Linux with POSIX termios, `select` and GNU `getopt_long`. Source: `src/koala_serial.c:44-47,169`, `src/koala_init.c:38`.
- **NFR-2** A command reply shall be awaited for at most 500 ms before the first byte; the auto-mode read shall not wait. Source: `src/koala_serial.c:167-168,225-226`.
- **NFR-3** One reply line is limited to 255 characters; command and reply buffers are 1024 bytes. Source: `src/koala_serial.c:49`, `src/koala_robot.h:48`.
- **NFR-4** The library keeps one serial descriptor per process. Source: `src/koala_robot.c:45`. Inference: calls from several threads are not synchronised.
- **NFR-5** Release builds use `-O3 -fPIC`; `DEBUG=1` uses `-g -fPIC`. Source: `src/Makefile:11-15`, `examples/Makefile:13-17`.
- **NFR-6** The source files state the GNU LGPL 2.1 or later. Source: `src/koala.h:5-8` (same header in every `src/*.c` and `src/*.h` file). The repository has no LICENSE file (`d6fa511`).

## 7. Constraints and dependencies

No lock or package files exist. Dependencies named by the build files and includes:

| Dependency | Use | Source |
|---|---|---|
| C compiler (`$(CC)`), `ar` (`$(AR)`) | Build | `src/Makefile:43,45,76` |
| C standard library, `libm` | Runtime | `src/Makefile:17`, `examples/Makefile:20` |
| `termio.h`, `getopt.h`, `fcntl.h`, `sys/time.h` | Linux headers | `src/koala_serial.c:44-47`, `src/koala_init.c:38`, `src/koala_utils.h:38` |
| GNU `cp -s` | Library link | `src/Makefile:50,65` |
| `doxygen` (config written for 1.6.3) | `make docs` | `src/Makefile:31`, `src/Doxyfile:1` |
| `makedepend` | `make depend` | `src/Makefile:71`, `examples/Makefile:73` |

## 8. Acceptance

1. On a Linux host, `make` in `src/` produces `build-x86/lib/libkoala.so.1.0`, `libkoala.a` and `build-x86/include/koala/*.h` (FR-38).
2. `llvm-nm -D` or `nm -D` on the library lists every `koala_*` function defined in `src/*.c`. This held for the committed binary on 2026-11-18.
3. `make` in `examples/` builds `koala_example` and `koala_test`. `koala_auto_mode_test` is expected to fail until the defects in 3.12 are fixed.
4. On the robot, after `sudo make install`, `./koala_test -v` prints `libkoala v1.1 [...]` and the firmware version (FR-1, FR-10). Each menu option named under "Manual check" then returns without an `ERROR` line.
5. With the robot on a stand, `ms 100 100` turns both wheels, `g` shows non-zero speed, and `s` stops them (FR-16, FR-20).

None of these steps was run for this document (see `docs/README.md`, "About this document").

## 9. Open questions

- Why the repository exists and who uses it, for example a course, a lab or a robot in use. The repository does not say.
- Which robot, firmware revision and onboard OS this copy was used with. The manual's revision history mentions "Robot since 2017"; the code gives no hardware revision. Partial answer: the committed binaries were compiled with the GCC 9.4.0 of Ubuntu 20.04 (compiler tag in the binaries); whether that build host was the robot is unknown.
- Whether the committed binaries were built from the current sources. Partial answer: the library's embedded build date is `Jul 18 2023`, the day of commit `10b1006` that committed them, which is after the last change to `src/*.c` (`98dbb69`, 2023-02-10) and to `koala_test.c`/`koala_example.c` (`0c142f6`, 2023-02-14); the exported `koala_*` symbols match the sources. The code itself was not compared.
- Which units are correct where the code comments and the manual disagree: open-loop PWM range +-2000 or +-2940, `max_current` in mA or in 0.1 A, AD range 0-1024 or 0-1023 (`src/koala_robot.c:548,602,902`; manual, commands J, L, U).
- The intended command terminator. The code sends CR (`src/koala_robot.c:237` and others); the manual says "terminated by a line feed", and its notation table swaps the codes of `\r` and `\n`.
- Whether `struct tm` fields from GPS (`tm_mon`, `tm_year` stored as received) are meant to follow `struct tm` conventions (`src/koala_robot.c:776`).
- What was changed in `Koala2.5 User Manual 1.3(EDIT).docx` relative to the PDF.
- Install prefix: `/usr` in `src/Makefile:7` against `/usr/local/lib` in the manual.
- Whether the owner's refactoring of `koala_auto_mode_test.c` was meant to be finished, and whether the K-Team licence header (removed in `6995e37`) was meant to be dropped from it.
- Inference to confirm with a build: compilers that reject implicit function declarations (GCC 14 and later by default) would reject `src/koala_serial.c` (`read`, `write`, `close` without `<unistd.h>`), `src/koala_robot.c:283` (`atoi` without `<stdlib.h>`), and the examples (`usleep`, `sleep`). The committed binaries were built with GCC 9.4.0, which only warns on these.

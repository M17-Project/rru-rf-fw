## rpi-interface Command Set

### Command format

| Command ID | Byte count | Params       |
|------------|------------|--------------|
| 1 byte     | 1 byte     | 0..253 bytes |

### Command list

| Command | Byte count | Action                                | Parameters                | Return value               |
|---------|------------|---------------------------------------|---------------------------|----------------------------|
| 0x00    | 2 (6)      | Ping/pong                             | -                         | 32-bit value (error flags) |
| 0x01    | 6          | Set RX frequency                      | 32-bit frequency in Hz    | 0/1                        |
| 0x02    | 6          | Set TX frequency                      | 32-bit frequency in Hz    | 0/1                        |
| 0x03    | 3          | Set TX power                          | 8-bit value, unsigned[1]  | 0/1                        |
| 0x04    | -          | Reserved                              | -                         | -                          |
| 0x05    | 4          | Set RX frequency correction           | 16-bit value, signed[2]   | 0/1                        |
| 0x06    | 4          | Set TX frequency correction           | 16-bit value, signed[2]   | 0/1                        |
| 0x07    | 3          | Set Automatic Frequency Control (AFC) | 0/1                       | 0/1                        |
| 0x08    | 2          | Transmission start                    | -                         | -                          |
| 0x09    | 3          | Reception stop/start                  | 0/1                       | 0/1 at RX end only         |
| ...     | ...        | ...                                   | ...                       | ...                        |
| 0x80    | 2          | Get IDENT string                      | -                         | IDENT string               |
| 0x81    | 2          | Get device's capabilities             | -                         | 8-bit value                |
| 0x82    | 2          | Get RX frequency                      | -                         | 32-bit frequency in Hz     |
| 0x83    | 2          | Get TX frequency                      | -                         | 32-bit frequency in Hz     |
| 0x84    | 2          | Get TX power                          | -                         | 8-bit value, signed[1]     |
| 0x85    | 2          | Get frequency correction              | -                         | 16-bit value, signed       |
| ...     | ...        | ...                                   | ...                       | ...                        |

All values are little-endian. Return value of 0 means success, any other value is an error code.
Parameter of 0 disables the function, 1 enables it.

[1] 0.25dBm steps, starting at 0.0dBm: P[dBm]=value*0.25<br>
[2] This is an arbitrary unit, **not** ppm<br>

### Device's capabilities

| Flag       | Meaning                                     |
|------------|---------------------------------------------|
| 0x01       | Amplitude modulation (incl. CW)             |
| 0x02       | Frequency modulation (incl. M17 and AFSK)   |
| 0x04       | Single sideband (incl. FreeDV)              |
| 0x08       | Phase shift keying (incl. pi/4-DQPSK)       |
| 0x10..0x40 | Reserved                                    |
| 0x80       | Simplex=0/duplex=1                          |

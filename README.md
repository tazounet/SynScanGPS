# SynScanGPS

Emulate a SynScan GPS using an Arduino (or a Raspberry Pi Pico, see "pico" branch) and a cheap GPS

# Hardware
- Arduino Mini
- 12v -> 5v convertissor
- RS232/TTL adaptor
- RJ cable to the SynScan (12v & RS232)
- A GPS receiver (configured at 4800 bauds, the code is not stable for faster speed, see issue #2)

- RP2040 DualCore Pico
- any new GPS receiver. 115200Bd welcome

The RP2040dualCore branch use both CPU core. Each one is dedicated for it's own serial port. There is no timeout and collision problem.
This version support latest SynScan Hand controller 6.4.01. Code use new function which request pure NMEA message.


![Protocol](img/protocol.jpg) ![Build](img/arduino_gps.jpg)

Thanks to keymlinux for the latitude/longitude encoding

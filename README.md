## Sources for my Arduino ISP compatible programmer based on IskraMini board (arduino-mini clone) and Prolific PL2303 UART-to-USB converter

### Directory layout

#### ArduinoISP_mod
ArduinoISP sketch taken from Arduino IDE 1.8.5 example, with some modifications:
 - removed use of hardware SPI (because of soldering to the wrong pins, and my laziness to correct this mistake)
 - removed BitBangedSPI logic, added custom software SPI implementation instead (much faster than BitBangedSPI)
 - SCLK pulse-times can be configured at compile time
 - use 115200 baudrate for serial connection
 - some other minor modifications

TODO: make SCLK timing parameters configurable at runtime (partially done)

#### Case
Device case drawings

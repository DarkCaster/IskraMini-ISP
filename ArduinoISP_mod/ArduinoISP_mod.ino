// ArduinoISP (modification for my custom IskraMini board's layout)

// The 2-Clause BSD License

// Copyright (c) 2008-2011 Randall Bohn
// Copyright (c) 2018 DarkCaster

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "Arduino.h"
#include "MicroSPI.h"

#undef SERIAL

#ifdef ARDUINO_HOODLOADER2
#error "running this sketch on ATmega16U2 serial converter chip is not supported for now"
#endif

// Pins configuration:

#define RESET     10
#define LED_HB    3
#define LED_ERR   4
#define LED_PMODE 5
#define PIN_MOSI  7
#define PIN_MISO  8
#define PIN_SCK   9

// Configure the serial port to use.
//
// Prefer the USB virtual serial port (aka. native USB port), if the Arduino has one:
//   - it does not autoreset (except for the magic baud rate of 1200).
//   - it is more reliable because of USB handshaking.
//
// Leonardo and similar have an USB virtual serial port: 'Serial'.
// Due and Zero have an USB virtual serial port: 'SerialUSB'.
//
// On the Due and Zero, 'Serial' can be used too, provided you disable autoreset.
// To use 'Serial': #define SERIAL Serial

#ifdef SERIAL_PORT_USBVIRTUAL
#define SERIAL SERIAL_PORT_USBVIRTUAL
#else
#define SERIAL Serial
#endif

// Configure the baud rate:

//#define BAUDRATE	19200 //Arduino as ISP use this by default
#define BAUDRATE	115200 //AVR ISP use this by default
// #define BAUDRATE	1000000

#define HWVER 2
#define SWMAJ 1
#define SWMIN 18

// additional delay (in ms) when trying to read page right after writing
// I need this hack for some of my 3.3v arduino pro mini clones to fix verification error on some tiny sketches (like blink example).
#define WRITE_READ_DELAY 100

// STK Definitions
static constexpr uint8_t operator "" _u8 (unsigned long long arg) noexcept
{
  return static_cast<uint8_t>(arg);
}

#define STK_OK 0x10_u8
#define STK_FAILED 0x11_u8
#define STK_UNKNOWN 0x12_u8
#define STK_INSYNC 0x14_u8
#define STK_NOSYNC 0x15_u8
#define CRC_EOP 0x20_u8

// SCLK pulse widths for SPI tranfser to target

// for 4 Mhz and faster target AVR controller: 1 usec == 4 clocks on 4 Mhz (>2 clocks) for LOW and HIGH SCLK states
#define HSCLK_uSec 1
#define LSCLK_uSec 1

// for 1 MHz target AVR controller: 3 usec == 3 clocks (>2 clocks) for LOW and HIGH SCLK states
// #define HSCLK_uSec 3
// #define LSCLK_uSec 3

// for 128 KHz target AVR controller: 24 usec ~= 3 clocks (>2 clocks) for LOW and HIGH SCLK states
// #define HSCLK_uSec 24
// #define LSCLK_uSec 24

static MicroSPI ISP(PIN_MOSI, PIN_MISO, PIN_SCK, HSCLK_uSec, LSCLK_uSec);

#define PTIME 200_u8
void pulse(uint8_t pin) {
  digitalWrite(pin, HIGH);
  delay(PTIME);
  digitalWrite(pin, LOW);
}

void setup() {
  SERIAL.begin(BAUDRATE);
  pinMode(LED_HB, OUTPUT);
  pulse(LED_HB);
  pinMode(LED_ERR, OUTPUT);
  pulse(LED_ERR);
  pinMode(LED_PMODE, OUTPUT);
  pulse(LED_PMODE);
}

static uint8_t readDelayNeeded = 0;
static uint8_t status = 0; //used to detect error status change after avrisp method execution
static uint8_t pmode = 0;
#define STATUS_SHIFT() ({ status=(status<<1)|(status&0x1); })
#define SET_ERROR() ({ status|=0x1; })
#define UNSET_ERROR() ({ status&=0x2; })
#define GET_ERROR_STATUS() ( status&0x3 )

// address for reading and writing, set by 'U' command
static unsigned int here;
static uint8_t buff[256]; // global block storage

#define beget16(addr) (*addr * 256 + *(addr+1) )
typedef struct param {
  uint8_t devicecode;
  uint8_t revision;
  uint8_t progtype;
  uint8_t parmode;
  uint8_t polling;
  uint8_t selftimed;
  uint8_t lockbytes;
  uint8_t fusebytes;
  uint8_t flashpoll;
  uint16_t eeprompoll;
  uint16_t pagesize;
  uint16_t eepromsize;
  uint32_t flashsize;
}
parameter;

parameter param;

// this provides a heartbeat on LED_HB, so you can tell the software is running.
static uint8_t hbval = 128;
static int8_t hbdelta = 8;
void heartbeat() {
  static unsigned long last_time = 0;
  unsigned long now = millis();
  if ((now - last_time) < 40)
    return;
  last_time = now;
  if (hbval > 192) hbdelta = -hbdelta;
  if (hbval < 32) hbdelta = -hbdelta;
  hbval += hbdelta;
  analogWrite(LED_HB, hbval);
}

static bool rst_active_high;

void reset_target(bool reset) {
  digitalWrite(RESET, ((reset && rst_active_high) || (!reset && !rst_active_high)) ? HIGH : LOW);
}

uint8_t getch()
{
  int data=-1;
  while ((data=SERIAL.read())<0);
  return data;
}

void fill(int n) {
  for (int x = 0; x < n; x++) {
    buff[x] = getch();
  }
}

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  ISP.TransferByte(a);
  ISP.TransferByte(b);
  ISP.TransferByte(c);
  return ISP.TransferByte(d);
}

void empty_reply() {
  if (CRC_EOP == getch()) {
    SERIAL.write(STK_INSYNC);
    SERIAL.write(STK_OK);
  } else {
    SET_ERROR();
    SERIAL.write(STK_NOSYNC);
  }
}

void breply(uint8_t b) {
  if (CRC_EOP == getch()) {
    SERIAL.write(STK_INSYNC);
    SERIAL.write(b);
    SERIAL.write(STK_OK);
  } else {
    SET_ERROR();
    SERIAL.write(STK_NOSYNC);
  }
}

void get_version(uint8_t c) {
  switch (c) {
    case 0x80:
      breply(HWVER);
      break;
    case 0x81:
      breply(SWMAJ);
      break;
    case 0x82:
      breply(SWMIN);
      break;
    case 0x93:
      breply('S'); // serial programmer
      break;
    default:
      breply(0);
  }
}

void set_parameters() {
  // call this after reading parameter packet into buff[]
  param.devicecode = buff[0];
  param.revision   = buff[1];
  param.progtype   = buff[2];
  param.parmode    = buff[3];
  param.polling    = buff[4];
  param.selftimed  = buff[5];
  param.lockbytes  = buff[6];
  param.fusebytes  = buff[7];
  param.flashpoll  = buff[8];
  // ignore buff[9] (= buff[8])
  // following are 16 bits (big endian)
  param.eeprompoll = beget16(&buff[10]);
  param.pagesize   = beget16(&buff[12]);
  param.eepromsize = beget16(&buff[14]);

  // 32 bits flashsize (big endian)
  param.flashsize = buff[16] * 0x01000000
                    + buff[17] * 0x00010000
                    + buff[18] * 0x00000100
                    + buff[19];

  // AVR devices have active low reset, AT89Sx are active high
  rst_active_high = (param.devicecode >= 0xe0);
}

void start_pmode() {

  // Reset target before driving PIN_SCK or PIN_MOSI

  // SPI.begin() will configure SS as output, so SPI master mode is selected.
  // We have defined RESET as pin 10, which for many Arduinos is not the SS pin.
  // So we have to configure RESET as output here,
  // (reset_target() first sets the correct level)
  reset_target(true);
  pinMode(RESET, OUTPUT);
  ISP.ResetPins();

  // See AVR datasheets, chapter "SERIAL_PRG Programming Algorithm":

  // Pulse RESET after PIN_SCK is low (ISP.ResetPins call above will do it):
  delay(20); // discharge PIN_SCK, value arbitrarily chosen
  reset_target(false);

  // Pulse must be minimum 2 target CPU clock cycles so 100 usec is ok for CPU
  // speeds above 20 KHz
  delayMicroseconds(100);
  reset_target(true);

  // Send the enable programming command:
  delay(50); // datasheet: must be > 20 msec
  spi_transaction(0xAC, 0x53, 0x00, 0x00);
}

void end_pmode() {
  // We're about to take the target out of reset so configure SPI pins as input
  pinMode(PIN_MOSI, INPUT);
  pinMode(PIN_SCK, INPUT);
  reset_target(false);
  pinMode(RESET, INPUT);
}

void universal() {
  uint8_t ch;

  fill(4);
  ch = spi_transaction(buff[0], buff[1], buff[2], buff[3]);
  breply(ch);
}

void flash(uint8_t hilo, unsigned int addr, uint8_t data) {
  spi_transaction(0x40 + 8 * hilo,
                  addr >> 8 & 0xFF,
                  addr & 0xFF,
                  data);
}
void commit(unsigned int addr) {
  spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
}

unsigned int current_page() {
  if (param.pagesize == 32) {
    return here & 0xFFFFFFF0;
  }
  if (param.pagesize == 64) {
    return here & 0xFFFFFFE0;
  }
  if (param.pagesize == 128) {
    return here & 0xFFFFFFC0;
  }
  if (param.pagesize == 256) {
    return here & 0xFFFFFF80;
  }
  return here;
}


void write_flash(int length) {
  fill(length);
  if (CRC_EOP == getch()) {
    SERIAL.write(STK_INSYNC);
    SERIAL.write(write_flash_pages(length));
  } else {
    SET_ERROR();
    SERIAL.write(STK_NOSYNC);
  }
}

uint8_t write_flash_pages(int length) {
  int x = 0;
  unsigned int page = current_page();
  while (x < length) {
    if (page != current_page()) {
      commit(page);
      page = current_page();
    }
    flash(LOW, here, buff[x++]);
    flash(HIGH, here, buff[x++]);
    here++;
  }

  commit(page);

  return STK_OK;
}

#define EECHUNK (32)
uint8_t write_eeprom(unsigned int length) {
  // here is a word address, get the byte address
  unsigned int start = here * 2;
  unsigned int remaining = length;
  if (length > param.eepromsize) {
    SET_ERROR();
    return STK_FAILED;
  }
  while (remaining > EECHUNK) {
    write_eeprom_chunk(start, EECHUNK);
    start += EECHUNK;
    remaining -= EECHUNK;
  }
  write_eeprom_chunk(start, remaining);
  return STK_OK;
}
// write (length) bytes, (start) is a byte address
uint8_t write_eeprom_chunk(unsigned int start, unsigned int length) {
  // this writes byte-by-byte, page writing may be faster (4 bytes at a time)
  fill(length);
  for (unsigned int x = 0; x < length; x++) {
    unsigned int addr = start + x;
    spi_transaction(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, buff[x]);
    delay(45);
  }
  return STK_OK;
}

void program_page() {
  uint8_t result = STK_FAILED;
  unsigned int length = 256 * getch();
  length += getch();
  char memtype = getch();
  // flash memory @here, (length) bytes
  if (memtype == 'F') {
    write_flash(length);
    return;
  }
  if (memtype == 'E') {
    result = (uint8_t)write_eeprom(length);
    if (CRC_EOP == getch()) {
      SERIAL.write(STK_INSYNC);
      SERIAL.write(result);
    } else {
      SET_ERROR();
      SERIAL.write(STK_NOSYNC);
    }
    return;
  }
  SERIAL.write(STK_FAILED);
  return;
}

uint8_t flash_read(uint8_t hilo, unsigned int addr) {
  return spi_transaction(0x20 + hilo * 8,
                         (addr >> 8) & 0xFF,
                         addr & 0xFF,
                         0);
}

char flash_read_page(int length) {
  for (int x = 0; x < length; x += 2) {
    uint8_t low = flash_read(LOW, here);
    SERIAL.write(low);
    uint8_t high = flash_read(HIGH, here);
    SERIAL.write(high);
    here++;
  }
  return STK_OK;
}

char eeprom_read_page(int length) {
  // here again we have a word address
  int start = here * 2;
  for (int x = 0; x < length; x++) {
    int addr = start + x;
    uint8_t ee = spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
    SERIAL.write(ee);
  }
  return STK_OK;
}

void read_page() {
  uint8_t result = STK_FAILED;
  int length = 256 * getch();
  length += getch();
  char memtype = getch();
  if (CRC_EOP != getch()) {
    SET_ERROR();
    SERIAL.write(STK_NOSYNC);
    return;
  }
  SERIAL.write(STK_INSYNC);
  if (memtype == 'F') result = flash_read_page(length);
  if (memtype == 'E') result = eeprom_read_page(length);
  SERIAL.write(result);
}

void read_signature() {
  if (CRC_EOP != getch()) {
    SET_ERROR();
    SERIAL.write(STK_NOSYNC);
    return;
  }
  SERIAL.write(STK_INSYNC);
  uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);
  SERIAL.write(high);
  uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
  SERIAL.write(middle);
  uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);
  SERIAL.write(low);
  SERIAL.write(STK_OK);
}

void loop(void)
{
  // light the heartbeat LED
  heartbeat();
  int avr_ch=-1;
  if((avr_ch=SERIAL.read())<0)
    return;
  uint8_t ch=avr_ch;
  // store previous status values
  STATUS_SHIFT();
  switch (ch)
  {
    case '0': // signon
      UNSET_ERROR();
      empty_reply();
      break;
    case '1':
      if (getch() == CRC_EOP)
      {
        SERIAL.write(STK_INSYNC);
        SERIAL.print("AVR ISP");
        SERIAL.write(STK_OK);
      }
      else
      {
        SET_ERROR();
        SERIAL.write(STK_NOSYNC);
      }
      break;
    case 'A':
      get_version(getch());
      break;
    case 'B':
      fill(20);
      set_parameters();
      empty_reply();
      break;
    case 'E': // extended parameters - ignore for now
      fill(5);
      empty_reply();
      break;
    case 'P':
      if (!pmode)
      {
        start_pmode();
        pmode=1;
        digitalWrite(LED_PMODE, HIGH);
      }
      empty_reply();
      break;
    case 'U': // set address (word)
      here = getch();
      here += 256 * getch();
      empty_reply();
      break;
    case 0x60: //STK_PROG_FLASH
      getch(); // low addr
      getch(); // high addr
      empty_reply();
      break;
    case 0x61: //STK_PROG_DATA
      getch(); // data
      empty_reply();
      break;
    case 0x64: //STK_PROG_PAGE
      program_page();
      readDelayNeeded=1;
      break;
    case 0x74: //STK_READ_PAGE 't'
      if(readDelayNeeded)
      {
        readDelayNeeded=0;
        delay(WRITE_READ_DELAY);
      }
      read_page();
      break;
    case 'V': //0x56
      universal();
      break;
    case 'Q': //0x51
      UNSET_ERROR();
      end_pmode();
      pmode=0;
      digitalWrite(LED_PMODE, LOW);
      empty_reply();
      break;
    case 0x75: //STK_READ_SIGN 'u'
      read_signature();
      break;
    // expecting a command, not CRC_EOP
    // this is how we can get back in sync
    case CRC_EOP:
      SET_ERROR();
      SERIAL.write(STK_NOSYNC);
      break;
    // anything else we will return STK_UNKNOWN
    default:
      SET_ERROR();
      if (CRC_EOP == getch())
        SERIAL.write(STK_UNKNOWN);
      else
        SERIAL.write(STK_NOSYNC);
  }
  //light-up or shut-down some LEDS depending on status change
  uint8_t stat=GET_ERROR_STATUS();
  //error variable as changed from 0 to 1
  if(stat==0x1)
    digitalWrite(LED_ERR, HIGH);
  //error variable as changed from >0 to 0
  else if(stat==0x2)
    digitalWrite(LED_ERR, LOW);
}

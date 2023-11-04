#include <avr/delay.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <Arduino.h>
#include <avr/io.h>

#include "lis3dh_reg.h"
#include "fxpt_atan2.h"

#define CLOCK_PIN PB0
#define DATA_PIN PB1

#define DATA_PORT PORTB

#define INPUT_PORT PINB
#define DATA_PORT_DDR DDRB

#define CS_PORT_DDR DDRB
#define CS_PORT PORTB
#define CS_PIN PB2

#define READ_BIT 0x80
#define AUTO_INCREMENT_BIT 0x40


typedef enum {
  LIS3DH_DATARATE_400_HZ = 0b0111,  //  400Hz
  LIS3DH_DATARATE_200_HZ = 0b0110,  //  200Hz
  LIS3DH_DATARATE_100_HZ = 0b0101,  //  100Hz
  LIS3DH_DATARATE_50_HZ = 0b0100,   //   50Hz
  LIS3DH_DATARATE_25_HZ = 0b0011,   //   25Hz
  LIS3DH_DATARATE_10_HZ = 0b0010,   // 10 Hz
  LIS3DH_DATARATE_1_HZ = 0b0001,    // 1 Hz
  LIS3DH_DATARATE_POWERDOWN = 0,
  LIS3DH_DATARATE_LOWPOWER_1K6HZ = 0b1000,
  LIS3DH_DATARATE_LOWPOWER_5KHZ = 0b1001,

} lis3dh_dataRate_t;



#define P0 PORTA0  // PA7
#define P1 PORTA1  // PA6
#define P2 PORTA2  // PA5
#define P3 PORTA3  // PA4
#define P4 PORTA4  // physical 2, PA3
#define P5 PORTA5  // physical 3, PA2
#define P6 PORTA6  // physical 4, PA1
#define P7 PORTA7  // physical 5, PA0


#define NUM_LEDS 39


// note: I have this flipped around vs. other boards
// since my naming convention in the hardware is D{cathode, anode}
typedef struct {
  const uint8_t cathode : 4;
  const uint8_t anode : 4;
} led_t;


const uint8_t NUM_SECTIONS = 10;
const uint8_t MAX_LEDS_PER_SECTION = 5;

const uint8_t SECTIONS[NUM_SECTIONS + 1] = {
  0,
  3,
  7,
  12,
  16,
  20,
  25,
  28,
  32,
  35,
  40
};

const led_t LED[NUM_LEDS] = {
  // top
  { P0, P5 },
  { P2, P1 },
  { P7, P5 },

  { P2, P3 },
  { P5, P4 },
  { P5, P3 },
  { P7, P3 },

  { P1, P3 },
  { P3, P4 },
  { P4, P3 },
  { P3, P0 },
  { P0, P2 },

  { P1, P0 },
  { P7, P0 },
  { P5, P0 },
  { P5, P7 },

  { P1, P2 },
  { P0, P6 },
  { P6, P5 },
  { P3, P7 },

  { P4, P2 },
  { P2, P4 },
  { P4, P6 },
  { P6, P4 },
  { P5, P6 },

  { P0, P3 },
  { P3, P2 },
  { P2, P0 },

  { P4, P5 },
  { P3, P5 },
  { P3, P1 },
  { P0, P4 },

  { P1, P4 },
  { P4, P1 },
  { P4, P0 },

  { P0, P7 },
  { P7, P6 },
  { P6, P7 },
  { P6, P0 },
};


void resetPix() {
  PORTA = 0;
  DDRA = 0;
}

void pix(uint8_t sinkPin, uint8_t sourcePin) {
  PORTA = 0;
  DDRA = (1 << sinkPin) | (1 << sourcePin);
  PORTA = (1 << sourcePin);
}

void pix(int8_t pinIx) {
  if (pinIx < 0 || pinIx > NUM_LEDS) {
    resetPix();
  } else {
    pix(LED[pinIx].cathode, LED[pinIx].anode);
  }
}



volatile boolean gotInterrupt = false;

void setup(void) {
  uint8_t reset_reason = MCUSR;  // could check this for brown-out or watchdog reset
  MCUSR = 0;

  wdt_disable();

  DDRA = 0;
  PORTA = 0;

  bitClear(ADCSRA, ADEN);  // disable ADC
  bitSet(ACSR, ACD);       // disable analog comparator

  // disable timer1, timer0, USI, shut down ADC
  PRR |= bit(PRTIM1) | bit(PRTIM0) | bit(PRUSI) | bit(PRADC);

  DIDR0 = 0xff;  // disable all digital inputs


  DATA_PORT_DDR |= bit(DATA_PIN) | bit(CLOCK_PIN);
  bitSet(CS_PORT, CS_PIN);
  bitClear(DATA_PORT, CLOCK_PIN);
  bitSet(CS_PORT_DDR, CS_PIN);

  startWatchdog();

  _delay_ms(10);  // app note says lis takes 5ms to boot
  if (!setupLis(true, false, false, true)) {
    // I have no idea what state the CS pin should be in this case
    // set it to an input. should be high-Z
    bitClear(CS_PORT_DDR, CS_PIN);
    while (true) {
      pix(0);
      _delay_ms(1);
      resetPix();
      _delay_ms(5);
      wdt_reset();
    }
  }
  _delay_ms(10);  // various mode changes take a second to transition
  wdt_reset();
}

void startWatchdog() {
  const uint8_t WATCHDOG_TIME = WDTO_1S;
  wdt_reset();
  wdt_enable(WATCHDOG_TIME);
  wdt_reset();
}



const uint8_t CUBIC_LUT_5_bit[] = { 0, 4, 8, 12, 16, 20, 24, 28, 32, 44, 56, 68, 80, 92, 104, 116, 128, 140, 152, 164, 176, 188, 200, 212, 224, 228, 232, 236, 240, 244, 248, 252 };

const int16_t CENTER_DISTANCES[] = { 0, 4690, 9012, 10683, 8180, 8474, 12077, 17020, 19657, 20330, 18061, 13793, 16694, 15183, 13563, 10344, 23308, 22568, 21231, 16591, 26321, 29240, 28651, 25911, 22642, 25516, 26203, 20620, 30171, 30296, 26999, 24075, 31228, 32767, 26543, 22155, 27079, 30138, 29522 };

void loop4() {

  // increment section
  // for each led in the section
  // light for a period of time


  const uint8_t max_lit_expected = 18;

  for (int16_t i = 0; i < 0x7fff >> 3; ++i) {

    // optimization:
    // could keep the leds sorted by distance
    // and just look for ones in a rolling window

    uint16_t time_spent = 0;
    for (uint8_t led = 0; led < NUM_LEDS; ++led) {
      int16_t distance = abs((i << 3) - CENTER_DISTANCES[led]);
      if (distance < 4096) {
        pix(led);
        uint16_t time_to_spend = (4096 >> 7) - (distance >> 7);
        delayMicroseconds(time_to_spend);
        time_spent += time_to_spend;
        resetPix();
      }
    }

    // need to waste some time if there are too few leds lit at a time
    // calculated offline as the max total spent
    delayMicroseconds(277 - time_spent);

    // there's a bug, some of the ones on the outer edge at lighting first

    wdt_reset();
  }

  wdt_reset();
  goToSleep();
}



void loop() {

  // increment section
  // for each led in the section
  // light for a period of time

  const uint8_t FRAME_LIMIT = 186;

  for (uint8_t section = 0; section < NUM_SECTIONS; ++section) {
    for (uint8_t j = 0; j < FRAME_LIMIT; ++j) {
      for (uint8_t led_in_section = 0; led_in_section < MAX_LEDS_PER_SECTION; ++led_in_section) {
        if (SECTIONS[section] + led_in_section < SECTIONS[section + 1]) {
          pix(SECTIONS[section] + led_in_section);
        }
        // start 100% on, fade out
        // delayMicroseconds((FRAME_LIMIT >> 1) - (j >> 1));
        // delayMicroseconds(CUBIC_LUT_5_bit[31 - (j >> 3)] >> 1);
        delayMicroseconds(FRAME_LIMIT - j);

        resetPix();

        if (section < NUM_SECTIONS - 1) {
          if (SECTIONS[section + 1] + led_in_section < SECTIONS[section + 2]) {
            pix(SECTIONS[section + 1] + led_in_section);
          }
        }
        // start 100% off, fade on
        // delayMicroseconds(j >> 1);
        delayMicroseconds(j);
        // delayMicroseconds(CUBIC_LUT_5_bit[j >> 3] >> 1);
        resetPix();
      }
    }
    wdt_reset();
  }
  // }

  wdt_reset();
  goToSleep();
}


void goToSleep(void) {

  // it's supposed to be possible to get the accelerometer to go into high/low datarate based on activity automatically
  // return to sleep, sleep to wake

  // decrease ODR
  writeOut(LIS3DH_CTRL_REG1, lis3dh_reg_t{ .ctrl_reg1 = {
                                             .xen = 1,
                                             .yen = 1,
                                             .zen = 1,
                                             .lpen = 1,                    // hr mode cannot be enabled while this is true
                                             .odr = LIS3DH_DATARATE_10_HZ  // 4 bits
                                           } });

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  sleep_enable();
  MCUCR &= ~(_BV(ISC01) | _BV(ISC00));  //INT0 on low level

  bitClear(CS_PORT, CS_PIN);      // probably not necessary
  bitClear(CS_PORT_DDR, CS_PIN);  // change to INPUT

  PORTA = 0;  // shouldn't be necessary

  cli();

  wdt_disable();

  //  do not allow sleep while the pin is still active, can cause freezing
  while (!bitRead(PINB, PB2)) {
    void;
  }

  GIMSK |= _BV(INT0);  //enable INT0

  //stop interrupts to ensure the BOD timed sequence executes as required
  byte mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);  //turn off the brown-out detector
  byte mcucr2 = mcucr1 & ~_BV(BODSE);
  MCUCR = mcucr1;
  MCUCR = mcucr2;

  sei();            //ensure interrupts enabled so we can wake up again
  sleep_cpu();      //go to sleep
  sleep_disable();  //wake up here

  startWatchdog();

  bitSet(CS_PORT_DDR, CS_PIN);  // change to OUTPUT

  _delay_us(10);  // wait for pin to settle

  // increase ODR
  writeOut(LIS3DH_CTRL_REG1, lis3dh_reg_t{ .ctrl_reg1 = {
                                             .xen = 1,
                                             .yen = 1,
                                             .zen = 1,
                                             .lpen = 1,                     // hr mode cannot be enabled while this is true
                                             .odr = LIS3DH_DATARATE_200_HZ  // 4 bits
                                           } });

  _delay_ms(10);  // I have no idea whether the start-up time is based on the old ODR or the new ODR
}

//external interrupt 0 wakes the MCU
ISR(INT0_vect) {
  GIMSK = 0;  //disable external interrupts (only need one to wake up)
}

uint8_t offsetLed(uint8_t start, int8_t offset) {
  // offset must be < NUM_LEDS
  // other conditions could be handled with recursion
  int8_t i = start;
  i += offset;
  if (i < 0) {
    return NUM_LEDS + i;
  } else if (i >= NUM_LEDS) {
    return i - NUM_LEDS;
  } else {
    return i;
  }
}


void smoothInt(uint16_t sample, uint8_t bits, long *filter) {
  long local_sample = ((long)sample) << 16;

  *filter += (local_sample - *filter) >> bits;
}


short getFilterValue(long filter) {
  return (short)((filter + 0x8000) >> 16);
}

long setFilterValue(short value) {
  return ((long)value) << 16;
}


int16_t repeatedRead(uint8_t addr) {
  const uint8_t size_of_fifo_queue = 32;
  int32_t buf = 0;
  for (uint8_t i = 0; i < size_of_fifo_queue; ++i) {
    buf += readIn2(addr);
  }
  return buf >> 9;  // 5 + 4 (5 = divide by 32, 4 = padding we naturally get from the sensor)
}

int16_t repeatedReadSmooth(uint8_t addr) {
  // if I'm doing it this way I could try to get the # of readings actually in the queue
  const uint8_t size_of_fifo_queue = 32;
  int32_t filter = 0;  // setFilterValue(readIn2(addr) >> 4);
  for (uint8_t i = size_of_fifo_queue; i > 0; ++i) {
    smoothInt(readIn2(addr) >> 4, (i >> 2) + 1, &filter);
  }
  return getFilterValue(filter) >> 5;
}


int16_t repeatedRead2(uint8_t addr) {
  const uint8_t size_of_fifo_queue = 32;
  int32_t buf = 0;
  for (uint8_t i = 0; i < size_of_fifo_queue; ++i) {
    buf += readIn2(addr);
  }
  return buf >> 9;  // 5 + 4 (5 = divide by 32, 4 = padding we naturally get from the sensor)
}



void rawWrite(uint8_t val) {
  //  bitSet(DDRB, DATA_PIN); // enable output

  for (int8_t i = 0; i < 8; i++) {
    bitWrite(DATA_PORT, DATA_PIN, bitRead(val, 7 - i));

    _delay_us(1);
    bitSet(DATA_PORT, CLOCK_PIN);
    _delay_us(1);

    bitClear(DATA_PORT, CLOCK_PIN);
  }
}

uint8_t rawRead() {
  uint8_t readVal = 0;

  // clock is expected to be low at this point

  for (int8_t i = 0; i < 8; ++i) {

    bitSet(DATA_PORT, CLOCK_PIN);  // may be high already from write operation
    _delay_us(1);

    bitWrite(readVal, 7 - i, bitRead(INPUT_PORT, DATA_PIN));

    bitClear(DATA_PORT, CLOCK_PIN);
    _delay_us(1);
  }

  return readVal;
}

void startTransaction() {
  bitClear(CS_PORT, CS_PIN);
}

void endTransaction() {
  bitSet(CS_PORT, CS_PIN);
}

// test the wake/sleep interrupt
//   could reconfigure the LIS on wake. only necessary if reconfigurating the HPF.
//   otherwise, just need to reconfigure the INT/CS pin on wake/sleep

inline void writeOut(uint8_t addr, lis3dh_reg_t val) {
  writeOut(addr, val.byte);
}

void writeOut(uint8_t addr, uint8_t val) {
  // should start/end transaction around this
  startTransaction();

  rawWrite(addr);
  rawWrite(val);

  endTransaction();
}

void startRead(uint8_t addr) {

  startTransaction();

  _delay_us(1);

  rawWrite(addr);

  bitClear(DATA_PORT, DATA_PIN);
  bitClear(DATA_PORT_DDR, DATA_PIN);  // enable input

  _delay_us(1);
}

void endRead() {
  endTransaction();

  // reset state of other pins
  bitSet(DATA_PORT_DDR, DATA_PIN);
}

uint8_t readIn(uint8_t addr) {

  startRead(addr          // should only use bottom 6 bits
            | READ_BIT);  // read bit

  uint8_t result = rawRead();

  endRead();

  return result;
}

int16_t readIn2(uint8_t addr) {

  startRead(addr  // should only use bottom 6 bits
            | READ_BIT
            | AUTO_INCREMENT_BIT);  // read bit

  int16_t output = (int16_t)rawRead() | int16_t(rawRead() << 8);

  endRead();

  return output;
}



uint8_t setupLis(boolean setupInterrupts, boolean enableHpf, boolean enableLatch, boolean enableFifo) {

  const boolean USE_CLICK_INT = false;
  const boolean USE_ACCEL_INT = true;

  //  writeOut(CTRL_REG4,
  //           (SENSITIVITY_2G << 4));
  //           | 1); // 3 wire mode

  // startup sequence, per the app note
  //1. Write CTRL_REG1
  //2. Write CTRL_REG2
  //3. Write CTRL_REG3
  //4. Write CTRL_REG4
  //5. Write CTRL_REG5
  //6. Write CTRL_REG6
  //7. Write REFERENCE
  //8. Write INTx_THS
  //9. Write INTx_DUR
  //10. Write INTx_CFG
  //11. Write CTRL_REG5


  // what next?
  // test an interrupt, maybe click
  // then try wake interrupt (need to enable hpf for sleep, disable on startup/wake)

  // how to set up FIFO:
  // per app note an5005 sec. 9.3.3
  // set FIFO_EN bit in REG3
  // set FM1 and FM0 bits in FIFO_CTRL_REG (streaming mode)

  // how to read:
  // read FSS bits (0-4) in FIFO_SRC_REG to get size of queue
  // read X, Y, Z that many times

  writeOut(LIS3DH_CTRL_REG1, lis3dh_reg_t{ .ctrl_reg1 = {
                                             .xen = 1,
                                             .yen = 1,
                                             .zen = 1,
                                             .lpen = 1,                     // hr mode cannot be enabled while this is true
                                             .odr = LIS3DH_DATARATE_200_HZ  // 4 bits
                                           } });

  writeOut(LIS3DH_CTRL_REG2, lis3dh_reg_t{ .ctrl_reg2 = {
                                             // high-pass filter config
                                             .hp = 0b111,                 // HPCLICK + HP_IA2 + HP_IA1 -> HP; // use HPF for interrupts
                                             .fds = (enableHpf) ? 1 : 0,  // enable hp filter on regular output
                                             .hpcf = 0,                   // 2 bits
                                             .hpm = 0                     // 2 bits, filter mode = normal
                                           } });

  writeOut(LIS3DH_CTRL_REG3, lis3dh_reg_t{ .ctrl_reg3 = {
                                             // various interrupt config
                                             .not_used_01 = 0,
                                             .i1_overrun = 0,
                                             .i1_wtm = 0,
                                             .i1_321da = 0,  // .i1_321da = 0, // change from lis3 "321DA interrupt on INT1."
                                             .i1_zyxda = 0,
                                             .i1_ia2 = 0,
                                             .i1_ia1 = (setupInterrupts && USE_ACCEL_INT) ? 1 : 0,
                                             .i1_click = (setupInterrupts && USE_CLICK_INT) ? 1 : 0,
                                           } });

  writeOut(LIS3DH_CTRL_REG4, lis3dh_reg_t{ .ctrl_reg4 = {
                                             .sim = 1,  // 3 wire mode
                                             .st = 0,   // self test mode
                                             .hr = 0,   // high resolution mode (12 bit output). 0 must be set in reg1.lpen. see section 3.2.1 in data sheet.
                                             .fs = 0,   // data range. 00 = 2g, 01 = 4g, 10 = 8g, 11 = 16g
                                             .ble = 0,  // big endian vs little endian. only matters for high resolution mode.
                                             .bdu = 1,  // block data ready. don't allow mis-matched pairs of high and low bytes
                                           } });

  writeOut(LIS3DH_CTRL_REG5, lis3dh_reg_t{ .ctrl_reg5 = { // fifo and interrupt latch
                                                          .d4d_int2 = 0,
                                                          .lir_int2 = 0,
                                                          .d4d_int1 = 0,
                                                          .lir_int1 = (enableLatch) ? 1 : 0,  // latch. if set, must read LIS3DH_INT1_SRC to clear. might be useful if we want to see how big the initial impact was.
                                                          .not_used_01 = 0,
                                                          .fifo_en = (enableFifo) ? 1 : 0,
                                                          .boot = 0 } });

  writeOut(LIS3DH_CTRL_REG6, lis3dh_reg_t{ .ctrl_reg6 = {
                                             .not_used_01 = 0,
                                             .int_polarity = 1,  // set int active low
                                             .not_used_02 = 0,
                                             .i2_act = 0,
                                             .i2_boot = 0,
                                             .i2_ia2 = 0,
                                             .i2_ia1 = 0,
                                             .i2_click = 0 } });


  readIn(LIS3DH_REFERENCE);  // resets HPF calibration (doesn't seem to do much)

  if (enableFifo) {
    writeOut(LIS3DH_FIFO_CTRL_REG, lis3dh_reg_t{ .fifo_ctrl_reg = {
                                                   .fth = 0,
                                                   .tr = 0,
                                                   .fm = 0b10,  // b00 = bypass, b01 = fifo mode, b10 = stream mode, b11 = stream-to-fifo mode
                                                 } });
  }

  if (setupInterrupts) {
    if (USE_ACCEL_INT) {                  // non-click interrupt
      writeOut(LIS3DH_INT1_THS, 10);      // copying what I did for skully
      writeOut(LIS3DH_INT1_DURATION, 0);  // probably unnecessary

      writeOut(LIS3DH_INT1_CFG, lis3dh_reg_t{ .int1_cfg = {
                                                .xlie = 0,
                                                .xhie = 1,
                                                .ylie = 0,
                                                .yhie = 1,
                                                .zlie = 0,
                                                .zhie = 1,
                                                ._6d = 0,
                                                .aoi = 0,
                                              } });
    }
    if (USE_CLICK_INT) {
      writeOut(LIS3DH_CLICK_CFG, lis3dh_reg_t{ .click_cfg = {
                                                 .xs = 0,
                                                 .xd = 0,
                                                 .ys = 0,
                                                 .yd = 0,
                                                 .zs = 1,
                                                 .zd = 1,
                                                 .not_used_01 = 0,
                                               } });

      writeOut(LIS3DH_CLICK_THS, lis3dh_reg_t{ .click_ths = {
                                                 .ths = 01,  // 1 LSB = full scale/128.
                                                 .lir_click = 0,
                                               } });

      //1 LSB = 1/ODR.
      //TLI7 through TLI0 define the maximum time interval that can elapse between the start of
      //the click-detection procedure (the acceleration on the selected channel exceeds the
      //programmed threshold) and when the acceleration falls back below the threshold.
      writeOut(LIS3DH_TIME_LIMIT, lis3dh_reg_t{ .time_limit = {
                                                  .tli = 10,  // chosen b/c adafruit default
                                                  .not_used_01 = 0,
                                                } });

      writeOut(LIS3DH_TIME_LATENCY, lis3dh_reg_t{ .time_latency = {
                                                    .tla = 20,  // chosen b/c adafruit default
                                                  } });

      writeOut(LIS3DH_TIME_WINDOW, lis3dh_reg_t{ .time_window = {
                                                   .tw = 255,  // chosen b/c adafruit default
                                                 } });
    }


  } else {
    writeOut(LIS3DH_INT1_CFG, 0);
  }

  return readIn(LIS3DH_WHO_AM_I) == LIS3DH_ID;
}

// plan
// figure out "top"
// rotate 45 degrees
// use same animation as 2019, but use that to determine start & end

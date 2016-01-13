//***************************************************************************************
//  MSP430 + PCD8544-based LCD (like Nokia 5110)
// 
//  MSP430x2xx Family User's Guide      : http://www.ti.com/lit/ug/slau144j/slau144j.pdf
//  MSP430G2x53 Data Sheet              : http://www.ti.com/lit/ds/symlink/msp430g2553.pdf
//  PCD8544 Data Sheet (Nokia 5110 MCU) : https://www.sparkfun.com/datasheets/LCD/Monochrome/Nokia5110.pdf
// 
//  My setup:
// 
//         NOKIA 5110 LCD                                               MSP-EXP430G2
//       -----------------                                           -------------------
//      |              GND|<-- Ground ------------------------------|J6     GND         |
//      |               BL|<-- Back-light (not connected)
//      |              VCC|<-- Vcc +3..5V --------------------------|J1.1   VCC         |
//      |                 |
//      |              CLC|<-- Clock -------------------------------|J1.7   P1.5        |
//      |              DIN|<-- Data Input --------------------------|J2.15  P1.7        |
//      |               DC|<-- Data/Command (high/low) -------------|J1.3   P1.1        |
//      |               CE|<-- Chip Enable (active low) ------------|J1.2   P1.0        |
//      |              RST|<-- Reset -------------------------------|J2.16  RST
// 
//                                                Onboard button -->|       P1.3
// 
// 
//***************************************************************************************

#include <msp430g2553.h>
#include "PCD8544.h"
#include "dht22.h"

#define SET(reg, bits) (reg |= bits)
#define RST(reg, bits) (reg &= ~bits)

#define LCD5110_SCLK_PIN            BIT5
#define LCD5110_DN_PIN              BIT7
#define LCD5110_SCE_PIN             BIT0
#define LCD5110_DC_PIN              BIT1
#define LCD5110_SELECT              P1OUT &= ~LCD5110_SCE_PIN
#define LCD5110_DESELECT            P1OUT |= LCD5110_SCE_PIN
#define LCD5110_SET_COMMAND         P1OUT &= ~LCD5110_DC_PIN
#define LCD5110_SET_DATA            P1OUT |= LCD5110_DC_PIN
#define LCD5110_COMMAND             0
#define LCD5110_DATA                1

// LCD functions declaration

// Sets LCD address (x = 0..83, y = 0..5)
void setAddr(unsigned char xAddr, unsigned char yAddr);
void writeToLCD(unsigned char dataCommand, unsigned char data);
void writeCharToLCD(char c);
void writeStringToLCD(const char *string);
void initLCD();
void clearLCD();
void clearBank(unsigned char bank);

// Helper functions

char* ul2a(unsigned long);
char* ul2hex(unsigned long);
char* char2hex(unsigned char);

// DHT22 sensor related declarations

// Type of data the DHT sensor sends
typedef union DHT_DATA {
    struct {
        char hh;        // Humidity, high byte
        char hl;        // Humidity, low byte
        char th;        // Temperature, high byte
        char tl;        // Temperature, low byte
        char crc;       // Checksum
    } val;
    char bytes[5];      // All sensor data in raw
} DHT_DATA;

typedef struct DHT {
    int pin;            // MCU pin the sensor is connected to
    volatile *timer;    // Pointer to a TimerA register
    DHT_DATA data;      // Data recieved from the sensor
    unsigned int ix;    // Index of bit in the data.bytes array
} DHT;

// DHT22 sensor related definitions

DHT dht = { BIT4, &TACCR0 };
unsigned int arr[42];


void dht_logic(void);
void (*dht_sensor_logic)(void) = dht_logic;


volatile static long cnt1 = 0;
volatile static long cnt2 = 0;
volatile static long cnt3 = 0;
volatile static long cnt4 = 0;
volatile static long cnt5 = 0;

#define TIMER_R0_DELAY  (200 - 1)
#define TIMER_R1_DELAY  (1000 - 1)
#define TIMER_R2_DELAY  (10000 - 1)

/*
Timer usage considerations.
The 1000 kHz source clock means 1 us clock interval.
Thus, max timer value 0xFFFF is equal to ~66 ms (65.535) interval.
Using max divider 8 (ID_3), we can get ~524 ms max interval.
To get ~1 sec interval we can use Up/Down mode: 1000000/(8*65535*2)=0.964 ms.
*/
void setupTimerA0() {
    TACTL = TASSEL_2        // SMCLK clock source
        | MC_2              // Continuous mode up to 0xFFFF
        | ID_0
    ;

    TACCTL0 |= CCIE;        // Enable interrupt on Register 0 value
    // TACCR0 = 0;             // Upper value.

    TACCTL1 |= CCIE;        // Enable interrupt on Register 1 value
    TACCR1 = TIMER_R1_DELAY;

    TACCTL2 |= CCIE;        // Enable interrupt on Register 2 value
    TACCR2 = TIMER_R2_DELAY;
}


// void setupTimerA1() {
//     TA1CTL = TASSEL_2       // SMCLK clock source
//         | MC_1              // Count up to TA1CCR0
//         | ID_3              // Divide by 8, (1000 kHz / 8 = 125 kHz)
//     ;
//     TA1CCTL0 |= CCIE;       // Enable interrupt on Register 0 value
//     TA1CCR0 = 12500 - 1;    // Upper value. Interrupt occurs every 125000 / 12500 = 0.1 sec
// }


void updateLCD(void) {
    // clearBank(3);
    // clearBank(4);
    // clearBank(5);

    // setAddr(10, 3);
    // writeStringToLCD(ul2a(cnt1));

    // setAddr(10, 4);
    // writeStringToLCD(ul2a(cnt2));

    // setAddr(0, 5);
    // writeStringToLCD(ul2a(cnt3));
    // writeStringToLCD(":");
    // writeStringToLCD(ul2a(arr[0]));
    // writeStringToLCD(":");
    // writeStringToLCD(ul2a(arr[1]));
    // writeStringToLCD(":");
    // writeStringToLCD(ul2a(arr[2]));

    int i;

    // Convert sensor time intervals to sensor bits

    int byte;
    // Clear old data
    for (i = 0; i < 5; i++) { dht.data.bytes[i] = 0; }

    for (i = 0; i < 40; i++) {
        byte = i >> 3;
        dht.data.bytes[byte] <<= 1;
        dht.data.bytes[byte] |= arr[i + 1] > 100;
    }

    // bit = dht.ix % 40u;         // Put it in the range 0..39
    // byte = bit >> 3;            // byte = (0..39) / 8 = 0..4
    // bit &= 0x7;                 // bit = (0..15)
    // dht.data.bytes[byte] <<= 1; // Shift left the byte
    // dht.data.bytes[byte] |= 1;

    // Check CRC

    int crc = 0;
    for (i = 0; i < 4; i++) { crc += dht.data.bytes[i]; }
    crc &= 0xff;

    clearLCD();
    // 
    writeStringToLCD(ul2a(cnt3));
    // 0..15, 16..31, 32..39
    for (i = 0; i < 16; i++) {
        writeStringToLCD(":");
        writeStringToLCD(ul2a(arr[i + 0]));
    }
    // CRC
    setAddr(0, 4);
    writeStringToLCD(crc ^ dht.data.val.crc ? "bad" : "ok");
    // writeStringToLCD(ul2hex(crc));
    // writeStringToLCD("-");
    // writeStringToLCD(char2hex(dht.data.val.crc));

    setAddr(0, 5);
    int hum = dht.data.val.hh * 256 + dht.data.val.hl;
    int temp = dht.data.val.th * 256 + dht.data.val.tl;
    writeStringToLCD(ul2a(hum));
    writeStringToLCD(" * ");
    writeStringToLCD(ul2a(temp));
    // for (i = 0; i < 5; i++) {
    //     if(i) { writeStringToLCD("."); }
    //     writeStringToLCD(char2hex(dht.data.bytes[i]));
    // }
}

DHT22 dht22;

void main(void) {

    dht22.test = 8;

    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer

    // Wait while the constants copied to memory if it was erased
    while(CALBC1_1MHZ ==0xFF || CALDCO_1MHZ == 0xFF);
    // Set calibration constants
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

    // while(CALBC1_16MHZ ==0xFF || CALDCO_16MHZ == 0xFF);
    // BCSCTL1 = CALBC1_16MHZ;
    // DCOCTL = CALDCO_16MHZ;


    /*  Default settings after reset:
    *
    *   Source of the Main system clock (MCLK) and sub-main system clock (SMCLK) is 
    *   Internal Digitally Controlled Oscillator Clock (DCO).
    *   MCLK is about 1 MHz.
    *   VLO is about 12 kHz.
    *
    *   Code execution begins in less then 2 us.
    *
    */

    // Setup pins for LCD
    P1OUT |= LCD5110_SCE_PIN | LCD5110_DC_PIN;  // Disable LCD, set Data mode
    P1DIR |= LCD5110_SCE_PIN | LCD5110_DC_PIN;  // Set pins to output direction

    // Setup USIB
    P1SEL |= LCD5110_SCLK_PIN | LCD5110_DN_PIN;
    P1SEL2 |= LCD5110_SCLK_PIN | LCD5110_DN_PIN;

    UCB0CTL0 |= UCCKPH | UCMSB | UCMST | UCSYNC; // 3-pin, 8-bit SPI master
    UCB0CTL1 |= UCSSEL_2;               // SMCLK
    UCB0BR0 |= 0x01;                    // 1:1
    UCB0BR1 = 0;
    UCB0CTL1 &= ~UCSWRST;               // clear SW

    __delay_cycles(500000);
    
    initLCD();
    clearLCD();
    
    setAddr(0, 1);
    writeStringToLCD("MSP-430G2553-3");

    setupTimerA0();
    // setupTimerA1();

    __enable_interrupt();

    _BIS_SR(LPM0_bits);     // Enter LPM0

} // eof main


// ============================================================================
// 
// LCD functions implementation
// 

// Sets address: x (pixels 0..79), y (rows 0..5)
void setAddr(unsigned char xAddr, unsigned char yAddr) {
    writeToLCD(LCD5110_COMMAND, PCD8544_SETXADDR | xAddr);
    writeToLCD(LCD5110_COMMAND, PCD8544_SETYADDR | yAddr);
}

void writeToLCD(unsigned char dataCommand, unsigned char data) {
    LCD5110_SELECT;

    if(dataCommand) {
        LCD5110_SET_DATA;
    } else {
        LCD5110_SET_COMMAND;
    }

    UCB0TXBUF = data;
    while(!(IFG2 & UCB0TXIFG));
    LCD5110_DESELECT;
}

void initLCD() {
    writeToLCD(LCD5110_COMMAND, PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION);
    writeToLCD(LCD5110_COMMAND, PCD8544_SETVOP | 0x3F);
    writeToLCD(LCD5110_COMMAND, PCD8544_SETTEMP | 0x02);
    writeToLCD(LCD5110_COMMAND, PCD8544_SETBIAS | 0x03);
    writeToLCD(LCD5110_COMMAND, PCD8544_FUNCTIONSET);
    writeToLCD(LCD5110_COMMAND, PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);
}

void writeCharToLCD(char c) {
    unsigned char i;
    for(i = 0; i < 5; i++) {
        writeToLCD(LCD5110_DATA, font[c - 0x20][i]);
    }
    writeToLCD(LCD5110_DATA, 0);
}

void writeStringToLCD(const char *string) {
    while(*string) {
        writeCharToLCD(*string++);
    }
}

void clearLCD() {
    setAddr(0, 0);
    int i = 0;
    while(i < PCD8544_MAXBYTES) {
        writeToLCD(LCD5110_DATA, 0);
        i++;
    }
    setAddr(0, 0);
}

void clearBank(unsigned char bank) {
    setAddr(0, bank);
    int i = 0;
    while(i < PCD8544_HPIXELS) {
        writeToLCD(LCD5110_DATA, 0);
        i++;
    }
    setAddr(0, bank);
}

// End of LCD functions
// ============================================================================


// ============================================================================
// 
// Helper functions
// 

// Converts unsigned long to string. Returns string
char* ul2a(unsigned long i) {
    // 32-bit value can fit into 11-byte buffer, including terminating zero.
    static char buf[11];
    char* p = buf + sizeof(buf);
    *--p = '\0';
    do {
        *--p = i % 10 + '0';
    } while (i /= 10);
    return p;
}

// Converts unsigned long to hex string with leading zero. Returns string
char* ul2hex(unsigned long i) {
    // 32-bit value can fit into 9-byte buffer, including terminating zero.
    static char buf[9];
    char* p = buf + sizeof(buf);
    *--p = '\0';
    do {
        *--p = i & 0xf;
        *p += *p > 9 ? 'a'-10 : '0';
        i >>= 4;
    } while (p > buf);
    return p;
}


// Converts byte to hex string with leading zero. Returns string
char* char2hex(unsigned char i) {
    // 8-bit value can fit into 3-byte buffer, including terminating zero.
    static char buf[3];
    char* p = buf + sizeof(buf);
    *--p = '\0';
    do {
        *--p = i & 0xf;
        *p += *p > 9 ? 'a'-10 : '0';
        i >>= 4;
    } while (p > buf);
    return p;
}

// End of helper functions
// ============================================================================

// ============================================================================
// 
// Interrupts
// 
// Interrupts declaration in mspgcc: http://stackoverflow.com/a/15500827
// TimerA with multiple time intervals: http://www.ti.com/lit/an/slaa513a/slaa513a.pdf
// 

// TimerA0 interrupt for register 0
__attribute__((__interrupt__(TIMER0_A0_VECTOR)))
isrTimerA0_R0(void) {
    cnt1++;
    dht_sensor_logic();
}


// TimerA0 interrupt for sources other then register 0
__attribute__((__interrupt__(TIMER0_A1_VECTOR)))
isrTimerA0_IV(void) {
    static i = 0;
    static sec = 0; 
    switch (TAIV) {
        case TA0IV_TACCR1:
            TACCR1 += TIMER_R1_DELAY;
            cnt2++;
            break;
        case TA0IV_TACCR2:
            // Assume interrupt occures each 0.01 sec
            TACCR2 += TIMER_R2_DELAY;
            // Use counter to get 1 sec interval
            if (++i > 99) {
                i = 0;
                sec++;
                if(sec == 3) {
                    // dht_start_read();
                }
                updateLCD();
                cnt1 = 0;
                cnt2 = 0;
            }
            break;
    }
}


// TimerA1 interrupt for register 0
// __attribute__((__interrupt__(TIMER1_A0_VECTOR)))
// isrTimerA1_R0(void) {
//     static int i=0;
//     if (++i > 9) {
//         i = 0;
//         // updateLCD();

//         // cnt1 = 0;
//         // cnt2 = 0;
//     }
// }

unsigned int ix;

__attribute__((__interrupt__(PORT1_VECTOR)))
isrPort1(void) {

    if (ix < 42) {
        arr[ix] = TAR;
        TAR = 0;
        ix++;
    }

    P1IFG &= ~dht.pin;      // Clear port interrupt flag
    cnt3++;
}


// ============================================================================

/*
DHT22 Temperature & Humidity sensor

Micro Controller Unit (MCU) talks with DHT.
Initial state: MCU -> high

Protocol timing table:
    who     level   time, us (range)
    -----------------------------------------
    Handshake
    MCU     low     1000 (800..20000)
    MCU     high    30 (20..200)
    DHT     low     80 (75..85)
    DHT     high    80 (75..85)
    DHT sends 40 bits to MCU
    DHT     low     50 (48..55)
    DHT     high    26 (22..30) if it sends 0
    DHT     high    70 (68..75) if it sends 1
    ...
    DHT     low     50 (45..55)

DHT must have min 2 sec delay between the requests.
DHT must have min 1 sec after power on before the first request.

DHT sends 40 bits (5 words x 8 bit):
    8 + 8 = 16 bit  Humidity multiplied per 10, e.g 985 = 98.5%
    8 + 8 = 16 bit  Temperature multiplied per 10, e.g 240 = 24.0 C
    8 bit Checksum = OR of the fist 4 octets.

In total, DHT may take up to (85 + 85) + (40 * (55 + 75)) + 55 = 5425 us = 6 ms.
Though, it never happens in reality since it never sends 1's only.

DHT sensor help:
https://www.adafruit.com/datasheets/DHT22.pdf
http://embedded-lab.com/blog/measurement-of-temperature-and-relative-humidity-using-dht11-sensor-and-pic-microcontroller/

Min and max time of low-to-high signal for 0 and 1, microseconds:
                       min     max
0: #_____###_           70      85
1: #_____#######_      116     130

*/

inline void dht_logic(void) {
    static int cycles;

    // State machine
    static int ost, st = 0;
    switch (ost = st) {

        case 0: // Wait 2 seconds to allow the sensor make measurements
            if (++cycles < 200) {       // 2sec / 10000us = 200
                *dht.timer += 10000;    // Set delay of 10 ms
                break;
            }
            P1DIR |= dht.pin;    // Set pin to output direction
            P1OUT &= ~dht.pin;   // Set output low

            *dht.timer += 1000;  // Set delay of 1 ms

            st = 1;
            break;

        case 1: // Wait for the sensor response, and process it
            dht.ix = ix = 0;
            cnt3 = 0;
            st = 2;
            /*
            Time critical section
            Disable interrupts we don't want, enable interrupts we need.
            */
            TACCTL1 &= ~CCIE;
            TACCTL2 &= ~CCIE;

            P1OUT |= dht.pin;       // Set output high
            __delay_cycles(40);     // Delay of 40 us at 1 MHz
            P1DIR &= ~dht.pin;      // Set pin to input direction

            // Recieve sensor's 'handshake' signal
            get_lo();
            get_hi();

            P1IES |= dht.pin;       // Interrupt on high-to-low edge
            P1IE |= dht.pin;        // Enable pin interrupt

            TAR = 0;
            *dht.timer = 0;
            *dht.timer += 6000;     // Set timeout of 6 ms (maximum response time)
                                    // (In my setup real response time was about 4 ms)
            break;

        case 2: // Wait for timeout
            P1IE &= ~dht.pin;       // Disable pin interrupt
            /*
            End of time critical section
            Enable interrupts we disabled in the beginning of the section
            */
            TACCTL1 |= CCIE;
            TACCTL2 |= CCIE;
            st = 0;
            break;
    }

    if(ost ^ st) {
        cycles = 0;
    }
}

inline int get_lo() {
    int i = 0;
    while((P1IN & dht.pin) && (++i < 80));
    return i;
}

inline int get_hi() {
    int i = 0;
    while((!(P1IN & dht.pin)) && (++i < 80));
    return i < 80 ? i : 0;
}

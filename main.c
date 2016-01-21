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

/*
    Buffer used to convert a number to string.
    12 bytes is enough to fit signed 32-bit value including terminating zero.
*/
char buf [12]; 
char* ul2a(unsigned long, void*);
char* ul2hex(unsigned long, void*);
char* char2hex(unsigned char, void*);
char* l2a(long, void*);

// Variables for debugging
volatile static long cnt1 = 0;
volatile static long cnt2 = 0;


// DHT22 sensor related definitions

DHT dht = { BIT4, &TACCR0};
// void (*dht_sensor_logic)(DHT*) = dht_logic;

// __attribute__((__interrupt__(PORT1_VECTOR)))
// isrPort1(void) {

//     if (dht.ix < 42) {
//         dht.arr[dht.ix] = TAR - dht.tar;
//         dht.ix++;
//     }

//     // switch (st) {
//     //     case 0:                     // Low level on input
//     //         P1IES &= ~dht.pin;      // Set interrupt on low-to-high edge
//     //         hi_time = TAR - dht.tar;
//     //         if (dht.ix < 42) {
//     //             dht.arr[dht.ix] = lo_time < hi_time;
//     //             dht.tar = TAR;
//     //             dht.ix++;
//     //         }
//     //         st = 1;
//     //     break;

//     //     case 1:                     // High level on input
//     //         P1IES |= dht.pin;       // Set interrupt on high-to-low edge
//     //         lo_time = TAR - dht.tar;
//     //         st = 0;
//     //     break;
//     // }

//     dht.cnt++;
//     dht.tar = TAR;
//     P1IFG &= ~dht.pin;      // Clear port interrupt flag
// }

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

    int i;

    // Convert sensor time intervals to sensor bits

    int byte;
    // Clear old data
    for (i = 0; i < 5; i++) { dht.data.bytes[i] = 0; }

    for (i = 0; i < 40; i++) {
        byte = i >> 3;
        dht.data.bytes[byte] <<= 1;
        dht.data.bytes[byte] |= dht.arr[i + 1] > 110;
    }

    // Check CRC

    int crc = 0;
    for (i = 0; i < 4; i++) { crc += dht.data.bytes[i]; }
    crc &= 0xff;
    
    int ok = !dht.error && !(crc ^ dht.data.val.crc);
    int hum = dht.data.val.hh * 256 + dht.data.val.hl;
    int temp = dht.data.val.th * 256 + dht.data.val.tl;

    clearLCD();
    writeStringToLCD("T  ");
    writeStringToLCD(ok? ul2a(temp, buf) : "-");
    writeCharToLCD(0x7f);
    writeCharToLCD('C');
    setAddr(0, 1);
    writeStringToLCD("RH ");
    writeStringToLCD(ok? ul2a(hum, buf) : "-");
    writeCharToLCD('%');

    if (dht.error) {
        setAddr(0, 2);
        writeStringToLCD("error:");
        writeStringToLCD(l2a(dht.error, buf));
    }

    setAddr(0, 3);
    static unsigned crc_err = 0;
    static unsigned dht_err = 0;
    if (crc ^ dht.data.val.crc) crc_err++;
    if (dht.error) dht_err++;
    setAddr(0, 3); writeStringToLCD("crc err: "); writeStringToLCD(l2a(crc_err, buf));
    setAddr(0, 4); writeStringToLCD("dht err: "); writeStringToLCD(l2a(dht_err, buf));

    setAddr(0, 5);
    writeStringToLCD(l2a(dht.debug, buf));
}


void main(void) {

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

    __enable_interrupt();

    _BIS_SR(LPM0_bits);     // Enter LPM0
    // _BIS_SR(LPM3_bits);     // TODO: Enter LPM3 (use ACLK)

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
char* ul2a(unsigned long i, void *buf) {
    // 32-bit value can fit into 11-byte buffer, including terminating zero.
    char* p = buf + 11;
    *--p = '\0';
    do {
        *--p = i % 10 + '0';
    } while (i /= 10);
    return p;
}

// Converts signed long to string. Returns string
char* l2a(long i, void *buf) {
    char n = i < 0;
    if (n) { i = -i; }
    char* p = ul2a(i, buf + 1);
    if (n) { *--p = '-'; }
    return p;
}

// Converts unsigned long to hex string with leading zero. Returns string
char* ul2hex(unsigned long i, void *buf) {
    // 32-bit value can fit into 9-byte buffer, including terminating zero.
    char* p = buf + 9;
    *--p = '\0';
    do {
        *--p = i & 0xf;
        *p += *p > 9 ? 'a'-10 : '0';
        i >>= 4;
    } while (p > (char*)buf);
    return p;
}


// Converts byte to hex string with leading zero. Returns string
char* char2hex(unsigned char i, void *buf) {
    // 8-bit value can fit into 3-byte buffer, including terminating zero.
    char* p = buf + 3;
    *--p = '\0';
    do {
        *--p = i & 0xf;
        *p += *p > 9 ? 'a'-10 : '0';
        i >>= 4;
    } while (p > (char*)buf);
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
    timerDHT();
}


// TimerA0 interrupt for sources other then register 0
__attribute__((__interrupt__(TIMER0_A1_VECTOR)))
isrTimerA0_IV(void) {
    static i = 0;
    switch (TAIV) {
        case TA0IV_TACCR1:
            TACCR1 += TIMER_R1_DELAY;
        break;

        case TA0IV_TACCR2:
            // Assume interrupt occures each 0.01 sec
            TACCR2 += TIMER_R2_DELAY;
            // Use counter to get 1 sec interval
            if (++i > 99) {
                i = 0;
                updateLCD();
            }
        break;
    }
}

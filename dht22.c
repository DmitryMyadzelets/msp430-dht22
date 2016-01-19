#include <msp430.h>
#include "dht22.h"


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

/*
Returns:
    0   - reading was done
    -1  - error, no initial waiting for low level from the sensor
    -2  - error during waiting for hight level from the sensor
    -3  - error during waiting for low level from the sensor
*/
inline int read_dht() {
    register int i;

    P1OUT |= dht.pin;        // Set output high
    __delay_cycles(40);      // Delay of 40 us at 1 MHz
    P1DIR &= ~dht.pin;       // Set pin to input direction

    dht.tar = TAR;
    while(P1IN & dht.pin) { if ((TAR - dht.tar) > 100) return -1; }

    for (i = 0; i < 42; i++) {
        while(!(P1IN & dht.pin)) { if ((TAR - dht.tar) > 100) return -2; }
        while(P1IN & dht.pin)    { if ((TAR - dht.tar) > 200) return -3; }
        dht.arr[i] = TAR - dht.tar;
        dht.tar = TAR;
    }
    return 0;
}


inline void dht_logic() {
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
            __disable_interrupt();
            dht.ok = read_dht();
            __enable_interrupt();
            st = 2;
            break;

        case 2: // Wait for timeout
            // P1IE &= ~dht.pin;        // Disable pin interrupt
            /*
            End of time critical section
            Enable interrupts we disabled in the beginning of the section
            */
            // TACCTL0 |= CCIE;
            // TACCTL1 |= CCIE;
            // TACCTL2 |= CCIE;
            st = 0;
            break;
    }

    if(ost ^ st) {
        cycles = 0;
    }
}


void startDHT() {
}


void timerDHT() {
    static DHT_STATE ost, st = WAIT_FOR_START;

    switch (ost = st) {
        case WAIT_FOR_TIMER:
        break;

        case WAIT_FOR_START:
            st = WAIT_2_SECONDS;
        break;

        case WAIT_2_SECONDS:
            st = WAIT_FOR_RESPONSE;
        break;

        case WAIT_FOR_RESPONSE:
        break;

        case WAIT_FOR_TIMEOUT:
        break;
    }
}

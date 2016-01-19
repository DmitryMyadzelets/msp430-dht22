#ifndef __DHT_22_H__
#define __DHT_22_H__


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
    unsigned int arr[42];
    unsigned int tar;   // Timer's value
    unsigned int cnt;
    int ok;
} DHT;

inline void dht_logic();

extern DHT dht;

void startDHT();        // Starts reading process
void timerDHT();        // Call it on timer interrupt

typedef enum DHT_STATE {
    WAIT_FOR_TIMER,
    WAIT_FOR_START,
    WAIT_2_SECONDS,
    WAIT_FOR_RESPONSE,
    WAIT_FOR_TIMEOUT
} DHT_STATE;

#endif

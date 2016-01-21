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
    int error;
    DHT_DATA data;      // Data recieved from the sensor
    unsigned int arr[41];
    unsigned int tar;   // Timer's value
    int debug;
} DHT;

extern DHT dht;

void timerDHT();        // Call it on timer interrupt

#endif

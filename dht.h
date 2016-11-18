#ifndef __DHT_H__
#define __DHT_H__

#include <msp430fr4133.h>

uint8_t dht_data[5];
bool dht_valid;
int16_t dht_temp;
int16_t dht_humi;

void dht_init(void);
bool dht_read(void);

#endif // __DHT_H__

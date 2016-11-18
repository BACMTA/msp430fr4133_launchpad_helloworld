#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "dht.h"

/*int16_t dht_temp = 0;
int16_t dht_humi = 0;
uint8_t data[5];
bool dht_valid = false;*/

#define DHT_PIN BIT5
#define DHT_SET_LOW() do{P1OUT&=~DHT_PIN;P1DIR|=DHT_PIN;}while(0)
#define DHT_SET_HIGH() do{P1OUT|=DHT_PIN;P1DIR|=DHT_PIN;}while(0)
#define DHT_PULL_HIGH() do{P1DIR&=~DHT_PIN;P1REN|=BIT5;P1OUT|=DHT_PIN;}while(0)
#define DHT_LEVEL ((P1IN&DHT_PIN)!=0)

void dht_init(void)
{
    P1DIR &= ~BIT5; // P1.5 input
    P1REN |= BIT5; // pullup
    P1OUT |= BIT5; // high
}

uint16_t expectPulse(bool level)
{
    uint16_t count = 0;

    while (DHT_LEVEL == level) {
        if (count++ >= 8000) {
            return 0;
        }
    }

    return count;
}

// read dht sensor data (gets about 25ms)
bool dht_read(void)
{
    memset(dht_data,0,5);

    int i;
    uint16_t cycles[80];
    dht_valid = false;

    DHT_SET_LOW();
    /*__delay_cycles(60000);
    __delay_cycles(60000);
    __delay_cycles(40000); // delay 20millis*/
    __delay_cycles(12000); // delay 1.5millis

    /* timing critical phase (5ms) */
    DHT_SET_HIGH();
    __delay_cycles(40*8); // wait 40us

    DHT_PULL_HIGH();
    __delay_cycles(10*8); // wait 10us

    if (expectPulse(false)==0)
        return false;

    if (expectPulse(true)==0)
        return false;

    for (i=0;i<80;) {
        cycles[i++]=expectPulse(false);
        cycles[i++]=expectPulse(true);
    }
    /* end of timing critical phase */

    for (i=0;i<80;) {
        int index = i/16;
        uint16_t clow = cycles[i++];
        uint16_t chigh = cycles[i++];
        dht_data[index]<<=1;
        if (clow<chigh)
            dht_data[index]|=1;
    }

    if (dht_data[4]==((dht_data[0]+dht_data[1]+dht_data[2]+dht_data[3])&0xFF)) {
        dht_humi = (dht_data[0]<<8)+dht_data[1];
        dht_temp = (dht_data[2]<<8)+dht_data[3];
        dht_valid = true;
        return true;
    }

    return false;
}

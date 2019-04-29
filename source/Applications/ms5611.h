

#ifndef _ms5611_h
#define _ms5611_h

#include "spi.h"
#include "util.h"


#define MS5611_SPI_BAUD		    SPI_BaudRatePrescaler_4	// 11.25Mhz

#define MS5611_SLOTS		    8				// ~13 Hz
#define MS5611_RETRIES              5

typedef struct {
    utilFilter_t tempFilter;
    spiClient_t *spi;
    volatile uint32_t spiFlag;
    volatile uint32_t d1[MS5611_SLOTS];
    volatile uint32_t d2[MS5611_SLOTS];
    uint16_t p[8];
    volatile uint8_t slot;
    uint8_t step;
    uint8_t enabled;
    uint8_t startTempConv;
    uint8_t startPresConv;
    uint8_t adcRead;
    uint8_t initialized;
    float rawTemp;
    volatile float temp;
    volatile float pres;
    volatile uint32_t lastUpdate;
} ms5611Struct_t;

extern ms5611Struct_t ms5611Data;

extern void ms5611PreInit(void);
extern uint8_t ms5611Init(void);
extern void ms5611InitialBias(void);
extern void ms5611Result(int unused);
extern void ms5611Conversion(int unused);
extern void ms5611Decode(void);
extern void ms5611Enable(void);
extern void ms5611Disable(void);
extern void ms5611Enable(void);
extern void ms5611Disable(void);

#endif

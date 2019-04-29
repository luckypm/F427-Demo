/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright 漏 2011-2014  Bill Nesbitt
*/

#ifndef _hmc5983_h
#define _hmc5983_h
#include "aq.h"

#define HMC5983_BYTES		    6//(1+6)
#define HMC5983_SLOT_SIZE	    ((HMC5983_BYTES+sizeof(int)-1) / sizeof(int) * sizeof(int))// 为8
#define HMC5983_SLOTS		    2				

#define HMC5983_RETRIES         5
#define DATE_OUTPUT_RATE		75//数据输出频率75HZ

#define HMC5883L_ADDRESS            0x1E // this device only has one address
#define HMC5883L_DEFAULT_ADDRESS    (HMC5883L_ADDRESS<<1)

#define HMC5883L_RA_CONFIG_A        0x00
#define HMC5883L_RA_CONFIG_B        0x01
#define HMC5883L_RA_MODE            0x02
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAX_L         0x04
#define HMC5883L_RA_DATAY_H         0x05
#define HMC5883L_RA_DATAY_L         0x06
#define HMC5883L_RA_DATAZ_H         0x07
#define HMC5883L_RA_DATAZ_L         0x08
#define HMC5883L_RA_STATUS          0x09
#define HMC5883L_RA_ID_A            0x0A
#define HMC5883L_RA_ID_B            0x0B
#define HMC5883L_RA_ID_C            0x0C

#define iic_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define iic_LONG_TIMEOUT         ((uint32_t)(30 * iic_FLAG_TIMEOUT))
#define iic_OK                    0
#define iic_FAIL                  1   



typedef struct {
    //spiClient_t *spi;
    //volatile uint32_t spiFlag;
    uint8_t rxBuf[HMC5983_SLOT_SIZE*HMC5983_SLOTS];
    volatile uint8_t slot;
    float rawMag[3];
    float mag[3];
    float magSign[3];
    volatile uint32_t lastUpdate;
	uint32_t readUpdate;
    uint8_t readCmd;
    uint8_t enabled;
    uint8_t initialized;
} hmc5983Struct_t;

extern hmc5983Struct_t hmc5983Data;

extern void hmc5983PreInit(void);
extern uint8_t hmc5983Init(void);
extern void hmc5983Decode(void);
extern void hmc5983Enable(void);
extern void hmc5983Disable(void);
extern void hmc5983Read(void);


#endif

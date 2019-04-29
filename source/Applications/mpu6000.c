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

    Copyright ?2011-2014  Bill Nesbitt
*/

#include "mpu6000.h"
#include "util.h"
#include "d_imu.h"

mpu6000Struct_t mpu6000Data;

static void mpu6000TransferComplete(int unused) {
    mpu6000Data.slot = (mpu6000Data.slot + 1) % MPU6000_SLOTS;
}

void mpu6600InitialBias(void) {
    uint32_t lastUpdate = mpu6000Data.lastUpdate;
    float tempSum;
    int i;

    tempSum = 0.0f;

    for (i = 0; i < 50; i++) {
	while (lastUpdate == mpu6000Data.lastUpdate)
	    delay(1);
	lastUpdate = mpu6000Data.lastUpdate;

	tempSum += mpu6000Data.rawTemp;
    }

    mpu6000Data.temp = tempSum / 50.0f;
    utilFilterReset(&mpu6000Data.tempFilter, mpu6000Data.temp);
}

static void mpu6000CalibAcc(float *in, volatile float *out) {
    float a, b, c;
    float x, y, z;

    // bias
    a = +(in[0] + p[IMU_ACC_BIAS_X]);
    b = +(in[1] + p[IMU_ACC_BIAS_Y]);
    c = +(in[2] + p[IMU_ACC_BIAS_Z]);
    // misalignment
    x = a ;
    y = b ;
    z = c;
    // scale
    x /= p[IMU_ACC_SCAL_X];
    y /= p[IMU_ACC_SCAL_Y];
    z /= p[IMU_ACC_SCAL_Z];

    // IMU rotation
    out[0] = x * dImuData.cosRot - y * imuData.sinRot;
    out[1] = y * dImuData.cosRot + x * imuData.sinRot;
    out[2] = z;
}

static void mpu6000ScaleGyo(int32_t *in, float *out, float divisor) {
    float scale;

    scale = 1.0f / ((1<<16) / (MPU6000_GYO_SCALE * 2.0f)) * divisor * DEG_TO_RAD;

    out[0] = mpu6000Data.gyoSign[0] * DIMU_ORIENT_GYO_X * scale;// 
    out[1] = mpu6000Data.gyoSign[1] * DIMU_ORIENT_GYO_Y * scale;//
    out[2] = mpu6000Data.gyoSign[2] * DIMU_ORIENT_GYO_Z * scale;// 
}

static void mpu6000ScaleAcc(int32_t *in, float *out, float divisor) {
    float scale;

    scale = 1.0f / ((1<<16) / (MPU6000_ACC_SCALE * 2.0f)) * divisor * GRAVITY;

    out[0] = mpu6000Data.accSign[0] * DIMU_ORIENT_ACC_X * scale;// 
    out[1] = mpu6000Data.accSign[1] * DIMU_ORIENT_ACC_Y * scale;// 
    out[2] = mpu6000Data.accSign[2] * DIMU_ORIENT_ACC_Z * scale;// 
}

static void mpu6000CalibGyo(float *in, volatile float *out) {
    float a, b, c;
    float x, y, z;

    // bias
    a = +(in[0] + p[IMU_GYO_BIAS_X]);
    b = +(in[1] + p[IMU_GYO_BIAS_Y]);
    c = +(in[2] + p[IMU_GYO_BIAS_Z]);

    // misalignment
    x = a ;
    y = b  ;
    z = c;

    // scale
    x /= p[IMU_GYO_SCAL_X];
    y /= p[IMU_GYO_SCAL_Y];
    z /= p[IMU_GYO_SCAL_Z];

    // IMU rotation
    out[0] = x * dImuData.cosRot - y * imuData.sinRot;
    out[1] = y * dImuData.cosRot + x * imuData.sinRot;
    out[2] = z;
}

void mpu6000DrateDecode(void) {
    volatile uint8_t *d = mpu6000Data.rxBuf;
    int32_t gyo[3];
    float divisor;
    int s, i;

    if (mpu6000Data.enabled) {
        for (i = 0; i < 3; i++)
            gyo[i] = 0;

        divisor = (float)MPU6000_DRATE_SLOTS;
        s = mpu6000Data.slot - 1;
        if (s < 0)
            s = MPU6000_SLOTS - 1;

        for (i = 0; i < MPU6000_DRATE_SLOTS; i++) {
            int j = s*MPU6000_SLOT_SIZE;

            // check if we are in the middle of a transaction for this slot
            if (s == mpu6000Data.slot && mpu6000Data.spiFlag == 0)	{
                divisor -= 1.0f;
            }
            else {
                gyo[0] += (int16_t)__rev16(*(uint16_t *)&d[j+9]);
                gyo[1] += (int16_t)__rev16(*(uint16_t *)&d[j+11]);
                gyo[2] += (int16_t)__rev16(*(uint16_t *)&d[j+13]);
            }

            if (--s < 0)
                s = MPU6000_SLOTS - 1;
        }

        divisor = 1.0f / divisor;

        mpu6000ScaleGyo(gyo, mpu6000Data.dRateRawGyo, divisor);
        mpu6000CalibGyo(mpu6000Data.dRateRawGyo, mpu6000Data.dRateGyo);
    }
}

void mpu6000Decode(void) {
    volatile uint8_t *d = mpu6000Data.rxBuf;
    int32_t acc[3], temp, gyo[3];
    float divisor;
    int i;

    for (i = 0; i < 3; i++) {
	acc[i] = 0;
	gyo[i] = 0;
    }
    temp = 0;

    divisor = (float)MPU6000_SLOTS;
    for (i = 0; i < MPU6000_SLOTS; i++) {
	int j = i*MPU6000_SLOT_SIZE;

	// check if we are in the middle of a transaction for this slot
	if (i == mpu6000Data.slot && mpu6000Data.spiFlag == 0)	{
	    divisor -= 1.0f;
	}
	else {
	    acc[0] += (int16_t)__rev16(*(uint16_t *)&d[j+1]);
	    acc[1] += (int16_t)__rev16(*(uint16_t *)&d[j+3]);
	    acc[2] += (int16_t)__rev16(*(uint16_t *)&d[j+5]);

	    temp += (int16_t)__rev16(*(uint16_t *)&d[j+7]);

	    gyo[0] += (int16_t)__rev16(*(uint16_t *)&d[j+9]);
	    gyo[1] += (int16_t)__rev16(*(uint16_t *)&d[j+11]);
	    gyo[2] += (int16_t)__rev16(*(uint16_t *)&d[j+13]);
	}
    }

    divisor = 1.0f / divisor;

    mpu6000Data.rawTemp = temp * divisor * (1.0f / 340.0f) + 36.53f;//公式见MPU6000说明书
    mpu6000Data.temp = utilFilter(&mpu6000Data.tempFilter, mpu6000Data.rawTemp);

    mpu6000ScaleAcc(acc, mpu6000Data.rawAcc, divisor);
    mpu6000CalibAcc(mpu6000Data.rawAcc, mpu6000Data.acc);

    mpu6000ScaleGyo(gyo, mpu6000Data.rawGyo, divisor);
    mpu6000CalibGyo(mpu6000Data.rawGyo, mpu6000Data.gyo);

    mpu6000Data.lastUpdate = timerMicros();
}

static uint32_t mpu6000RxBuf;
static uint32_t mpu6000TxBuf;

static uint8_t mpu6000GetReg(uint8_t reg) {
    ((uint8_t *)&mpu6000TxBuf)[0] = MPU6000_READ_BIT | reg;

    mpu6000Data.spiFlag = 0;
    spiTransaction(mpu6000Data.spi, &mpu6000RxBuf, &mpu6000TxBuf, 2);

    while (!mpu6000Data.spiFlag)
        ;

    return ((uint8_t *)&mpu6000RxBuf)[1];
}

static void mpu6000SetReg(uint8_t reg, uint8_t val) {
    ((uint8_t *)&mpu6000TxBuf)[0] = MPU6000_WRITE_BIT | reg;
    ((uint8_t *)&mpu6000TxBuf)[1] = val;

    mpu6000Data.spiFlag = 0;
    spiTransaction(mpu6000Data.spi, &mpu6000RxBuf, &mpu6000TxBuf, 2);

    while (!mpu6000Data.spiFlag)
       ;
}

static void mpu6000ReliablySetReg(uint8_t reg, uint8_t val) {
	u8 regValue = 0;
    do {
        delay(10);
        mpu6000SetReg(reg, val);
        delay(10);
		regValue = mpu6000GetReg(reg);
    } while (regValue != val);
}

static void mpu6000IntHandler(void) {
    if (mpu6000Data.enabled)
        spiTransaction(mpu6000Data.spi, &mpu6000Data.rxBuf[mpu6000Data.slot*MPU6000_SLOT_SIZE], &mpu6000Data.readReg, MPU6000_BYTES);
}

inline void mpu6000Enable(void) {
    mpu6000Data.enabled = 1;
}

inline void mpu6000Disable(void) {
    mpu6000Data.enabled = 0;
}

void mpu6000PreInit(void) {
    mpu6000Data.spi = spiClientInit(DIMU_MPU6000_SPI, MPU6000_SPI_REG_BAUD, 0, DIMU_MPU6000_CS_PORT, DIMU_MPU6000_CS_PIN, &mpu6000Data.spiFlag, 0);
}

void mpu6000Init(void) {
    switch ((int)p[IMU_FLIP]) {
        case 1:// IMU 沿X轴翻转
            mpu6000Data.accSign[0] =  1.0f;
            mpu6000Data.accSign[1] = -1.0f;
            mpu6000Data.accSign[2] = -1.0f;
            mpu6000Data.gyoSign[0] =  1.0f;
            mpu6000Data.gyoSign[1] = -1.0f;
            mpu6000Data.gyoSign[2] = -1.0f;
            break;

        case 2:// IMU 沿Y轴翻转
            mpu6000Data.accSign[0] = -1.0f;
            mpu6000Data.accSign[1] =  1.0f;
            mpu6000Data.accSign[2] = -1.0f;
            mpu6000Data.gyoSign[0] = -1.0f;
            mpu6000Data.gyoSign[1] =  1.0f;
            mpu6000Data.gyoSign[2] = -1.0f;
            break;

        case 0:
        default:
            mpu6000Data.accSign[0] = 1.0f;
            mpu6000Data.accSign[1] = 1.0f;
            mpu6000Data.accSign[2] = 1.0f;
            mpu6000Data.gyoSign[0] = 1.0f;
            mpu6000Data.gyoSign[1] = 1.0f;
            mpu6000Data.gyoSign[2] = 1.0f;
            break;
    }

    utilFilterInit(&mpu6000Data.tempFilter, DIMU_OUTER_DT, DIMU_TEMP_TAU, 20.0f);

    // reset
    mpu6000SetReg(107, 0x80);
    delay(100);

    // wake up w/ Z axis clock reg
    mpu6000ReliablySetReg(107, 0x03);

    // disable I2C interface
    mpu6000ReliablySetReg(106, 0x10);

    // wait for a valid response
    while (mpu6000GetReg(117) != 0x68)
	delay(10);

    // GYO scale
#if MPU6000_GYO_SCALE == 250
    mpu6000ReliablySetReg(27, 0b00000);//keil不支持二进制，要转换成16进制才能用
#endif
#if MPU6000_GYO_SCALE == 500
    mpu6000ReliablySetReg(27, 0b01000);
#endif
#if MPU6000_GYO_SCALE == 1000
    mpu6000ReliablySetReg(27, 0x10);
#endif
#if MPU6000_GYO_SCALE == 2000
    mpu6000ReliablySetReg(27, 0b11000);
#endif

    // ACC scale
#if MPU6000_ACC_SCALE == 2
    mpu6000ReliablySetReg(28, 0b00000);
#endif
#if MPU6000_ACC_SCALE == 4
    mpu6000ReliablySetReg(28, 0b01000);
#endif
#if MPU6000_ACC_SCALE == 8
    mpu6000ReliablySetReg(28, 0x10);
#endif
#if MPU6000_ACC_SCALE == 16
    mpu6000ReliablySetReg(28, 0b11000);
#endif

    // Sample rate
    mpu6000ReliablySetReg(25, 0x00);

    // LPF
    mpu6000ReliablySetReg(26, 0x00);

    // Interrupt setup
    mpu6000ReliablySetReg(55, 0x01<<4);
    mpu6000ReliablySetReg(56, 0x01);

    // bump clock rate up to 21MHz
    spiChangeBaud(mpu6000Data.spi, MPU6000_SPI_RUN_BAUD);

    mpu6000Data.readReg = MPU6000_READ_BIT | 0x3b;	// start of sensor registers 为OXBB,即187

    spiChangeCallback(mpu6000Data.spi, mpu6000TransferComplete);

    // External Interrupt line for data ready
    extRegisterCallback(DIMU_MPU6000_INT_PORT, DIMU_MPU6000_INT_PIN, EXTI_Trigger_Rising, 1, GPIO_PuPd_NOPULL, mpu6000IntHandler);

}


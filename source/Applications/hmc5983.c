/*
  将原5983驱动，改成HMC5883的IIC通信
*/
#include "hmc5983.h"
#include "util.h"
#include "d_imu.h"


__IO uint32_t  iicTimeout = iic_LONG_TIMEOUT;   

hmc5983Struct_t hmc5983Data;

/**
 * @brief  Initializes the I2C peripheral used to drive the HMC5883L
 * @param  None
 * @retval None
 */
static void HMC5883L_I2C_Init(void)
{
    I2C_InitTypeDef I2C_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable I2C and GPIO clocks */
    RCC_APB1PeriphClockCmd(HMC5883L_I2C_RCC_Periph, ENABLE);
    RCC_AHB1PeriphClockCmd(HMC5883L_I2C_RCC_Port, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Configure I2C pins: SCL and SDA */
    GPIO_InitStructure.GPIO_Pin = HMC5883L_I2C_SCL_Pin | HMC5883L_I2C_SDA_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(HMC5883L_I2C_Port, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(HMC5883L_I2C_Port, HMC5883L_I2C_SCL_Souce, HMC5883L_I2C_SCL_AF);
	GPIO_PinAFConfig(HMC5883L_I2C_Port, HMC5883L_I2C_SDA_Souce, HMC5883L_I2C_SDA_AF);
	
    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0xE0;//HMC5883L_DEFAULT_ADDRESS; // HMC5883L 7-bit adress = 0x1E;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = HMC5883L_I2C_Speed;

    /* Apply I2C configuration after enabling it */
    I2C_Init(HMC5883L_I2C, &I2C_InitStructure);

    I2C_Cmd(HMC5883L_I2C, ENABLE);
}

/**
 * @brief  Writes one byte to the  HMC5883L.
 * @param  slaveAddr : slave address HMC5883L_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer  containing the data to be written to the HMC5883L.
 * @param  WriteAddr : address of the register in which the data will be written
 * @retval None
 */
static uint32_t HMC5883L_I2C_ByteWrite(u8 slaveAddr, u8* pBuffer, u8 WriteAddr)
{
    // ENTR_CRT_SECTION();
    /* Send START condition */
    I2C_GenerateSTART(HMC5883L_I2C, ENABLE);

    /* Test on EV5 and clear it */
	iicTimeout = iic_LONG_TIMEOUT;
    while (!I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_MODE_SELECT))
	{
    	if((iicTimeout--) == 0) return iic_FAIL;
    }

    /* Send HMC5883 address for write */
    I2C_Send7bitAddress(HMC5883L_I2C, slaveAddr, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
	iicTimeout = iic_LONG_TIMEOUT;
    while (!I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
    	if((iicTimeout--) == 0) return iic_FAIL;
    }
    /* Send the HMC5883L internal address to write to */
    I2C_SendData(HMC5883L_I2C, WriteAddr);

    /* Test on EV8 and clear it */
	iicTimeout = iic_LONG_TIMEOUT;
    while (!I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
    	if((iicTimeout--) == 0) return iic_FAIL;
    }

    /* Send the byte to be written */
    I2C_SendData(HMC5883L_I2C, *pBuffer);

    /* Test on EV8 and clear it */
	iicTimeout = iic_LONG_TIMEOUT;
    while (!I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
    	if((iicTimeout--) == 0) return iic_FAIL;
    }

    /* Send STOP condition */
    I2C_GenerateSTOP(HMC5883L_I2C, ENABLE);
	return iic_OK;
    // EXT_CRT_SECTION();
}

/**
 * @brief  Reads a block of data from the HMC5883L.
 * @param  slaveAddr  : slave address HMC5883L_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer that receives the data read from the HMC5883L.
 * @param  ReadAddr : HMC5883L's internal address to read from.
 * @param  NumByteToRead : number of bytes to read from the HMC5883L ( NumByteToRead >1  only for the Magnetometer reading).
 * @retval None
 */
static uint32_t HMC5883L_I2C_BufferRead(u8 slaveAddr, u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{
    // ENTR_CRT_SECTION();
    /* While the bus is busy */
	iicTimeout = iic_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(HMC5883L_I2C, I2C_FLAG_BUSY))
	{
    	if((iicTimeout--) == 0) return iic_FAIL;
    }
    /* Send START condition */
    I2C_GenerateSTART(HMC5883L_I2C, ENABLE);

    /* Test on EV5 and clear it */
	iicTimeout = iic_LONG_TIMEOUT;
    while (!I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    {
    	if((iicTimeout--) == 0) return iic_FAIL;
    }

    /* Send HMC5883L_Magn address for write */ // Send HMC5883L address for write
    I2C_Send7bitAddress(HMC5883L_I2C, slaveAddr, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
	iicTimeout = iic_LONG_TIMEOUT;
    while (!I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
    	if((iicTimeout--) == 0) return iic_FAIL;
    }

    /* Clear EV6 by setting again the PE bit */
    I2C_Cmd(HMC5883L_I2C, ENABLE);

    /* Send the HMC5883L's internal address to write to */
    I2C_SendData(HMC5883L_I2C, ReadAddr);

    /* Test on EV8 and clear it */
	iicTimeout = iic_LONG_TIMEOUT;
    while (!I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
    	if((iicTimeout--) == 0) return iic_FAIL;
    }
    /* Send STRAT condition a second time */
    I2C_GenerateSTART(HMC5883L_I2C, ENABLE);

    /* Test on EV5 and clear it */
	iicTimeout = iic_LONG_TIMEOUT;
    while (!I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_MODE_SELECT))
	{
    	if((iicTimeout--) == 0) return iic_FAIL;
    }

    /* Send HMC5883L address for read */
    I2C_Send7bitAddress(HMC5883L_I2C, slaveAddr, I2C_Direction_Receiver);

    /* Test on EV6 and clear it */
	iicTimeout = iic_LONG_TIMEOUT;
    while (!I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
    	if((iicTimeout--) == 0) return iic_FAIL;
    }
    /* While there is data to be read */
    while (NumByteToRead)
    {
        if (NumByteToRead == 1)
        {
            /* Disable Acknowledgement */
            I2C_AcknowledgeConfig(HMC5883L_I2C, DISABLE);

            /* Send STOP Condition */
            I2C_GenerateSTOP(HMC5883L_I2C, ENABLE);
        }

        /* Test on EV7 and clear it */
        if (I2C_CheckEvent(HMC5883L_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            /* Read a byte from the HMC5883L */
            *pBuffer = I2C_ReceiveData(HMC5883L_I2C);

            /* Point to the next location where the byte read will be saved */
            pBuffer++;

            /* Decrement the read bytes counter */
            NumByteToRead--;
        }
    }

    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(HMC5883L_I2C, ENABLE);
	return iic_OK;
    // EXT_CRT_SECTION();
}


static void hmc5983TransferComplete(int unused) {
    hmc5983Data.slot = (hmc5983Data.slot + 1) % HMC5983_SLOTS;
}

static void hmc5983ScaleMag(int32_t *in, float *out, float divisor) {
    float scale;

   // scale = divisor * (1.0f / 187.88f); 
   scale = divisor * (1.0f / 660.0f); 

    out[0] = hmc5983Data.magSign[0] * DIMU_ORIENT_MAG_X * scale;//DIMU_ORIENT_MAG_X为-in[0]
    out[1] = hmc5983Data.magSign[1] * DIMU_ORIENT_MAG_Y * scale;//+in[1]
    out[2] = hmc5983Data.magSign[2] * DIMU_ORIENT_MAG_Z * scale;//-in[2]
}

static void hmc5983CalibMag(float *in, volatile float *out) {
    float a, b, c;
    float x, y, z;

    // bias
    a = +(in[0] );
    b = +(in[1] );
    c = -(in[2] );

    // misalignment
    x = a ;
    y = b ;
    z = c;

    // scale
    x /= 1;
    y /= 1;
    z /= 1;

    out[0] = x ;// 顺时针旋转
    out[1] = y ;
    out[2] = z;
}

void hmc5983Read(void){
	if (hmc5983Data.enabled){
		if((timerMicros() - hmc5983Data.readUpdate) > ((uint32_t)(1.0f/DATE_OUTPUT_RATE*1.0e6f))){
			hmc5983Data.readUpdate = timerMicros();
			UTIL_ISR_DISABLE;
			HMC5883L_I2C_BufferRead(HMC5883L_DEFAULT_ADDRESS, &hmc5983Data.rxBuf[hmc5983Data.slot*HMC5983_SLOT_SIZE], hmc5983Data.readCmd, HMC5983_BYTES);
			UTIL_ISR_ENABLE;
			hmc5983TransferComplete(0);
		}
	}
}

void hmc5983Decode(void) {
    volatile uint8_t *d = hmc5983Data.rxBuf;
    int32_t mag[3];
    float divisor;
    int i;
    if (hmc5983Data.enabled) {
        mag[0] = 0;
        mag[1] = 0;
        mag[2] = 0;

        divisor = (float)HMC5983_SLOTS;
        for (i = 0; i < HMC5983_SLOTS; i++) {
            int j = i*HMC5983_SLOT_SIZE;   
                mag[1] += (int16_t)__rev16(*(uint16_t *)&d[j+0]);//注意这里是mag[1],相当于mag[1] = mag x
                mag[2] += (int16_t)__rev16(*(uint16_t *)&d[j+2]);//mag[2] = mag z
                mag[0] += (int16_t)__rev16(*(uint16_t *)&d[j+4]);//mag[0] = mag y 
        }

        divisor = 1.0f / divisor;

        hmc5983ScaleMag(mag, hmc5983Data.rawMag, divisor);
        hmc5983CalibMag(hmc5983Data.rawMag, hmc5983Data.mag);

        hmc5983Data.lastUpdate = timerMicros();
    }
}

static uint8_t hmc5983GetReg(uint8_t reg) {
    static uint8_t rxBuf[2];

    //txBuf[0] = HMC5983_READ_BIT | reg;
     UTIL_ISR_DISABLE;
	 HMC5883L_I2C_BufferRead(HMC5883L_DEFAULT_ADDRESS, rxBuf, reg, 1);
	 UTIL_ISR_ENABLE;

//    hmc5983Data.spiFlag = 0;
//    spiTransaction(hmc5983Data.spi, rxBuf, txBuf, 2);
//
//    while (!hmc5983Data.spiFlag)
//        ;

    return rxBuf[0];
}

static void hmc5983SetReg(uint8_t reg, uint8_t val) {
    static uint8_t txBuf[2];
	txBuf[0] = val;
	UTIL_ISR_DISABLE;
	HMC5883L_I2C_ByteWrite(HMC5883L_DEFAULT_ADDRESS, txBuf, reg);
	UTIL_ISR_ENABLE;

//    txBuf[0] = HMC5983_WRITE_BIT | reg;
//    txBuf[1] = val;
//
//    hmc5983Data.spiFlag = 0;
//    spiTransaction(hmc5983Data.spi, rxBuf, txBuf, 2);
//
//    while (!hmc5983Data.spiFlag)
//        ;
}

static void hmc5983ReliablySetReg(uint8_t reg, uint8_t val) {
    uint8_t ret;

    do {
        delay(10);
        hmc5983SetReg(reg, val);
        delay(10);
        ret = hmc5983GetReg(reg);
    } while (ret != val);
}

//void hmc5983IntHandler(void) {
//    if (hmc5983Data.enabled){
//        //spiTransaction(hmc5983Data.spi, &hmc5983Data.rxBuf[hmc5983Data.slot*HMC5983_SLOT_SIZE], &hmc5983Data.readCmd, HMC5983_BYTES);
//		HMC5883L_I2C_BufferRead(HMC5883L_DEFAULT_ADDRESS, &hmc5983Data.rxBuf[hmc5983Data.slot*HMC5983_SLOT_SIZE], hmc5983Data.readCmd, HMC5983_BYTES);
//		hmc5983TransferComplete(0);
//    }
//}

inline void hmc5983Enable(void) {
    if (hmc5983Data.initialized)
        hmc5983Data.enabled = 1;
    }

inline void hmc5983Disable(void) {
    hmc5983Data.enabled = 0;
}

void hmc5983PreInit(void) {
   HMC5883L_I2C_Init();
}

uint8_t hmc5983Init(void) {
    int i = HMC5983_RETRIES;

    switch (0) {
        case 1:
            hmc5983Data.magSign[0] =  1.0f;
            hmc5983Data.magSign[1] = -1.0f;
            hmc5983Data.magSign[2] = -1.0f;
            break;

        case 2:
            hmc5983Data.magSign[0] = -1.0f;
            hmc5983Data.magSign[1] =  1.0f;
            hmc5983Data.magSign[2] = -1.0f;
            break;

        case 0:
        default:
            hmc5983Data.magSign[0] = 1.0f;
            hmc5983Data.magSign[1] = 1.0f;
            hmc5983Data.magSign[2] = 1.0f;
            break;
    }

    // wait for a valid response
    while (--i && hmc5983GetReg(HMC5883L_RA_ID_A) != 'H')
        delay(100);

    if (i > 0) {
        // 75Hz, 8x oversample 数据输出寄存器更新频率75HZ，每次测量输出时采样平均数为8
        hmc5983ReliablySetReg(HMC5883L_RA_CONFIG_A, 0xF8);
        delay(10);
        //    // highest gain (+-0.88 Ga)
        //    hmc5983ReliablySetReg(0x01, 0b00000000);
        // gain (+-2.5 Ga) 增益=660=LSB/GAUS 推荐使用范围为+-2.5高斯
        hmc5983ReliablySetReg(HMC5883L_RA_CONFIG_B, 0x60);
        delay(10);

        hmc5983ReliablySetReg(HMC5883L_RA_MODE, 0x00);
        delay(10);

        hmc5983Data.readCmd = HMC5883L_RA_DATAX_H;

        // External Interrupt line for data ready
        //extRegisterCallback(DIMU_HMC5883L_INT_PORT, DIMU_HMC5883L_INT_PIN, EXTI_Trigger_Rising, 1, GPIO_PuPd_NOPULL, hmc5983IntHandler);

        hmc5983Data.initialized = 1;
    }
    else {
        hmc5983Data.initialized = 0;
    }

    return hmc5983Data.initialized;
}


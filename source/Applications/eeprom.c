/*
将AQ中的cat25512 EEPROM驱动稍作修改，改成FM25V01铁电存储器的驱动，没有使用FM25V01的快速读取功能
*/

#include "config.h"
#include "eeprom.h"
#include "util.h"
#include <string.h>

eepromStruct_t eepromData;

void eepromPreInit(void) {
    eepromData.spi = spiClientInit(DIMU_EEPROM_SPI, DIMU_EEPROM_SPI_BAUD, 0, DIMU_EEPROM_CS_PORT, DIMU_EEPROM_CS_PIN, &eepromData.spiFlag, 0);
}

void eepromWriteStatus(int8_t status) {
    eepromData.buf.cmd = EEPROM_WRSR;
    *(uint8_t *)&eepromData.buf.addr = status;

    eepromData.spiFlag = 0;
    spiTransaction(eepromData.spi, &eepromData.buf, &eepromData.buf, 2);

    while (!eepromData.spiFlag)
	vTaskDelay(1);

    vTaskDelay(5);
}
//FM25V01是可以读ID的
void eepromReadID(void) {
    eepromData.buf.cmd = EEPROM_RDID;

    eepromData.spiFlag = 0;
    spiTransaction(eepromData.spi, &eepromData.buf, &eepromData.buf, 10);

    while (!eepromData.spiFlag)
	vTaskDelay(1);

}

void eepromReadStatus(void) {
    eepromData.buf.cmd = EEPROM_RDSR;

    eepromData.spiFlag = 0;
    spiTransaction(eepromData.spi, &eepromData.buf, &eepromData.buf, 2);

    while (!eepromData.spiFlag)
	vTaskDelay(1);

    eepromData.status = *(uint8_t *)(&eepromData.buf.addr);
}

void eepromReadBlock(uint16_t address, int size) {
    eepromData.buf.cmd = EEPROM_READ;
    eepromData.buf.addr[1] = (address&0xff);
    eepromData.buf.addr[0] = (address>>8);

    eepromData.spiFlag = 0;
    spiTransaction(eepromData.spi, &eepromData.buf, &eepromData.buf, size+3);

    while (!eepromData.spiFlag)
	vTaskDelay(1);

//debug_printf("read %x, %x %c %c %c %c\n", address, size, eepromData.buf.data[0], eepromData.buf.data[1], eepromData.buf.data[2], eepromData.buf.data[3]);
}

void eepromWriteEnable(void) {
    eepromData.buf.cmd = EEPROM_WREN;

    eepromData.spiFlag = 0;
    spiTransaction(eepromData.spi, &eepromData.buf, &eepromData.buf, 1);

    while (!eepromData.spiFlag)
	vTaskDelay(1);
}

void eepromWriteBlock(uint16_t address, int size) {
    eepromWriteEnable();

    eepromData.buf.cmd = EEPROM_WRITE;
    eepromData.buf.addr[1] = (address&0xff);
    eepromData.buf.addr[0] = (address>>8);
//debug_printf("write %x, %x %c %c %c %c\n", address, size, eepromData.buf.data[0], eepromData.buf.data[1], eepromData.buf.data[2], eepromData.buf.data[3]);
    eepromData.spiFlag = 0;
    spiTransaction(eepromData.spi, &eepromData.buf, &eepromData.buf, size+3);

    while (!eepromData.spiFlag)
	vTaskDelay(1);
//铁电存储器写基本没延时，不需要下面这个
//    do {
//	vTaskDelay(1);
//	eepromReadStatus();
//    } while (eepromData.status & 0b01);
}

void eepromChecksum(void *memory, int size) {
    uint8_t *p;
    int i;

    p = (uint8_t *)memory;
    for (i = 0; i < size; i++) {
	eepromData.ck[0] += *p++;
	eepromData.ck[1] += eepromData.ck[0];
    }
}

uint32_t eepromReadHeader(void) {
    uint32_t seq = 0;

    eepromReadBlock(0x0000, sizeof(eepromHeader_t));
    memcpy(&eepromData.header, &eepromData.buf.data, sizeof(eepromHeader_t));

    eepromData.ck[0] = 0;
    eepromData.ck[1] = 0;
    eepromChecksum(&eepromData.header, sizeof(eepromHeader_t)-2);

    // is eepromData header valid
    if (eepromData.header.signature == EEPROM_SIGNATURE && eepromData.header.headerCk[0] == eepromData.ck[0] && eepromData.header.headerCk[1] == eepromData.ck[1])
	seq = eepromData.header.seq;

    return seq;
}

void eepromWriteHeader(void) {
    eepromData.ck[0] = 0;
    eepromData.ck[1] = 0;
    eepromChecksum(&eepromData.header, sizeof(eepromHeader_t)-2);

    eepromData.header.headerCk[0] = eepromData.ck[0];
    eepromData.header.headerCk[1] = eepromData.ck[1];

    memcpy(&eepromData.buf.data, &eepromData.header, sizeof(eepromHeader_t));
    eepromWriteBlock(0x0000, sizeof(eepromHeader_t));
}

uint8_t eepromFormat(void) {
    eepromData.header.signature = EEPROM_SIGNATURE;
    eepromData.header.version = EEPROM_VERSION;
    eepromData.header.seq = 0x01;
    eepromData.header.start = DIMU_EEPROM_BLOCK_SIZE;
    eepromData.header.size = 0;

    eepromWriteHeader();
    eepromReadHeader();

    if (eepromData.header.seq == 0x01)
	return 1;
    else
	return 0;
}

uint8_t *eepromOpenWrite(void) {
    eepromReadHeader();

    eepromData.header.seq++;
    eepromData.header.start = 0x40;//保持这个起始地址，铁电存储器接近可写无限次
    eepromData.header.size = 0;
    eepromData.header.fileCk[0] = 0;
    eepromData.header.fileCk[1] = 0;
    eepromData.header.headerCk[0] = 0;
    eepromData.header.headerCk[1] = 0;

    eepromData.ck[0] = 0;
    eepromData.ck[1] = 0;

    return eepromData.buf.data;
}

uint8_t eepromCheckFile(void) {
    uint8_t ret = 0;
    int i;

    eepromData.ck[0] = 0;
    eepromData.ck[1] = 0;
    for (i = 0; i < eepromData.header.size; i += DIMU_EEPROM_BLOCK_SIZE) {
	eepromReadBlock(eepromData.header.start + i, DIMU_EEPROM_BLOCK_SIZE);
	eepromChecksum(eepromData.buf.data, DIMU_EEPROM_BLOCK_SIZE);//这里也是最后一个block的校验信息有效
    }

    if (eepromData.ck[0] == eepromData.header.fileCk[0] && eepromData.ck[1] == eepromData.header.fileCk[1])
	ret = 1;

    return ret;
}

uint8_t *eepromOpenRead(void) {
    uint8_t *ret = 0;

    if (eepromReadHeader() != 0 && eepromData.header.size != 0 && eepromCheckFile() != 0) {
	eepromData.readPointer = 0;
	ret = eepromData.buf.data;
    }

    return ret;
}

uint8_t eepromRead(int size) {
    uint8_t actualSize;

    if ((eepromData.readPointer + size) > eepromData.header.size)
	actualSize = eepromData.header.size - eepromData.readPointer;
    else
	actualSize = size;

    if (actualSize > 0) {
	eepromReadBlock(eepromData.header.start + eepromData.readPointer, actualSize);
	eepromData.readPointer += actualSize;
    }

    return actualSize;
}

void eepromWrite(void) {
    eepromChecksum(eepromData.buf.data, DIMU_EEPROM_BLOCK_SIZE);
    eepromData.header.fileCk[0] = eepromData.ck[0];//这个fileCK最终存储的是最后一次调用此函数时计算的
    eepromData.header.fileCk[1] = eepromData.ck[1];

    eepromWriteBlock(eepromData.header.start + eepromData.header.size, DIMU_EEPROM_BLOCK_SIZE);
    eepromData.header.size += DIMU_EEPROM_BLOCK_SIZE;
}

void eepromClose(void) {
    eepromWriteHeader();
}

void eepromInit(void) {
    if (eepromReadHeader() == 0 || eepromData.header.version < EEPROM_VERSION)
	eepromFormat();
}


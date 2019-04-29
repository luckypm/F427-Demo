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

#include "aq.h"
#include "util.h"
#include "comm.h"
#include "flash.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint32_t heapUsed, heapHighWater, dataSramUsed;
uint32_t *ccmHeap[UTIL_CCM_HEAP_SIZE] __attribute__((section(".ccm")));//指针数组

void utilTaskPeriodTime(uint32_t *time){	
	time[0] = timerMicros() - time[1];
	time[1] = timerMicros();
}

//可以知道使用了多少堆内存，以及出现过的最高水位堆使用量，防止堆内存不够
void *aqCalloc(size_t count, size_t size) {
    char *addr = 0;

    if (count * size) {
        addr = calloc(count, size);

        heapUsed += count * size;
        if (heapUsed > heapHighWater)
            heapHighWater = heapUsed;

        if (addr == 0)
            AQ_NOTICE("Out of heap memory!\n");
    }

    return addr;
}

void aqFree(void *ptr, size_t count, size_t size) {
    if (ptr) {
        free(ptr);
        heapUsed -= count * size;
    }
}

// allocates memory from 64KB CCM//从64KB的CCM中分配存储空间
void *aqDataCalloc(uint16_t count, uint16_t size) {
    uint32_t words;

    // round up to word size 4字节对齐
    words = (count*size + sizeof(int)-1) / sizeof(int);

    if ((dataSramUsed + words) > UTIL_CCM_HEAP_SIZE) {
        AQ_NOTICE("Out of data SRAM!\n");
    }
    else {
        dataSramUsed += words;
    }
	//ccmHeap[0] ccmHeap[1] ...是指向Uint32_t数据的指针，下面返回的是相应指针比如ccmHeap[n]的本身的地址
    return (void *)(ccmHeap + dataSramUsed - words);
}

void utilSerialNoString(void) {
    AQ_PRINTF("MCU S/N: %08X-%08X-%08X\n", flashSerno(2), flashSerno(1), flashSerno(0));
}

void utilVersionString(void) {
    AQ_PRINTF("FW ver: %s, HW ver: %s,DATE: %s\n", FIMRWARE_VERSION, BOARD_VERSION,__DATE__);
}

void info(void) {
    utilSerialNoString();

    //AQ_PRINTF("SYS Clock: %u MHz\n", rccClocks.SYSCLK_Frequency / 1000000);
    AQ_PRINTF("%u/%u heap used/high water\n", heapUsed, heapHighWater);
    AQ_PRINTF("%u of %u CCM heap used\n", dataSramUsed * sizeof(int), UTIL_CCM_HEAP_SIZE * sizeof(int));

    vTaskDelay(100);
    utilVersionString();
}

void utilFilterReset(utilFilter_t *f, float setpoint) {
    f->z1 = setpoint;
}

void utilFilterReset3(utilFilter_t *f, float setpoint) {
    utilFilterReset(&f[0], setpoint);
    utilFilterReset(&f[1], setpoint);
    utilFilterReset(&f[2], setpoint);
}

// larger tau, smoother filter// util可译作工具，通用工具，utilFilter可译作滤波工具或通用滤波器
void utilFilterInit(utilFilter_t *f, float dt, float tau, float setpoint) {// tau 读作tao，为一阶惯性环节的时间常数，如果是在RC滤波器中，tau = RC,AQ中的这个值是怎么来的不清楚
    f->tc = dt / tau;
    utilFilterReset(f, setpoint);
}

void utilFilterInit3(utilFilter_t *f, float dt, float tau, float setpoint) {
    utilFilterInit(&f[0], dt, tau, setpoint);
    utilFilterInit(&f[1], dt, tau, setpoint);
    utilFilterInit(&f[2], dt, tau, setpoint);
}

float utilFilter(utilFilter_t *f, float signal) {// 这个是一阶滞后滤波器
    register float z1;// 算法公式  Y(n) = a*X(n) + (1-a)Y(n-1) 其中 a 为滤波系数，代表权重；Xn 为本次采样值，Yn为滤波输出值,其中 a = dt/(T+dt),T为时间常数，当dt很小时，a = dt/T
    // z1为 Yn； signal 为本次采样值Xn， f->tc为滤波系数a,tc越大，说明本次采样权重越高
    z1 = f->z1 + (signal - f->z1) * f->tc; 

    f->z1 = z1;

    return z1;
}

float utilFilter3(utilFilter_t *f, float signal) {
    return utilFilter(&f[0], utilFilter(&f[1], utilFilter(&f[2], signal)));
}

float utilFirFilter(utilFirFilter_t *f, float newValue) {
    float result = 0.0f;
    int i;

    f->data[f->i] = newValue;
    f->i = (f->i + 1) % f->n;

    for (i = 0; i < f->n; i++)
        result += f->window[i] * f->data[(f->i + i) % f->n];

    return result;
}

void utilFirFilterInit(utilFirFilter_t *f, const float *window, float *buffer, uint8_t n) {
    int i;

    f->window = window;
    f->data = buffer;
    f->n = n;
    f->i = 0;

    for (i = 0; i < n; i++)
        f->data[i] = 0.0f;
}
// floating point numbers to ascii 为什么不直接用sprintf函数呢？
int ftoa(char *buf, float f, unsigned int digits) {
    int index = 0;
    int exponent;
    long multiplier, whole, part;
    float g;
    char format[16];

    // handle sign
    if (f < 0.0f) {
        buf[index++] = '-';
        f = -f;
    }

    // handle infinite values 无穷数
    if (isinf(f)) {
        strcpy(&buf[index], "INF");
        return 3;
    }
    // handle Not a Number
    else if (isnan(f)) {
        strcpy(&buf[index], "NaN");
        return 3;
    }
    else {
        // max digits
        if (digits > 6)
            digits = 6;
        multiplier = powf(10.0f, digits);     // fix int => long

        if (f > 0.0f)
            exponent = (int)log10f(f);
        else
            exponent = 0;

        g = f / powf(10.0f, exponent);
        if ((g < 1.0f) && (g != 0.0f)) {
            g *= 10.0f;
            exponent--;
        }

        whole = (long)(g);                     // single digit
        part = (long)((g-whole)*multiplier);   // # digits

        sprintf(format, "%%ld.%%0%dldE%%+.2d", digits);
        sprintf(&buf[index], format, whole, part, exponent);
        return strlen(buf);
    }
}


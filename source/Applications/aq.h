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

// NOTE: These parameters must be passed to GCC:
//
//  -fsingle-precision-constant
//

#ifndef _aq_h
#define _aq_h

#include "stm32f4xx.h"
#include "rcc.h"
#include "board_6_1a.h"
#include "board_dimu_v1_1.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "tick.h"
#include "ext_irq.h"
#include "queue.h"
#include "semphr.h"

#define FIMRWARE_VERSION "PL-V0.0"
#define BOARD_VERSION "F70-M V1.5"



#define USE_MAVLINK
#define USE_PRES_ALT		 	// uncomment to use pressure altitude instead of GPS
#define USE_SIGNALING                   // uncomment to use external signaling events and ports
//#define USE_QUATOS
//#define USE_EXTERNAL_ESC              // uncomment to use external ESCs on board version 8

//事件标志位定义
#define dIMU_EVENT_BIT	(1<<0)				
#define gpsVelFlag		(1<<1)
#define gpsPosFlag		(1<<2)
#define dRateFlag		(1<<3)
#define sensorFlag		(1<<4)
#define EVENTBIT_ALL	(dIMU_EVENT_BIT|gpsVelFlag|gpsPosFlag|dRateFlag|sensorFlag)


#ifndef M_PI
#define M_PI			3.14159265f
#define M_PI_2			(M_PI / 2.0f)
#endif

#define RAD_TO_DEG		(180.0f / M_PI)//弧度转化成角度
#define DEG_TO_RAD		(M_PI / 180.0f)

#define GRAVITY			9.80665f	// m/s^2

#define AQ_US_PER_SEC		1000000// 1秒=1000000微秒


#ifndef NAN
#define NAN	__float32_nan
#endif


#endif

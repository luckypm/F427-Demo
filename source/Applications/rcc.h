
#ifndef _rcc_h
#define _rcc_h

#define RTC_MASK_YEAR	(0b1111111<<25)
#define RTC_MASK_MONTH	(0b1111<<21)
#define RTC_MASK_DAY	(0b11111<<16)
#define RTC_MASK_HOUR	(0b11111<<11)
#define RTC_MASK_MINUTE	(0b111111<<5)
#define RTC_MASK_SECOND (0b11111)

extern RCC_ClocksTypeDef rccClocks;

extern void rccConfiguration(void);

#endif

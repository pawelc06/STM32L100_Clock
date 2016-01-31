#include "stm32l1xx_rtc.h"
#include "stm32l1xx_pwr.h"
#include "Lcd_Driver.h"
#include "clock.h"



volatile uint8_t day = 0;



void displayTime(){
    uint8_t rtc_time = 0;

    	RTC_TimeTypeDef  RTC_TimeStructure1;


		RTC_GetTime(RTC_Format_BCD, &RTC_TimeStructure1);

		rtc_time = RTC_TimeStructure1.RTC_Hours;

		LCD_Write_TimeBCD2(45, 0, &RTC_TimeStructure1);
		//LCD_Write_TimeBCD_On_Background(45, 20, &RTC_TimeStructure1);





}

void displayDate(){
	//uint8_t year, month, day, weekday;

	RTC_DateTypeDef  RTC_DateStruct;


		RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);


	day = RTC_DateStruct.RTC_Date;


	LCD_Write_Date(55,100,&RTC_DateStruct);

	//RTC_ITConfig(RTC_IT_WUT, ENABLE);

	//LCD_Write_TimeBCD(55, 20, &RTC_TimeStructure1);

}


void updateAndDisplayDate(){
	//uint8_t year, month, day, weekday;

	RTC_DateTypeDef  RTC_DateStruct;

	//RTC_ITConfig(RTC_IT_WUT, DISABLE);

	//RTC_DateStructInit(&RTC_DateStruct);

	//RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);

	/*
	if(day == 0)
		day = RTC_DateStruct.RTC_Date;
		*/

	do{
		RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);
	}
	while(RTC_DateStruct.RTC_Date == day);

	day = RTC_DateStruct.RTC_Date;


	LCD_Write_Date(55,100,&RTC_DateStruct);

	//RTC_ITConfig(RTC_IT_WUT, ENABLE);

	//LCD_Write_TimeBCD(55, 20, &RTC_TimeStructure1);

}

uint8_t bcd2dec(uint8_t numberbcd){
	uint8_t h = numberbcd >> 4;
	  // n - 6 * (n >> 4)
	  return numberbcd - (((h << 1) + h) << 1);

}

uint8_t dec2bcd(uint8_t numberdec){
	return (numberdec / 10 * 16 +  numberdec % 10);


}


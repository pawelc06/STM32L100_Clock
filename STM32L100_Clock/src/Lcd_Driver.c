#include "stm32l1xx.h"
#include "Lcd_Driver.h"
#include "LCD_Config.h"
#include "delay.h"
#include "GUI.h"
#include <string.h>
#include "clock.h"

extern bool updateDate;

/******************** (C) COPYRIGHT 2011 ÉÁŇ«µç×ÓÇ¶ČëĘ˝żŞ·˘ą¤×÷ĘŇ ********************
//ÉîŰÚTFTŇşľ§ÄŁ×éĹú·˘
//×¨×˘Ňşľ§Ĺú·˘
//Č«łĚĽĽĘőÖ§łÖ
//Tel:15989313508
//QQŁş573355510
//ĚÔ±¦ÍřµęµŘÖ·Łşmytft.taobao.com
//ČČłĎ»¶Ó­ÄúµÄąâÁŮ~Łˇ
/***************************************************************/
//Ňşľ§ĆÁĘýľÝŇý˝ĹËµĂ÷Łş
//   Ňşľ§ĆÁŇý˝Ĺ       ¶ÔÓ¦ą¦ÄÜ
//  Pin2		CS
//	Pin3		SCL
//	Pin4		SDI
//	Pin5		RS
//	Pin6		RESET	
/***************************************************************/

/***************************************************************/
//łĚĐň˝ÓĎßËµĂ÷Łş
//#define LCD_CTRL   		GPIOB		//¶¨ŇĺTFTĘýľÝ¶ËżÚ
//#define LCD_CS        	GPIO_Pin_11 //MCU_PB11 ¶ÔÓ¦--->>TFT --PIN_CS
//#define LCD_SCL        	GPIO_Pin_12	//MCU_PB12 ¶ÔÓ¦--->>TFT --SCL
//#define LCD_SDA        	GPIO_Pin_13	//MCU_PB13 ¶ÔÓ¦--->>TFT --SDA
//#define LCD_RS         	GPIO_Pin_14	//MCU_PB14 ¶ÔÓ¦--->>TFT --RS
//#define LCD_RST     		GPIO_Pin_15	//MCU_PB15 ¶ÔÓ¦--->>TFT --RST
/***************************************************************/

/***************************************************************/
//±ľ˛âĘÔłĚĐňą¦ÄÜËµĂ÷Łş
//1.Č«ĆÁĚîłä˛âĘÔŔýłĚ
//2.Ó˘ÎÄĎÔĘľ˛âĘÔŔýłĚ
//3.ÖĐÎÄĎÔĘľ˛âĘÔŔýłĚ
//4.2D°´ĹĄĎÔĘľ˛âĘÔŔýłĚ
//5.ĘýÂëąÜ×ÖĚĺĘý×ÖĎÔĘľ˛âĘÔŔýłĚ
//±¸×˘ŁşÓÉÓÚ±ľČËĘ±ĽäşÜĂ¦Ł¬Î´ĽÓČë´ĄĂţ˛âĘÔşÍÍĽĆ¬ĎÔĘľĘľŔýŁ¬ÓĐĐčŇŞµÄżÍ»§żÉŇÔÁŞĎµÎŇ

//¸řÓč·˘ËÍĆäËűŇşľ§ÄŁżéÉĎĂćËůÓĂµÄ´ĄĂţŔýłĚşÍÍĽĆ¬ĎÔĘľĘľŔý×÷ÎŞ˛ÎżĽŁ¬Đ»Đ»Ŕí˝âˇŁ
/***************************************************************/

//------------------------ioÄŁÄâspi˛ż·Ö---------------------------
// PB6-MOSI
#define SPIv_SetData(d) { if(d & 0x80) GPIO_SetBits(GPIOB,GPIO_Pin_15); else GPIO_ResetBits(GPIOB,GPIO_Pin_15);}
//PB7-MISO
#define SPIv_ReadData() GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)
// PE0-SCK
#define SPIv_SetClk()	GPIO_SetBits(GPIOB,GPIO_Pin_13)
#define SPIv_ClrClk()	GPIO_ResetBits(GPIOB,GPIO_Pin_13)

volatile u8 mode = 0;


//uint8_t Red, Green, Blue;
uint16_t Color;
uint16_t bkColor;

volatile uint16_t ssTogle;

//uint8_t Bk_red, Bk_green, Bk_blue;

volatile u16 last_hours, last_minutes, last_seconds;

volatile uint8_t last_year, last_month, last_day, last_weekday;


uint8_t Red, Green, Blue;
uint8_t Bk_red, Bk_green, Bk_blue;

void Set_color( uint8_t R, uint8_t G, uint8_t B  ) {
	uint32_t color24;

	color24 = (R << 16) | (G << 8) | B;
	Color = (uint16_t) LCD_RGB_24to16(color24);


}

void Set_bk_color( uint8_t R, uint8_t G, uint8_t B  ) {
	uint32_t color24;

		color24 = (R << 16) | (G << 8) | B;

		bkColor = (uint16_t) LCD_RGB_24to16(color24);
}

void Set_color32( uint32_t color ) {
	uint32_t color24;

	Red = (color>>16)&0xfe;
	Green = (color>>8)&0xfe;
	Blue = (color)&0xfe;

	color24 = (Red << 16) | (Green << 8) | Blue;
	Color = (uint16_t) LCD_RGB_24to16(color24);

}

void Set_bk_color32( uint32_t color ) {
	uint32_t color24;

	Bk_red = (color>>16)&0xfe;
	Bk_green = (color>>8)&0xfe;
	Bk_blue = (color)&0xfe;

	color24 = (Bk_red << 16) | (Bk_green << 8) | Bk_blue;
	bkColor = (uint16_t) LCD_RGB_24to16(color24);

}

void Draw_pixel() {

	//SPI_WriteByte(SPI2,(uint8_t)(Color>>8));
		//SPI_WriteByte(SPI2,(uint8_t)Color);



		Lcd_WriteData(Color>>8);
		Lcd_WriteData(Color);



}

void Draw_bk_pixel() {

	//SPI_WriteByte(SPI2,(uint8_t)(bkColor>>8));
	//SPI_WriteByte(SPI2,(uint8_t)bkColor);

	Lcd_WriteData(bkColor>>8);
	Lcd_WriteData(bkColor);
}

/*
void put_pixel(int x, int y) {
	y+=frame_ptr;
	Set_active_window(x, y, x, y);
	Write_command( 0x2c );
	Draw_pixel();
}

void put_bk_pixel(int x, int y) {
	y+=frame_ptr;
	Set_active_window(x, y, x, y);
	Write_command( 0x2c );
	Draw_bk_pixel();
}

*/

void tft_bitmap( int x, int y,  uint8_t * glyph, int width, int height  ) {

	uint32_t cred, cgreen, cblue;
	//uint16_t color;
	uint32_t color24;
		uint16_t idx=0;

		Lcd_SetRegion(x, y, x+width-1, y+height-1);

		while( idx < width*height*3 ) {
			cred =  glyph[ idx++ ] ;
			cgreen =  glyph[ idx++  ] ;
			cblue =  glyph[ idx++  ] ;

			color24 = (cred << 16) | (cgreen << 8) | cblue;
			//color = (uint16_t) LCD_RGB_24to16(color24);

			Set_color(cred, cgreen, cblue);
			Draw_pixel();
		}

	/*
	uint8_t cred, cgreen, cblue;
	uint16_t idx=0;
    y+=frame_ptr;
	Set_active_window(x, y, x+width-1, y+height-1);
	Write_command(0x2c);

	while( idx < width*height*3 ) {
		cred = pgm_read_byte( &glyph[ idx++ ] );
		cgreen = pgm_read_byte( &glyph[ idx++  ] );
		cblue = pgm_read_byte( &glyph[ idx++  ] );
		Set_color(cred, cgreen, cblue);
		Draw_pixel();
	}
	*/




}


u8 SPIv_WriteByte(u8 Byte)
{
	u8 i,Read;
	
	for(i=8; i; i--)
	{	
		SPIv_ClrClk();
		SPIv_SetData(Byte);	
		Byte<<=1;
		SPIv_SetClk();
		//Read <<= 1;
		//Read |= SPIv_ReadData();
	}
	//SPIv_ClrClk();
	return Read;
}

//ÓĂiożÚÄŁÄâµÄspiłőĘĽ»Ż
void SPIv_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	

	 /*************************************/
	 /*!< SD_SPI_CS_GPIO, SD_SPI_MOSI_GPIO, SD_SPI_MISO_GPIO, SD_SPI_DETECT_GPIO
	       and SD_SPI_SCK_GPIO Periph clock enable */
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB , ENABLE);

	  /*!< SD_SPI Periph clock enable */
	  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);


	  while(RCC_GetSYSCLKSource() != 0x0C);                //odczekaj az PLL bedzie sygnalem zegarowym systemu
	  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 , ENABLE);

	  /*!< Configure SD_SPI pins: SCK */
	  GPIO_InitStructure.GPIO_Pin = LCD_SCL;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	  GPIO_Init(LCD_CTRL, &GPIO_InitStructure);

	  /*!< Configure SD_SPI pins: MISO */
	  GPIO_InitStructure.GPIO_Pin = LCD_SDO;
	  GPIO_Init(LCD_CTRL, &GPIO_InitStructure);

	  /*!< Configure SD_SPI pins: MOSI */
	  GPIO_InitStructure.GPIO_Pin = LCD_SDA;
	  GPIO_Init(LCD_CTRL, &GPIO_InitStructure);

	  /*!< Configure SD_SPI_CS_PIN pin: SD Card CS pin */
	  GPIO_InitStructure.GPIO_Pin = LCD_CS;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	  GPIO_Init(LCD_CTRL, &GPIO_InitStructure);

	/*
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;  //PB7-MISO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);  

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;  //PB6-MOSI
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12| GPIO_Pin_10 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	*/

}




//spi Đ´Ň»¸ö×Ö˝Ú
u8 SPI_WriteByte(SPI_TypeDef* SPIx,u8 Byte)
{
	while((SPIx->SR&SPI_I2S_FLAG_TXE)==RESET);		//µČ´ý·˘ËÍÇřżŐ
	SPIx->DR=Byte;	 	//·˘ËÍŇ»¸öbyte
	while((SPIx->SR&SPI_I2S_FLAG_RXNE)==RESET);//µČ´ý˝ÓĘŐÍęŇ»¸öbyte
	return SPIx->DR;          	     //·µ»ŘĘŐµ˝µÄĘýľÝ
} 

//ÉčÖĂSPIµÄËŮ¶Č
//SpeedSet:1,¸ßËŮ;0,µÍËŮ;
void SPI_SetSpeed(SPI_TypeDef* SPIx,u8 SpeedSet)
{
	SPIx->CR1&=0XFFC7;
	if(SpeedSet==1)//¸ßËŮ
	{
		SPIx->CR1|=SPI_BaudRatePrescaler_8;//Fsck=Fpclk/2	
	}
	else//µÍËŮ
	{
		SPIx->CR1|=SPI_BaudRatePrescaler_32; //Fsck=Fpclk/32
	}
	SPIx->CR1|=1<<6; //SPIÉč±¸ĘąÄÜ
} 

void SPI2_Init(void)	
{

	SPI_InitTypeDef  SPI_InitStructure;

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

	    // GPIOB - SCK, MISO, MOSI
		GPIOB->MODER |= GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
		GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR13 | GPIO_OSPEEDER_OSPEEDR14 | GPIO_OSPEEDER_OSPEEDR15;
		GPIOB->AFR[1] = 0x55500000;

		// GPIOB - PB12( CS )
		GPIOB->MODER |= GPIO_MODER_MODER12_0;
		GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12;

		// init spi2
		RCC->APB1RSTR |= RCC_APB1RSTR_SPI2RST;
	    delay_ms( 15 );
	    RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI2RST;
	 
		/*
	    SPI2->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_BR_0;
	    SPI2->CR1 |= SPI_CR1_SPE;
	    */

	    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	    	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	    	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	    	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	    	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	    	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	    	//SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //8MHz
	    	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; // 16MHz
	    	//SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; //500 kHz
	    	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	    	SPI_InitStructure.SPI_CRCPolynomial = 7;
	    	SPI_Init(SPI2, &SPI_InitStructure);


	    	SPI_Cmd(SPI2, ENABLE);

	//uint16_t afr = GPIOB->AFR[1];
	//uint16_t cr1 = SPI2->CR1;

}

/****************************************************************************
* Ăű    łĆŁşvoid ili9220B_WriteIndex(u16 idx)
* ą¦    ÄÜŁşĐ´ ili9220B żŘÖĆĆ÷ĽÄ´ćĆ÷µŘÖ·
* ČëżÚ˛ÎĘýŁşidx   ĽÄ´ćĆ÷µŘÖ·
* łöżÚ˛ÎĘýŁşÎŢ
* Ëµ    Ă÷Łşµ÷ÓĂÇ°ĐčĎČŃˇÖĐżŘÖĆĆ÷Ł¬ÄÚ˛żşŻĘý
****************************************************************************/
void Lcd_WriteIndex(u8 Index)
{
   u8 i=0;
   //SPI Đ´ĂüÁîĘ±ĐňżŞĘĽ
   LCD_CS_CLR;
   LCD_RS_CLR;

   /*
   for (i = 0; i < 5; i++) {
   			__asm__("NOP");
   		}
*/


   SPI_WriteByte(SPI2,Index);
   
   LCD_CS_SET;
}

/****************************************************************************
* Ăű    łĆŁşvoid ili9220B_WriteData(u16 dat)
* ą¦    ÄÜŁşĐ´ ili9220B ĽÄ´ćĆ÷ĘýľÝ
* ČëżÚ˛ÎĘýŁşdat     ĽÄ´ćĆ÷ĘýľÝ
* łöżÚ˛ÎĘýŁşÎŢ
* Ëµ    Ă÷ŁşĎňżŘÖĆĆ÷Ö¸¶¨µŘÖ·Đ´ČëĘýľÝŁ¬µ÷ÓĂÇ°ĐčĎČĐ´ĽÄ´ćĆ÷µŘÖ·Ł¬ÄÚ˛żşŻĘý
****************************************************************************/
void Lcd_WriteData(u8 Data)
{
   u8 i=0;

   LCD_RS_SET;
   LCD_CS_CLR;

   /*
   for ( i = 0; i < 5; i++) {
   			__asm__("NOP");
   		}
*/

   //SPIv_WriteByte(Data);
   SPI_WriteByte(SPI2,Data);




   LCD_CS_SET;
}

void Lcd_WriteData16Bit(u8 DataH,u8 DataL)
{
	Lcd_WriteData(DataH);
	Lcd_WriteData(DataL);
}

void Lcd_WriteIndex16Bit(u8 DataH,u8 DataL)
{
	Lcd_WriteIndex(DataH);
	Lcd_WriteIndex(DataL);
}



void Lcd_Reset(void)
{
	LCD_RST_CLR;
	delay_ms(10);
	LCD_RST_SET;
	delay_ms(50);
}


void Lcd_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	char TFTDriver=0;
	int i=0;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	//LCD_RST_CLR
	/*!< Configure SD_SPI_CS_PIN pin: SD Card CS pin */
		  GPIO_InitStructure.GPIO_Pin = LCD_RST;
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
		  GPIO_Init(LCD_CTRL, &GPIO_InitStructure);

		  	  	  GPIO_InitStructure.GPIO_Pin = LCD_CS;
		  		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		  		  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  		  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		  		  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
		  		  GPIO_Init(LCD_CTRL, &GPIO_InitStructure);

		  		GPIO_InitStructure.GPIO_Pin = LCD_RS;
		  				  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		  				  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  				  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		  				  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
		  				  GPIO_Init(LCD_CTRL, &GPIO_InitStructure);

		  LCD_RST_SET;
		  LCD_CS_SET;
		  LCD_RS_SET;


	//SPIv_Init();
	SPI2_Init();
	Lcd_Reset();
	
	for(i=0;i<3;i++)
	    {
			//printf("Pętla: %d\r\n",i);
	        TFTDriver = readID();
	    }

	Lcd_WriteIndex(0xCB);  
        Lcd_WriteData(0x39); 
        Lcd_WriteData(0x2C); 
        Lcd_WriteData(0x00); 
        Lcd_WriteData(0x34); 
        Lcd_WriteData(0x02); 

        Lcd_WriteIndex(0xCF);  
        Lcd_WriteData(0x00); 
        Lcd_WriteData(0XC1); 
        Lcd_WriteData(0X30); 
 
        Lcd_WriteIndex(0xE8);  
        Lcd_WriteData(0x85); 
        Lcd_WriteData(0x00); 
        Lcd_WriteData(0x78); 
 
        Lcd_WriteIndex(0xEA);  
        Lcd_WriteData(0x00); 
        Lcd_WriteData(0x00); 
 
        Lcd_WriteIndex(0xED);  
        Lcd_WriteData(0x64); 
        Lcd_WriteData(0x03); 
        Lcd_WriteData(0X12); 
        Lcd_WriteData(0X81); 

        Lcd_WriteIndex(0xF7);  
        Lcd_WriteData(0x20); 
  
        Lcd_WriteIndex(0xC0);    //Power control 
        Lcd_WriteData(0x23);   //VRH[5:0] 
 
        Lcd_WriteIndex(0xC1);    //Power control 
        Lcd_WriteData(0x10);   //SAP[2:0];BT[3:0] 
 
        Lcd_WriteIndex(0xC5);    //VCM control 
        Lcd_WriteData(0x3e); //¶Ô±Č¶Čµ÷˝Ú
        Lcd_WriteData(0x28); 
 
        Lcd_WriteIndex(0xC7);    //VCM control2 
        Lcd_WriteData(0x86);  //--
 
        Lcd_WriteIndex(0x36);    // Memory Access Control 
        //Lcd_WriteData(0x48); //C8	   //48 68ĘúĆÁ//28 E8 şáĆÁ
        //Lcd_WriteData(MADCTL_MX | MADCTL_BGR);


        Lcd_WriteData(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);

        //Lcd_WriteData(0x28);

        //Lcd_WriteData(0xE8);

        Lcd_WriteIndex(0x3A);    
        Lcd_WriteData(0x55); 

        Lcd_WriteIndex(0xB1);    
        Lcd_WriteData(0x00);  
        Lcd_WriteData(0x18); 
 
        Lcd_WriteIndex(0xB6);    // Display Function Control 
        Lcd_WriteData(0x08); 
        Lcd_WriteData(0x82);
        Lcd_WriteData(0x27);  
 
        Lcd_WriteIndex(0xF2);    // 3Gamma Function Disable 
        Lcd_WriteData(0x00); 
 
        Lcd_WriteIndex(0x26);    //Gamma curve selected 
        Lcd_WriteData(0x01); 
 
        Lcd_WriteIndex(0xE0);    //Set Gamma 
        Lcd_WriteData(0x0F); 
        Lcd_WriteData(0x31); 
        Lcd_WriteData(0x2B); 
        Lcd_WriteData(0x0C); 
        Lcd_WriteData(0x0E); 
        Lcd_WriteData(0x08); 
        Lcd_WriteData(0x4E); 
        Lcd_WriteData(0xF1); 
        Lcd_WriteData(0x37); 
        Lcd_WriteData(0x07); 
        Lcd_WriteData(0x10); 
        Lcd_WriteData(0x03); 
        Lcd_WriteData(0x0E); 
        Lcd_WriteData(0x09); 
        Lcd_WriteData(0x00); 

        Lcd_WriteIndex(0XE1);    //Set Gamma 
        Lcd_WriteData(0x00); 
        Lcd_WriteData(0x0E); 
        Lcd_WriteData(0x14); 
        Lcd_WriteData(0x03); 
        Lcd_WriteData(0x11); 
        Lcd_WriteData(0x07); 
        Lcd_WriteData(0x31); 
        Lcd_WriteData(0xC1); 
        Lcd_WriteData(0x48); 
        Lcd_WriteData(0x08); 
        Lcd_WriteData(0x0F); 
        Lcd_WriteData(0x0C); 
        Lcd_WriteData(0x31); 
        Lcd_WriteData(0x36); 
        Lcd_WriteData(0x0F); 
 
        Lcd_WriteIndex(0x11);    //Exit Sleep 
        delay_ms(120); 
				
        Lcd_WriteIndex(0x29);    //Display on 
        Lcd_WriteIndex(0x2c); 
#if 0
	//************* Start Initial Sequence **********//
	
	Lcd_WriteIndex16Bit(0x00,0x01);
	Lcd_WriteData16Bit(0x01,0x1C); // set SS and NL bit
	Lcd_WriteIndex16Bit(0x00,0x02);
	Lcd_WriteData16Bit(0x01,0x00); // set 1 line inversion
	Lcd_WriteIndex16Bit(0x00,0x03);
	Lcd_WriteData16Bit(0x10,0x30); // set GRAM write direction and BGR=1.//1030
	Lcd_WriteIndex16Bit(0x00,0x08);
	Lcd_WriteData16Bit(0x08,0x08); // set BP and FP
	Lcd_WriteIndex16Bit(0x00,0x0C);
	Lcd_WriteData16Bit(0x00,0x00); // RGB interface setting R0Ch=0x0110 for RGB 18Bit and R0Ch=0111for RGB16Bit
	Lcd_WriteIndex16Bit(0x00,0x0F);
	Lcd_WriteData16Bit(0x0b,0x01); // Set frame rate//0b01
	Lcd_WriteIndex16Bit(0x00,0x20);
	Lcd_WriteData16Bit(0x00,0x00); // Set GRAM Address
	Lcd_WriteIndex16Bit(0x00,0x21);
	Lcd_WriteData16Bit(0x00,0x00); // Set GRAM Address
	//*************Power On sequence ****************//
	delay_ms(50);                         // Delay 50ms
	Lcd_WriteIndex16Bit(0x00,0x10);
	Lcd_WriteData16Bit(0x0a,0x00); // Set SAP,DSTB,STB//0800
	Lcd_WriteIndex16Bit(0x00,0x11);
	Lcd_WriteData16Bit(0x10,0x38); // Set APON,PON,AON,VCI1EN,VC
	delay_ms(50);                  // Delay 50ms
	Lcd_WriteIndex16Bit(0x00,0x12);
	Lcd_WriteData16Bit(0x11,0x21); // Internal reference voltage= Vci;
	Lcd_WriteIndex16Bit(0x00,0x13);
	Lcd_WriteData16Bit(0x00,0x63); // Set GVDD
	Lcd_WriteIndex16Bit(0x00,0x14);
	Lcd_WriteData16Bit(0x4b,0x44); // Set VCOMH/VCOML voltage//3944
	//------------- Set GRAM area ------------------//
	Lcd_WriteIndex16Bit(0x00,0x30);
	Lcd_WriteData16Bit(0x00,0x00);
	Lcd_WriteIndex16Bit(0x00,0x31);
	Lcd_WriteData16Bit(0x00,0xDB);
	Lcd_WriteIndex16Bit(0x00,0x32);
	Lcd_WriteData16Bit(0x00,0x00);
	Lcd_WriteIndex16Bit(0x00,0x33);
	Lcd_WriteData16Bit(0x00,0x00);
	Lcd_WriteIndex16Bit(0x00,0x34);
	Lcd_WriteData16Bit(0x00,0xDB);
	Lcd_WriteIndex16Bit(0x00,0x35);
	Lcd_WriteData16Bit(0x00,0x00);
	Lcd_WriteIndex16Bit(0x00,0x36);
	Lcd_WriteData16Bit(0x00,0xAF);
	Lcd_WriteIndex16Bit(0x00,0x37);
	Lcd_WriteData16Bit(0x00,0x00);
	Lcd_WriteIndex16Bit(0x00,0x38);
	Lcd_WriteData16Bit(0x00,0xDB);
	Lcd_WriteIndex16Bit(0x00,0x39);
	Lcd_WriteData16Bit(0x00,0x00);
	// ----------- Adjust the Gamma Curve ----------//
	Lcd_WriteIndex16Bit(0x00,0x50);
	Lcd_WriteData16Bit(0x00,0x03);
	Lcd_WriteIndex16Bit(0x00,0x51);
	Lcd_WriteData16Bit(0x09,0x00);
	Lcd_WriteIndex16Bit(0x00,0x52);
	Lcd_WriteData16Bit(0x0d,0x05);
	Lcd_WriteIndex16Bit(0x00,0x53);
	Lcd_WriteData16Bit(0x09,0x00);
	Lcd_WriteIndex16Bit(0x00,0x54);
	Lcd_WriteData16Bit(0x04,0x07);
	Lcd_WriteIndex16Bit(0x00,0x55);
	Lcd_WriteData16Bit(0x05,0x02);
	Lcd_WriteIndex16Bit(0x00,0x56);
	Lcd_WriteData16Bit(0x00,0x00);
	Lcd_WriteIndex16Bit(0x00,0x57);
	Lcd_WriteData16Bit(0x00,0x05);
	Lcd_WriteIndex16Bit(0x00,0x58);
	Lcd_WriteData16Bit(0x17,0x00);
	Lcd_WriteIndex16Bit(0x00,0x59);
	Lcd_WriteData16Bit(0x00,0x1F);
	delay_ms(50);                    // Delay 50ms
	Lcd_WriteIndex16Bit(0x00,0x07);
	Lcd_WriteData16Bit(0x10,0x17);
	Lcd_WriteIndex16Bit(0x00,0x22);		
	delay_ms(200);
#endif

}

void Lcd_Init2(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	char TFTDriver=0;
	int i=0;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	//LCD_RST_CLR
	/*!< Configure SD_SPI_CS_PIN pin: SD Card CS pin */
		  GPIO_InitStructure.GPIO_Pin = LCD_RST;
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
		  GPIO_Init(LCD_CTRL, &GPIO_InitStructure);

		  	  	  GPIO_InitStructure.GPIO_Pin = LCD_CS;
		  		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		  		  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  		  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		  		  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
		  		  GPIO_Init(LCD_CTRL, &GPIO_InitStructure);

		  		GPIO_InitStructure.GPIO_Pin = LCD_RS;
		  				  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		  				  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  				  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		  				  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
		  				  GPIO_Init(LCD_CTRL, &GPIO_InitStructure);

		  LCD_RST_SET;
		  LCD_CS_SET;
		  LCD_RS_SET;


	//SPIv_Init();
	SPI2_Init();
	Lcd_Reset();

	for(i=0;i<3;i++)
	    {
			//printf("Pętla: %d\r\n",i);
	        TFTDriver = readID();
	    }

	Lcd_WriteIndex(0xEF);
	  Lcd_WriteData(0x03);
	  Lcd_WriteData(0x80);
	  Lcd_WriteData(0x02);

	  Lcd_WriteIndex(0xCF);
	  Lcd_WriteData(0x00);
	  Lcd_WriteData(0XC1);
	  Lcd_WriteData(0X30);

	  Lcd_WriteIndex(0xED);
	  Lcd_WriteData(0x64);
	  Lcd_WriteData(0x03);
	  Lcd_WriteData(0X12);
	  Lcd_WriteData(0X81);

	  Lcd_WriteIndex(0xE8);
	  Lcd_WriteData(0x85);
	  Lcd_WriteData(0x00);
	  Lcd_WriteData(0x78);

	  Lcd_WriteIndex(0xCB);
	  Lcd_WriteData(0x39);
	  Lcd_WriteData(0x2C);
	  Lcd_WriteData(0x00);
	  Lcd_WriteData(0x34);
	  Lcd_WriteData(0x02);

	  Lcd_WriteIndex(0xF7);
	  Lcd_WriteData(0x20);

	  Lcd_WriteIndex(0xEA);
	  Lcd_WriteData(0x00);
	  Lcd_WriteData(0x00);

	  Lcd_WriteIndex(ILI9341_PWCTR1); //Power control
	  Lcd_WriteData(0x23); //VRH[5:0]

	  Lcd_WriteIndex(ILI9341_PWCTR2); //Power control
	  Lcd_WriteData(0x10); //SAP[2:0];BT[3:0]

	  Lcd_WriteIndex(ILI9341_VMCTR1); //VCM control
	  Lcd_WriteData(0x3e); //¶Ô±È¶Èµ÷½Ú
	  Lcd_WriteData(0x28);

	  Lcd_WriteIndex(ILI9341_VMCTR2); //VCM control2
	  Lcd_WriteData(0x86); //--

	  Lcd_WriteIndex(ILI9341_MADCTL); // Memory Access Control
	  //Lcd_WriteData(0x48);
	  Lcd_WriteData(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);

	  Lcd_WriteIndex(ILI9341_PIXFMT);
	  Lcd_WriteData(0x55);



	  Lcd_WriteIndex(ILI9341_FRMCTR1);
	  Lcd_WriteData(0x00);
	  Lcd_WriteData(0x18);

	  Lcd_WriteIndex(ILI9341_DFUNCTR); // Display Function Control
	  Lcd_WriteData(0x08);
	  Lcd_WriteData(0x82);
	  Lcd_WriteData(0x27);






	  Lcd_WriteIndex(0xF2); // 3Gamma Function Disable
	  Lcd_WriteData(0x00);

	  Lcd_WriteIndex(ILI9341_GAMMASET); //Gamma curve selected
	  Lcd_WriteData(0x01);

	  Lcd_WriteIndex(ILI9341_GMCTRP1); //Set Gamma
	  Lcd_WriteData(0x0F);
	  Lcd_WriteData(0x31);
	  Lcd_WriteData(0x2B);
	  Lcd_WriteData(0x0C);
	  Lcd_WriteData(0x0E);
	  Lcd_WriteData(0x08);
	  Lcd_WriteData(0x4E);
	  Lcd_WriteData(0xF1);
	  Lcd_WriteData(0x37);
	  Lcd_WriteData(0x07);
	  Lcd_WriteData(0x10);
	  Lcd_WriteData(0x03);
	  Lcd_WriteData(0x0E);
	  Lcd_WriteData(0x09);
	  Lcd_WriteData(0x00);

	  Lcd_WriteIndex(ILI9341_GMCTRN1); //Set Gamma
	  Lcd_WriteData(0x00);
	  Lcd_WriteData(0x0E);
	  Lcd_WriteData(0x14);
	  Lcd_WriteData(0x03);
	  Lcd_WriteData(0x11);
	  Lcd_WriteData(0x07);
	  Lcd_WriteData(0x31);
	  Lcd_WriteData(0xC1);
	  Lcd_WriteData(0x48);
	  Lcd_WriteData(0x08);
	  Lcd_WriteData(0x0F);
	  Lcd_WriteData(0x0C);
	  Lcd_WriteData(0x31);
	  Lcd_WriteData(0x36);
	  Lcd_WriteData(0x0F);

	  Lcd_WriteIndex(ILI9341_SLPOUT); //Exit Sleep
	  delay_ms(120);
	  Lcd_WriteIndex(ILI9341_DISPON); //Display on


}


/*************************************************
şŻĘýĂűŁşLCD_Set_Region
ą¦ÄÜŁşÉčÖĂlcdĎÔĘľÇřÓňŁ¬ÔÚ´ËÇřÓňĐ´µăĘýľÝ×Ô¶Ż»»ĐĐ
ČëżÚ˛ÎĘýŁşxyĆđµăşÍÖŐµă,Y_IncMode±íĘľĎČ×ÔÔöyÔŮ×ÔÔöx
·µ»ŘÖµŁşÎŢ
*************************************************/
void Lcd_SetRegion(u16 x_start,u16 y_start,u16 x_end,u16 y_end)
{	
	Lcd_WriteIndex(0x2a); //column address set
	Lcd_WriteData16Bit(x_start>>8,x_start);
	Lcd_WriteIndex(0x2b); // page address set
	Lcd_WriteData16Bit(y_start>>8,y_start);
	Lcd_WriteIndex(0x2c); //memory write
	/*	
	Lcd_WriteIndex(0x50);
	Lcd_WriteData16Bit(x_start>>8,x_start);
	Lcd_WriteIndex(0x51);
	Lcd_WriteData16Bit(x_end>>8,x_end);

	Lcd_WriteIndex(0x52);
	Lcd_WriteData16Bit(y_start>>8,y_start);
	Lcd_WriteIndex(0x53);
	Lcd_WriteData16Bit(y_end>>8,y_end);


	Lcd_WriteIndex(0x22); */

}

void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {

	Lcd_WriteIndex(ILI9341_CASET); // Column addr set
	Lcd_WriteData(x0 >> 8);
	Lcd_WriteData(x0 & 0xFF); // XSTART
	Lcd_WriteData(x1 >> 8);
	Lcd_WriteData(x1 & 0xFF); // XEND

  Lcd_WriteIndex(ILI9341_PASET); // Row addr set
  Lcd_WriteData(y0>>8);
  Lcd_WriteData(y0); // YSTART
  Lcd_WriteData(y1>>8);
  Lcd_WriteData(y1); // YEND

  Lcd_WriteIndex(ILI9341_RAMWR); // write to RAM
}

/*************************************************
şŻĘýĂűŁşLCD_Set_XY
ą¦ÄÜŁşÉčÖĂlcdĎÔĘľĆđĘĽµă
ČëżÚ˛ÎĘýŁşxy×ř±ę
·µ»ŘÖµŁşÎŢ
*************************************************/
void Lcd_SetXY(u16 x,u16 y)
{
  	Lcd_WriteIndex(0x2a);
	Lcd_WriteData16Bit(x>>8,x);
	Lcd_WriteIndex(0x2b);
	Lcd_WriteData16Bit(y>>8,y);

	Lcd_WriteIndex(0x2c);
}

	
/*************************************************
şŻĘýĂűŁşLCD_DrawPoint
ą¦ÄÜŁş»­Ň»¸öµă
ČëżÚ˛ÎĘýŁşÎŢ
·µ»ŘÖµŁşÎŢ
*************************************************/
void Gui_DrawPoint(u16 x,u16 y,u16 Data)
{
	Lcd_SetRegion(x,y,x,y);
	Lcd_WriteData(Data>>8);
	Lcd_WriteData(Data);

}    

/*****************************************
 şŻĘýą¦ÄÜŁş¶ÁTFTÄłŇ»µăµÄŃŐÉ«
 łöżÚ˛ÎĘýŁşcolor  µăŃŐÉ«Öµ
******************************************/
unsigned int Lcd_ReadPoint(u16 x,u16 y)
{
  unsigned int Data=0;
  Lcd_SetXY(x,y);

  //Lcd_ReadData();//¶ŞµôÎŢÓĂ×Ö˝Ú
  //Data=Lcd_ReadData();
  Lcd_WriteData(Data);
  return Data;
}
/*************************************************
şŻĘýĂűŁşLcd_Clear
ą¦ÄÜŁşČ«ĆÁÇĺĆÁşŻĘý
ČëżÚ˛ÎĘýŁşĚîłäŃŐÉ«COLOR
·µ»ŘÖµŁşÎŢ
*************************************************/
void Lcd_Clear(u16 Color)               
{	
   unsigned int i,m;
   Lcd_SetRegion(0,0,X_MAX_PIXEL-1+200,Y_MAX_PIXEL-1+200);
   LCD_CS_CLR;
   //LCD_RS_SET;
   LCD_RS_SET;
   //SPIv_WriteByte(Data);

   
   for(i=0;i<Y_MAX_PIXEL+200;i++)
   {
    for(m=0;m<X_MAX_PIXEL+200;m++)
      {	 
	  	//SPIv_WriteByte(Color>>8);  
		//SPIv_WriteByte(Color);
		SPI_WriteByte(SPI2,Color>>8);
		SPI_WriteByte(SPI2,Color);
	  	//Lcd_WriteData16Bit(Color>>8,Color);
		//Lcd_WriteData(Color>>8);
		//Lcd_WriteData(Color);
      }   
	}
	 LCD_CS_SET;
}

void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {

		  // rudimentary clipping (drawChar w/big text requires this)
		  if((x >= X_MAX_PIXEL) || (y >= Y_MAX_PIXEL)) return;
		  if((x + w - 1) >= X_MAX_PIXEL) w = X_MAX_PIXEL - x;
		  if((y + h - 1) >= Y_MAX_PIXEL) h = Y_MAX_PIXEL - y;

		  setAddrWindow(x, y, x+w-1, y+h-1);

		  uint8_t hi = color >> 8, lo = color;

		  //*dcport |= dcpinmask;
		  //digitalWrite(_dc, HIGH);
		  //*csport &= ~cspinmask;
		  //digitalWrite(_cs, LOW);

		  LCD_CS_CLR;
		  LCD_RS_SET;

		  for(y=h; y>0; y--) {
		    for(x=w; x>0; x--) {
		    	SPI_WriteByte(SPI2,hi);
		    	SPI_WriteByte(SPI2,lo);
		    }
		  }
		  //digitalWrite(_cs, HIGH);
		  //*csport |= cspinmask;
		  LCD_CS_SET;
		}

void Lcd_Clear2(u16 Color)
{
	fillRect(0, 0,  X_MAX_PIXEL, Y_MAX_PIXEL, Color);
}

char readID(void)
{
    char i=0;
    uint8_t data[3] ;
    uint8_t ID[3] = {0x00, 0x93, 0x41};
    char ToF=1;
    for(i=0;i<3;i++)
    {
		//rintf("Read reg 1\r\n");
        data[i]=Read_Register(0xd3,i+1);
        if(data[i] != ID[i])
        {
            ToF=0;
        }
        //printf("Read reg 2\r\n");
    }
    if(!ToF) /* data!=ID */
    {
        //printf("\n\rRead TFT ID failed, ID should be 0x09341, but read ID = 0x");
        for(i=0;i<3;i++)
        {
            //printf("Dane : %d - %d " , i , data[i] );
        }
        //Serial.println();
    } else
		//printf("TFT Found\n\r");
    	;
    return ToF;
}

char Read_Register(char Addr, char xParameter)
{
    char data=0;
    uint8_t tt = 0;
    int i;
    //sendCMD(0xd9); /* ext command */
    Lcd_WriteIndex(0xd9);

    //WRITE_DATA(0x10+xParameter); /* 0x11 is the first Parameter */
    Lcd_WriteData(0x10+xParameter);

    //TFT_DC_LOW;
    LCD_RS_CLR;

    //TFT_CS_LOW;
    LCD_CS_CLR;

    for (i = 0; i < 5; i++) { /* Wait a bit. */
			__asm__("NOP");
		}
    //tt = spi_xfer(SPI2, Addr)

    tt = SPI_WriteByte(SPI2, Addr);

		//while (!(SPI_SR(SPI2) & SPI_SR_RXNE));
    //TFT_DC_HIGH;
		LCD_RS_SET;

    for ( i = 0; i < 2; i++) { /* Wait a bit. */
			__asm__("NOP");
		}
    //tt =  SPI_DR(SPI2);
    tt = SPI2->DR;

    tt = spi_readwrite(SPI2, 0);
    //tt = SPI_WriteByte(SPI2, 0);

    //TFT_CS_HIGH;
    LCD_CS_SET;

    return tt;
}

static uint8_t spi_readwrite(uint32_t spi, uint8_t data)
{
	//while (SPI_SR(spi) & SPI_SR_BSY);
	while (SPI2->SR & SPI_SR_BSY);

	//SPI_DR(spi) = data;
	SPI2->DR = data;

	while (!(SPI2->SR & SPI_SR_RXNE));
	return SPI2->DR;
}

/**
* Konwersja Int na String
* x = input integer, s = output buffer
*/
void itoa(uint16_t n, uint8_t s[]){
	int i, sign;
	if ((sign = n) < 0) /* record sign */
	n = -n; /* make n positive */
	i = 0;
	do { /* generate digits in reverse order */
		s[i++] = n % 10 + '0'; /* get next digit */
	} while ((n /= 10) > 0); /* delete it */

	if (sign < 0)
	s[i++] = '-';

	if(i<2)
		s[i++] = '0';

	/* ending of the string */
	s[i] = 0;
	reverse(s);
}

/**
* odwrócenie stringów
* s = string
*/
void reverse(int8_t s[])
{
int i, j;
char c;

for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
		 c = s[i];
s[i] = s[j];
s[j] = c;
}
}

void LCD_Write_Colon(u16 xpos,u16 ypos){
	u16 short_break = 30;

	//colon
	//Gui_DrawFont_Num32(xpos,ypos,RED,GRAY0,11);
	tft_puts(xpos,ypos, ":", red, white);

	xpos = xpos+short_break;
}

void writeWeekDay(u16 xpos,u16 ypos,RTC_DateTypeDef  * RTC_DateStruct,uint16_t blink){

	if(!blink){
		tft_puts(xpos, ypos, "            ", white, black);

	} else {

	switch (RTC_DateStruct->RTC_WeekDay) {
	case 1:
			tft_puts(xpos, ypos, "poniedzialek", white, black);
			break;
		case 2:
			tft_puts(xpos, ypos, "wtorek      ", white, black);
			break;
		case 3:
			tft_puts(xpos, ypos, "sroda       ", white, black);
			break;
		case 4:
			tft_puts(xpos, ypos, "czwartek    ", white, black);
			break;
		case 5:
			tft_puts(xpos, ypos, "piatek      ", white, black);
			break;
		case 6:
			tft_puts(xpos, ypos, "sobota      ", white, black);
			break;
		case 7:
			tft_puts(xpos, ypos, "niedziela   ", white, black);
			break;
		default:
			tft_puts(xpos, ypos, "error       ", white, black);
			break;
	}
	}
}

void LCD_Write_Date(u16 xpos,u16 ypos,RTC_DateTypeDef  * RTC_DateStruct)
{
	uint8_t year, month, day, weekday;
	const u16 short_break = 22;
	uint8_t yearStr[5];
	uint8_t mStr[5];
	uint8_t dStr[5];

	uint8_t datas[9];
	u16 xposInit,blink;

	xposInit = xpos;

	year = RTC_DateStruct->RTC_Year;
	month = RTC_DateStruct->RTC_Month;
	day = RTC_DateStruct->RTC_Date;
	weekday = RTC_DateStruct->RTC_WeekDay;


	itoa((uint16_t) year + 2000, yearStr);
	itoa((uint16_t) month, mStr);
	itoa((uint16_t) day, dStr);

	setCurrentFont(&Verdana26ptFontInfo);
	//setCurrentFont( &DefaultFontInfo);

	blink = ((0x000F & ssTogle) % 2);

	switch (mode) {

	case 0:
	case 1:
	case 2:
		tft_puts(xpos, ypos, yearStr, white, black);
		xpos = xpos + 4 * short_break;

		tft_puts(xpos, ypos, "-", white, black);

		xpos = xpos + short_break;

		tft_puts(xpos, ypos, mStr, white, black);

		xpos = xpos + 2 * short_break;

		tft_puts(xpos, ypos, "-", white, black);

		xpos = xpos + short_break;

		tft_puts(xpos, ypos, dStr, white, black);

		ypos += 32;

		writeWeekDay(xposInit, ypos,RTC_DateStruct,1);

		break;

	case 3: //setting year

		if (blink) {
			tft_puts(xpos, ypos, yearStr, white, black);

		} else {
			tft_puts(xpos, ypos, "    ", white, black);
		}
		xpos = xpos + 4 * short_break;

		tft_puts(xpos, ypos, "-", white, black);

		xpos = xpos + short_break;

		tft_puts(xpos, ypos, mStr, white, black);

		xpos = xpos + 2 * short_break;

		tft_puts(xpos, ypos, "-", white, black);

		xpos = xpos + short_break;

		tft_puts(xpos, ypos, dStr, white, black);
		break;
	case 4: //setting month
		tft_puts(xpos, ypos, yearStr, white, black);
		xpos = xpos + 4 * short_break;



		tft_puts(xpos, ypos, "-", white, black);

		xpos = xpos + short_break;

		if (blink) {
			tft_puts(xpos, ypos, mStr, white, black);

		} else {
			tft_puts(xpos, ypos, "  ", white, black);
		}

		xpos = xpos + 2 * short_break;

		tft_puts(xpos, ypos, "-", white, black);

		xpos = xpos + short_break;

		tft_puts(xpos, ypos, dStr, white, black);
		break;
	case 5: //setting day
		tft_puts(xpos, ypos, yearStr, white, black);
		xpos = xpos + 4 * short_break;

		tft_puts(xpos, ypos, "-", white, black);

		xpos = xpos + short_break;

		tft_puts(xpos, ypos, mStr, white, black);

		xpos = xpos + 2 * short_break;

		tft_puts(xpos, ypos, "-", white, black);

		xpos = xpos + short_break;

		if (blink) {
			tft_puts(xpos, ypos, dStr, white, black);

		} else {
			tft_puts(xpos, ypos, "  ", white, black);
		}

		break;

	case 6: //setting weekday
		//tft_puts(xpos, ypos, yearStr, white, black);
				xpos = xpos + 4 * short_break;

				//tft_puts(xpos, ypos, "-", white, black);

				xpos = xpos + short_break;

				//tft_puts(xpos, ypos, mStr, white, black);

				xpos = xpos + 2 * short_break;

				//tft_puts(xpos, ypos, "-", white, black);

				xpos = xpos + short_break;

				//tft_puts(xpos, ypos, dStr, white, black);

				ypos += 32;

				writeWeekDay(xposInit, ypos,RTC_DateStruct,blink);

			break;
	default:
		break;

	}



}

void LCD_Write_TimeBCD2(u16 xpos,u16 ypos,RTC_TimeTypeDef * RTC_TimeStructure1)
{

	unsigned char datas[6];
	unsigned char datah[3];
	unsigned char datam[3];

	unsigned colon[2];

	uint8_t short_break = 47;

	u16 hh = 14;
	u16 mm = 33;
	u16 ss = 55;
	uint32_t color = white;
	uint32_t bkColor = black;
	uint8_t year, month, day, weekday;
	u16 blink;

	hh = RTC_TimeStructure1->RTC_Hours;
	mm = RTC_TimeStructure1->RTC_Minutes;
	ss = RTC_TimeStructure1->RTC_Seconds;



	blink = ((0x000F & ssTogle) % 2);







	//hours

	datah[0] = datas[0] = (hh >> 4) + 48;
	datah[1] = datas[1] = (0x000F & hh) + 48;

	datah[2] = 0;

	if (blink) {
		colon[0] = ':';
	} else {
		colon[0] = ' ';
	}

	colon[1] = 0;

	datam[0] = datas[3] = (mm >> 4) + 48;
	datam[1] = datas[4] = (0x000F & mm) + 48;
	datas[5] = 0;

	datam[2] = 0;

	setCurrentFont(&LetsgoDigital60ptFontInfo);

	switch (mode) {
	case 0:
		if (last_hours != hh){
			tft_puts(xpos, ypos, datah, color, bkColor);

			//if((last_hours == 23) && (hh==0))


		}

		xpos = xpos + 2 * short_break;
		tft_puts(xpos, ypos, colon, color, bkColor);
		xpos = xpos + short_break;

		if (last_minutes != mm)
			tft_puts(xpos, ypos, datam, color, bkColor);

		last_hours = hh;
		last_minutes = mm;

		break;
	case 1:
		if (blink) {
			//Gui_DrawFont_Num32(xpos,ypos,RED,GRAY0,15);
			tft_puts(xpos, ypos, "  ", color, bkColor);

			xpos = xpos + 2 * short_break;

		} else {

			tft_puts(xpos, ypos, datah, color, bkColor);

			xpos = xpos + 2 * short_break;

		}
		tft_puts(xpos, ypos, colon, color, bkColor);

		xpos = xpos + short_break;
		tft_puts(xpos, ypos, datam, color, bkColor);
		break;
	case 2:
		tft_puts(xpos, ypos, datah, color, bkColor);

		xpos = xpos + 2 * short_break;
		tft_puts(xpos, ypos, colon, color, bkColor);

		xpos = xpos + short_break;
		if (blink) {
			//Gui_DrawFont_Num32(xpos,ypos,RED,GRAY0,15);

			tft_puts(xpos, ypos, "  ", color, bkColor);

		} else {
			//Gui_DrawFont_Num32(xpos,ypos,RED,GRAY0,datas[0]);

			tft_puts(xpos, ypos, datam, color, bkColor);

			//xpos = xpos+short_break;

			//Gui_DrawFont_Num32(xpos,ypos,RED,GRAY0,datas[1]);
			//tft_puts(xpos,ypos, datas[1], red, bkColor);
			//xpos = xpos+4*short_break;

		}
		break;
	case 3:
	case 4:
	case 5:
	case 6:
		if (last_hours != hh){
					tft_puts(xpos, ypos, datah, color, bkColor);

					//if((last_hours == 23) && (hh==0))


				}

				xpos = xpos + 2 * short_break;
				tft_puts(xpos, ypos, colon, color, bkColor);
				xpos = xpos + short_break;

				if (last_minutes != mm)
					tft_puts(xpos, ypos, datam, color, bkColor);

				last_hours = hh;
				last_minutes = mm;


		displayDate();
		break;
	default:
		break;

	}


	if(!hh && !mm && !ss)
				updateDate = true;


}


void LCD_Write_TimeBCD_On_Background(u16 xpos,u16 ypos,RTC_TimeTypeDef * RTC_TimeStructure1)
{

	unsigned char datas[6];
	unsigned char datah[3];
	unsigned char datam[3];

	unsigned colon[2];

	uint8_t short_break=47;

	u16 hh=14;
	u16 mm=33;
	u16 ss=55;


	hh = RTC_TimeStructure1->RTC_Hours;
	mm = RTC_TimeStructure1->RTC_Minutes;
	ss = RTC_TimeStructure1->RTC_Seconds;

	//hours

	datah[0] = datas[0] = (hh >> 4)+48;
	datah[1] = datas[1] = (0x000F & hh)+48;

	datah[2] = 0;

	if(((0x000F & ss)%2)){
		colon[0] = ':';
	}	else {
		colon[0] = ' ';
	}

	colon[1] = 0;



	datam[0] = datas[3] = (mm >> 4)+48;
	datam[1] = datas[4] = (0x000F & mm)+48;
	datas[5] = 0;

	datam[2] = 0;


	           setCurrentFont( &LetsgoDigital60ptFontInfo );

	           if((mode == 1) && ((0x000F & ss)%2)){

	           				tft_puts_on_background(xpos,ypos, "  ", blue);


	           				xpos = xpos+2*short_break;

	           			} else {

	           				tft_puts_on_background(xpos,ypos, datah, blue);


	           				xpos = xpos+2*short_break;



	           			}

	           tft_puts_on_background(xpos,ypos, colon, blue);

	           xpos = xpos+short_break;

	           if((mode == 2) && ((0x000F & ss)%2)){

	           	           				tft_puts_on_background(xpos,ypos, "  ", blue);




	           	           			} else {

	           	           				tft_puts_on_background(xpos,ypos, datam, blue);




	           	           			}





}



void LCD_Write_TimeBCD(u16 xpos,u16 ypos,RTC_TimeTypeDef * RTC_TimeStructure1)
{

	unsigned char datas[6];


	u16 short_break = 30;
	u16 hh=14;
	u16 mm=33;
	u16 ss=55;


	hh = RTC_TimeStructure1->RTC_Hours;
	mm = RTC_TimeStructure1->RTC_Minutes;
	ss = RTC_TimeStructure1->RTC_Seconds;

	//hours

	datas[0] = (hh >> 4)+48;
	datas[1] = (0x000F & hh)+48;
	datas[2] = ':';
	datas[3] = (mm >> 4)+48;
	datas[4] = (0x000F & mm)+48;
	datas[5] = 0;

			mode = 0;

			if((mode == 1) && ((0x000F & ss)%2)){
				//Gui_DrawFont_Num32(xpos,ypos,RED,GRAY0,15);
				tft_puts(xpos,ypos, " ", red, white);

				xpos = xpos+short_break;

				//Gui_DrawFont_Num32(xpos,ypos,RED,GRAY0,15);
				tft_puts(xpos,ypos, " ", red, white);

				xpos = xpos+short_break;

			} else {
				//Gui_DrawFont_Num32(xpos,ypos,RED,GRAY0,datas[0]);
				tft_puts(xpos,ypos, datas, red, white);

				//xpos = xpos+short_break;

				//Gui_DrawFont_Num32(xpos,ypos,RED,GRAY0,datas[1]);
				//tft_puts(xpos,ypos, datas[1], red, white);
				xpos = xpos+short_break;



			}

				//colon
			    //Gui_DrawFont_Num32(xpos,ypos,RED,GRAY0,11);
				xpos = xpos+short_break;


					//minutes

				datas[0] = (mm >> 4);
				datas[1] = (0x000F & mm);

				if((mode == 2) && ((0x000F & ss)%2)){
					tft_puts(xpos,ypos, " ", red, white);

									xpos = xpos+short_break;

									//Gui_DrawFont_Num32(xpos,ypos,RED,GRAY0,15);
									tft_puts(xpos,ypos, " ", red, white);

									xpos = xpos+short_break;

								} else {
									//Gui_DrawFont_Num32(xpos,ypos,RED,GRAY0,datas[0]);
									tft_puts(xpos,ypos, 48+datas[0], red, white);

									xpos = xpos+short_break;

									//Gui_DrawFont_Num32(xpos,ypos,RED,GRAY0,datas[1]);
									tft_puts(xpos,ypos, 48+datas[1], red, white);
									xpos = xpos+short_break;


				}

				//colon
				//Gui_DrawFont_Num32(xpos,ypos,RED,GRAY0,11);
				xpos = xpos+short_break;

					//seconds

					datas[0] = (ss >> 4);
					datas[1] = (0x000F & ss);

					if((mode == 3) && ((0x000F & ss)%2)){
							Gui_DrawFont_Num32(xpos,ypos,RED,GRAY0,15);

							xpos = xpos+short_break;

							Gui_DrawFont_Num32(xpos,ypos,RED,GRAY0,15);

							xpos = xpos+short_break;

					} else {


							Gui_DrawFont_Num32(xpos,ypos,RED,GRAY0,datas[0]);
							xpos = xpos+short_break;

							Gui_DrawFont_Num32(xpos,ypos,RED,GRAY0,datas[1]);

					}
}

uint32_t LCD_RGB_24to16(uint32_t color)
{
uint32_t r, g, b, rgb;



rgb = ((color & 0xF80000)>>8) | ((color & 0xFC00)>>5) | ((color & 0xF8)>>3);

return( rgb );
}

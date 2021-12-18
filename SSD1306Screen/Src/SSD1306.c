/*
 * SSD1306.c
 *
 *      Author: Talha SEVİNÇ
 */

#include "SSD1306.h"
#include "stm32f4xx_hal.h"


SSD_Color screenColor=BLACK;
SSD1306_Position positions;

FontSpecification Font_7x10  ={  7,10,Font7x10 };
FontSpecification Font_11x18 ={ 11,18,Font11x18 };
FontSpecification Font_16x26 ={ 16,26,Font16x26 };

SSD1306_State screenState;


static uint8_t ScreenBuffer[SSD1306_Width * SSD1306_Height / 8];

void delay()
{
    uint16_t counter=0;
    while(counter++ < 4000);

}

void i2cWrite(uint8_t registerAddress,uint8_t *sendData,uint16_t dataLength)
{
     HAL_I2C_Mem_Write(&hi2c2, SSD1306Address, registerAddress, 1, &sendData, dataLength, 100);
}

void scrollActivate(uint8_t select)
{
    if(select == 1)
    	SSD1306_SendCommand(scrollActive,1);
    else
    	SSD1306_SendCommand(scrollDeactive,1);
}

void scrollActiveLeftRight(uint8_t startPage, uint8_t stopPage,uint8_t timeInterval)
{
    SSD1306_SendCommand(0x00,1);
    SSD1306_SendCommand(startPage,1);
    SSD1306_SendCommand(timeInterval,1); //0 – 5 frames    4 – 3 frames   1 – 64 frames    5 – 4 frames
    								     //2 – 128 frames  6 – 25 frame   3 – 256 frames   7 – 2 frames
    SSD1306_SendCommand(stopPage,1);
    SSD1306_SendCommand(0x00,1);
    SSD1306_SendCommand(0xFF,1);
}

uint8_t rightScrollActive(uint8_t startPage, uint8_t stopPage,uint8_t timeInterval)
{
	if(startPage>7 || stopPage>7 || timeInterval>7 )
		return 0;

    SSD1306_SendCommand(rightScroll,1);
    scrollActiveLeftRight(startPage,stopPage,timeInterval);
    scrollActivate(1);
    return 1;
}

uint8_t leftScrollActive(uint8_t startPage, uint8_t stopPage,uint8_t timeInterval)
{
	if(startPage>7 || stopPage>7 || timeInterval>7 )
		return 0;

    SSD1306_SendCommand(leftScroll,1);
    scrollActiveLeftRight(startPage,stopPage,timeInterval);
    scrollActivate(1);
    return 1;
}


void setVerticalScrollArea(uint8_t topArea,uint8_t totalArea)
{
	SSD1306_SendCommand(SetVerticalArea,1);
	SSD1306_SendCommand(topArea,1);   //0  ===>  RESET
	SSD1306_SendCommand(totalArea,1); //64 ===>  RESET

	/*
	  Notes in Datasheet:
			(1) A[5:0]+B[6:0] <= MUX ratio
			(2) B[6:0] <= MUX ratio
			(3a) Vertical scrolling offset (E[5:0] in 29h/2Ah) < B[6:0]
			(3b) Set Display Start Line (X5X4X3X2X1X0 of 40h~7Fh) < B[6:0]

			(4) The last row of the scroll area shifts to the first row of the scroll area.
			(5) For 64d MUX display
				A[5:0] = 0, B[6:0]=64 : whole area scrolls
				A[5:0]= 0, B[6:0] < 64 : top area scrolls
				A[5:0] + B[6:0] < 64 : central area scrolls
				A[5:0] + B[6:0] = 64 : bottom area scrolls

	 */

}

void scrollActiveVerticalHorizantal(uint8_t startPage, uint8_t stopPage,uint8_t timeInterval)
{
    SSD1306_SendCommand(0x00,1);
    SSD1306_SendCommand(startPage,1);
    SSD1306_SendCommand(timeInterval,1); //0 – 5 frames    4 – 3 frames   1 – 64 frames    5 – 4 frames
    								     //2 – 128 frames  6 – 25 frame   3 – 256 frames   7 – 2 frames
    SSD1306_SendCommand(stopPage,1);
    SSD1306_SendCommand(0x25,1);         //Scrolling Offset Value (0-63)

}

uint8_t verticalScrollRight(uint8_t startPage,uint8_t stopPage,uint8_t timeInterval)
{
	if(startPage>7 || stopPage>7 || timeInterval>7 )
		return 0;

	SSD1306_SendCommand(verticalRight,1);
	scrollActiveVerticalHorizantal(startPage,stopPage,timeInterval);
	scrollActivate(1);
	return 1;
}

uint8_t verticalScrollLeft(uint8_t startPage,uint8_t stopPage,uint8_t timeInterval)
{
	if(startPage>7 || stopPage>7 || timeInterval>7 )
		return 0;

	SSD1306_SendCommand(verticalLeft,1);
	scrollActiveVerticalHorizantal(startPage,stopPage,timeInterval);
	scrollActivate(1);
	return 1;
}


uint8_t SSD_Init()
{
	if (HAL_I2C_IsDeviceReady(&hi2c2, SSD1306Address, 1, 20000) != HAL_OK)
       return 0;
	delay();
	HAL_Delay(100);
	SSD1306_SendCommand(displayOff, 1);
	SSD1306_SendCommand(setAddresingMode, 1);
	SSD1306_SendCommand(0x10, 1);             // A[7:0]  ******A1A2   00:Horizontal 01:Verticilar 10:Page 11:Invalid
	SSD1306_SendCommand(pageStartAddr,1);
	SSD1306_SendCommand(0xC8,1);
	SSD1306_SendCommand(0x00,1);              // Start address This value can be 0x00-0x0F
	SSD1306_SendCommand(0x10,1);              // This value can be 0x10-0x1F
	SSD1306_SendCommand(0x40,1);              // Display start line. This value can be 0x40-0x7F

	SSD1306_SendCommand(0x81,1);              // Contrast control register
	SSD1306_SendCommand(0xFF,1);              // Contrast value

	SSD1306_SendCommand(0xA1,1);              //Set Segment Re-map (A0h/A1h)

	SSD1306_SendCommand(0xA6,1);              // Normal Display
	SSD1306_SendCommand(0xA8,1);              // Set multiplex ratio(1 to 64)
	SSD1306_SendCommand(0x3F,1);              // Decimal:64
	SSD1306_SendCommand(entireDisplayOn_1,1); // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	SSD1306_SendCommand(0xD3,1);              // Display offset
	SSD1306_SendCommand(0x00,1);              //-not offset
	SSD1306_SendCommand(0xD5,1);              // Display clock divide ratio/oscillator frequency
	SSD1306_SendCommand(0xF0,1);              // Set divide ratio
	SSD1306_SendCommand(0xD9,1);              // Set pre-charge period
	SSD1306_SendCommand(0x22,1);
	SSD1306_SendCommand(0xDA,1);              // Com pins hardware configuration
	SSD1306_SendCommand(0x12,1);
	SSD1306_SendCommand(0xDB,1);
	SSD1306_SendCommand(0x20,1);
	SSD1306_SendCommand(0x8D,1);
	SSD1306_SendCommand(0x14,1);
	SSD1306_SendCommand(displayOn,1);

	SSD1306_SendCommand(scrollDeactive,1);

    screenState.height=SSD1306_Height;
    screenState.width =SSD1306_Width;
    screenState.init  =true;
    screenState.font  =Font_7x10;
    screenState.arraySize= SSD1306_Width*SSD1306_Height/8;

    setWriteColor(BLACK);
    screenClear();
    positions.xPosition=0;
    positions.yPosition=0;


}

void setWriteColor(SSD_Color colorType)
{
	screenColor=colorType;
	screenClear();

}
void screenFill()
{	uint8_t m;
    uint8_t regAddr=0xB0;
    uint8_t addr;
	for (m = 0; m < 8; m++)
	{
		addr=regAddr+m;
		SSD1306_SendCommand(addr ,1);
		SSD1306_SendCommand(0x00 ,1);
		SSD1306_SendCommand(0x10 ,1);


		HAL_I2C_Mem_Write(&hi2c2, SSD1306Address, 0x40, 1, &ScreenBuffer[screenState.width*m], screenState.width, 100);
    }

}
void screenClear()
{
    if(screenColor == BLACK)
    	memset(ScreenBuffer,0xFF,screenState.arraySize);
    else
    	memset(ScreenBuffer,0x00,screenState.arraySize);

    screenFill();

}

void drawPixel(uint16_t xPos,uint16_t yPos,SSD_Color color)
{

    if (color == WHITE)
    {
        ScreenBuffer[xPos + (yPos / 8) * SSD1306_Width] |= 1 << (yPos % 8);
    }
    else
    {
    	ScreenBuffer[xPos + (yPos / 8) * SSD1306_Width] &= ~(1 << (yPos % 8));
    }


}

uint8_t charWrite(char ch,SSD_Color screenCol)
{
     if(SSD1306_Width< positions.xPosition || SSD1306_Height < positions.yPosition)  //Edge control
    	 return 0;

     uint16_t character;
     for(int i=0;i<screenState.font.Height;i++)
     {
    	 character=screenState.font.FontS[ ((uint8_t)ch-32)*screenState.font.Height+i ];
    	 for(int j=0;j<screenState.font.Width;j++)
    	 {
               if( (character<<j) & 0x8000 )
            	   drawPixel(positions.xPosition+j,positions.yPosition+i,(SSD_Color)screenColor);
               else
            	   drawPixel(positions.xPosition+j,positions.yPosition+i,(SSD_Color)!screenColor);

    	 }

     }

     positions.xPosition += screenState.font.Width;

     return 1;

}

void writeScreen(char *message,FontSpecification fontType,uint16_t xStart,uint16_t yStart)
{
	positions.xPosition=xStart;
	positions.yPosition=yStart;
	screenState.font=fontType;
	while(*message)
	{
		charWrite(*message,screenColor);
		*message++;

	}

	screenFill();


}




void DrawBitmap(int16_t xStart, int16_t yStart, const unsigned char* bitmap, int16_t w, int16_t h)
{

    int16_t byteWidth = (w + 7) / 8;
    uint8_t byte = 0;

    for(int16_t j=0; j<h; j++, yStart++)
    {
        for(int16_t i=0; i<w; i++)
        {
            if(i & 7)
            {
               byte <<= 1;
            }
            else
            {
               byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
            }
            if(byte & 0x80) drawPixel(xStart+i,yStart,screenColor);
        }
    }
    screenFill();
}




void SSD1306_SendCommand(uint8_t value, uint16_t length)
{
	HAL_I2C_Mem_Write(&hi2c2, SSD1306Address, 0x00, 1, &value, length, 100);
}

void SSD1306_SendData(uint8_t registerAddr, uint8_t value,uint16_t length)
{
	HAL_I2C_Mem_Write(&hi2c2, SSD1306Address, 0x40, 1, &value, length, 100);
}

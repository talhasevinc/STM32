/*
 * SSD1306.h
 *
 *  Created on: Oct 21, 2021
 *      Author: mypc
 */

#ifndef SSD1306_H_
#define SSD1306_H_

#include "stm32f4xx_hal.h"

/*
I2C Address of SSD1306. This value can be two different number according to where resistor is.
In my module, 0x78 and 0x7A are possible I2C addresses.
*/

#define SSD1306Address   0x78

#define SSD1306_Width     128
#define SSD1306_Height     64



/*      COMMANDS      */
#define displayOn         0xAF
#define displayOff        0xAE
#define contrastSet       0x81   //This command have to be 2 byte. After send command, you have to sent contrast value.
#define entireDisplayOn_1 0xA4   //Output follows RAM contents...
#define entireDisplayOn_2 0xA5   //Output ignores RAM contents...
#define normalDisplay     0xA6
#define inverseDisplay    0xA7
#define rightScroll       0x26
#define leftScroll        0x27
#define verticalRight     0x29
#define verticalLeft      0x30
#define scrollActive      0x2F
#define scrollDeactive    0x2E
#define SetVerticalArea   (uint8_t)0xA3
#define setAddresingMode  0x20
#define pageStartAddr     0xB0  //B0-B7 possible
#define ComScanDirect     0xC0
	/*C0h, X[3]=0b: normal mode (RESET) Scan from
	COM0 to COM[N –1]
	C8h, X[3]=1b: remapped mode. Scan from
	COM[N-1] to COM0
	Where N is the Multiplex ratio.*/



extern I2C_HandleTypeDef hi2c2;
extern uint16_t Font7x10 [];
extern uint16_t Font11x18 [];
extern uint16_t Font16x26 [];


typedef enum
{
   false = 0x00, true=0x01
}bool;

typedef struct
{
    uint16_t xPosition;
    uint16_t yPosition;

} SSD1306_Position;

typedef struct
{
	uint8_t Width;
	uint8_t Height;
	uint16_t *FontS;


}FontSpecification;

typedef struct
{
	uint16_t Length;
	uint16_t Height;
}FontSize;



typedef struct
{
    uint16_t height;
    uint16_t width;
    uint16_t arraySize;
    bool init;
    FontSpecification font;


} SSD1306_State;


typedef enum
{
   BLACK = 0x00, WHITE=0x01
}SSD_Color;


void delay();
void i2cWrite(uint8_t registerAddress,uint8_t *sendData,uint16_t dataLength);
void scrollActivate(uint8_t select);
void scrollActiveLeftRight(uint8_t startPage, uint8_t stopPage,uint8_t timeInterval);
uint8_t rightScrollActive(uint8_t startPage, uint8_t stopPage,uint8_t timeInterval);
uint8_t leftScrollActive(uint8_t startPage, uint8_t stopPage,uint8_t timeInterval);

void setVerticalScrollArea(uint8_t topArea,uint8_t totalArea);
void scrollActiveVerticalHorizantal(uint8_t startPage, uint8_t stopPage,uint8_t timeInterval);
uint8_t verticalScrollRight(uint8_t startPage,uint8_t stopPage,uint8_t timeInterval);
uint8_t verticalScrollLeft(uint8_t startPage,uint8_t stopPage,uint8_t timeInterval);


uint8_t SSD_Init();
void setWriteColor(SSD_Color colorType);
void screenFill();
void screenClear();
uint8_t charWrite(char ch,SSD_Color screenCol);
void writeScreen(char *message,FontSpecification fontType,uint16_t xStart,uint16_t yStart);
void DrawBitmap(int16_t xStart, int16_t yStart, const unsigned char* bitmap, int16_t w, int16_t h);
void SSD1306_SendCommand(uint8_t value, uint16_t length);
void SSD1306_SendData(uint8_t registerAddr, uint8_t value,uint16_t length);

#endif /* SSD1306_H_ */

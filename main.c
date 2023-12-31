/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC16F1705
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include "mcc_generated_files/mcc.h"
// Motor Direction
#define FWD 1
#define REV 0

#include <stdio.h>
#include <string.h>
#include "mcc_generated_files/examples/i2c_master_example.h"

/******* FLASH *******/
#define FLASH_ROWSIZE   32      //size of a row
#define HEFLASH_START   0x1F80  //first address in HE Flash memory
#define HEFLASH_END     0x1FFF  //last address in HE Flash memory

void _unlock (void){
    asm("BANKSEL PMCON2");
    asm("MOVLW  0x55");
    asm("MOVWF	PMCON2 & 0x7F");
    asm("MOVLW	0xAA");
    asm("MOVWF	PMCON2 & 0x7F");
    asm("BSF    PMCON1 & 0x7F, 1");
    asm("NOP");
    asm("NOP");
} // unlock
unsigned FLASH_readConfig (unsigned address){
    // 1. load the address pointers
    PMADR = address;
    PMCON1bits.CFGS = 1;    //select the configuration Flash address space
    PMCON1bits.RD = 1;  //next operation will be a read
    NOP();
    NOP();
    // 2. return value read
    return PMDAT; 
} // FLASH_config
unsigned FLASH_read (unsigned address) {
    // 1. load the address pointers
    PMADR = address;
    PMCON1bits.CFGS = 0;    //select the Flash address space
    PMCON1bits.RD = 1;      //next operation will be a read
    NOP();
    NOP();
    // 2. return value read
    return PMDAT; 
} // FLASH_read
void FLASH_readBlock (unsigned *buffer, unsigned address, char count) {
    while (count > 0)
    {
        *buffer++ = FLASH_read (address++);
        count--; 
    }
} // FLASH_readBLock
void FLASH_write (unsigned address, unsigned data, char latch) {
    // 1. disable interrupts (remember setting)
    char temp = INTCONbits.GIE;
    INTCONbits.GIE = 0;
    // 2. load the address pointers 
    PMADR = address;
    PMDAT = data;
    PMCON1bits.LWLO = latch;    //1 = latch, 0 = write row
    PMCON1bits.CFGS = 0;    //select the Flash address space
    PMCON1bits.FREE = 0;    //next operation will be a write
    PMCON1bits.WREN = 1;    //enable Flash memory write/erase
    // 3. perform unlock sequence
    _unlock();
    // 4. restore interrupts 
    if (temp)
       INTCONbits.GIE = 1;
}//FLASH_write
void FLASH_erase (unsigned address){
    // 1. disable interrupts (remember setting)
    char temp = INTCONbits.GIE;
    INTCONbits.GIE = 0;
    // 2. load the address pointers 
    PMADR = address;
    PMCON1bits.CFGS = 0;    // select the Flash address space
    PMCON1bits.FREE = 1;    // next operation will be an erase
    PMCON1bits.WREN = 1;    // enable Flash memory write/erase
    // 3. perform unlock sequence and erase 
    _unlock();
    // 4. disable writes and restore interrupts
    PMCON1bits.WREN = 0; // disable Flash memory write/erase 
    if (temp)
       INTCONbits.GIE = 1;
}//FLASH_erase
#define HEFLASH_MAXROWS (HEFLASH_END-HEFLASH_START+1)/FLASH_ROWSIZE
char HEFLASH_writeBlock (char radd, char* data, char count) {
    // 1. obtain absolute address in HE FLASH row 
    unsigned add = radd * FLASH_ROWSIZE + HEFLASH_START;
    // 2. check input parameters
    if ((count > FLASH_ROWSIZE)||(radd >= HEFLASH_MAXROWS))
          return -1;//return parameter error
    // 3. erase the entire row 
    FLASH_erase (add);
    // 4. fill the latches with data 
    while (count > 1)
    {
        //load data in latches without writing 
        FLASH_write (add++, (unsigned) *data++, 1); 
        count--;
    } 
    // no delay here!!!
    // 5. last byte of data -> write 
    FLASH_write (add, (unsigned) *data, 0); 
    // NOTE: 2ms typ. delay here!!!
    __delay_ms(2);
    // 6. return success
    return PMCON1bits.WRERR; //0 success, 1 = write error
} //HEFLASH_writeBlock
char HEFLASH_readBlock (char *buffer, char radd, char count) {
    // 1. obtain absolute address in HE FLASH row 
    unsigned add = radd * FLASH_ROWSIZE + HEFLASH_START;
    // 2. check input parameters
    if ((count > FLASH_ROWSIZE)||(radd >= HEFLASH_MAXROWS))
        return -1;
    // 3. read content 
    while (count > 0)
    {
        *buffer++ = (char) FLASH_read (add++);
        count--; 
    }
    // 4. success
    return 0;
} //HEFLASH_readBlock
char HEFLASH_readByte (char radd, char offset) {
    // 1. add offset into HE Flash memory
    unsigned add = radd * FLASH_ROWSIZE + HEFLASH_START + offset; 
    // 2. read content
    return (char) FLASH_read (add);
} //HEFLASH_read

/******** I2C ********/
/*
 *  Use MSSP I2C by MCC
 */

/******** I2C LCD ********/
#define LCD_ADDR 0x3E
bool LCD = true;//false;
char DisplayData[8]="12345678";

int is_I2C_Connected(uint8_t slave_address){
    int exist_the_lcd = 1;  // 0: exist the LCD, 1: No LCD
    
    if ( (RC0_GetValue() | RC1_GetValue()) != 0){ // if no pull-up
        if( I2C_Open(slave_address) == I2C_NOERR){ 
            exist_the_lcd = 0;
        } else {
            if( I2C_Open(slave_address) == I2C_FAIL ){
                exist_the_lcd = 1;
            } else {
                if( I2C_Open(slave_address) == I2C_BUSY ){
                    exist_the_lcd = 1;
                }
            }
        }
    }
    return exist_the_lcd;
}
void writeLCDCommand(char t_command){
    I2C_Write1ByteRegister(LCD_ADDR, 0x00, t_command );
    __delay_us(30);     //Instruction Execution Time 14.3-26.3us
}
void LCD_Init(){
    __delay_ms(400);
    writeLCDCommand(0x38);
    writeLCDCommand(0x39);
    writeLCDCommand(0x14);
    writeLCDCommand(0x76);// contast LSB setting ; 0b0111 xxxx
    writeLCDCommand(0x50);// 5V=0b0101 00xx, 3V=0b0101 01xx,  xx=contrast MSB
    writeLCDCommand(0x6C);
    __delay_ms(250);
    writeLCDCommand(0x38);
    writeLCDCommand(0x0C);
    writeLCDCommand(0x01);
    __delay_us(1100);        //Instruction Execution Time 0.59-1.08ms (550:NG, 600:GOOD)
}
void LCD_xy(uint8_t x, uint8_t y){
    writeLCDCommand(0x80 + 0x40 * y + x);
}
void LCD_str2(const char *c) {
    unsigned char wk;
    for (int i=0;i<8;i++){
        wk = c[i];
        if  (wk == 0x00) break;
        I2C_Write1ByteRegister(LCD_ADDR, 0x40, wk );
    }
    __delay_us(30);
}
void LCD_clear(){
    writeLCDCommand(0x01);
    __delay_us(1100);        //Instruction Execution Time 0.59-1.08ms (550:NG, 600:GOOD)
}
void writeLCDData(char t_data){
    I2C_Write1ByteRegister(LCD_ADDR, 0x40, t_data );
    __delay_us(30);     //Instruction Execution Time 14.3-26.3us
}
void LCD_SetCG(const char *c){
    for (int i=0;i<40;i++){
        writeLCDCommand(0x48+(char)i);
        writeLCDData(c[i]);
    }
}

/******** I2C AS5600 ********/
#define AS5600_ADDR 0x36
bool AS5600 = true;//false;
uint8_t ReadBuffer[4];
uint16_t ReadBuffer16;
#define STATUS_MD 0b00100000
#define STATUS_ML 0b00010000
#define STATUS_MH 0b00001000


void motor_on(unsigned int Direction, uint16_t PWM){
    /*
     * PWM range 0..100 (0%..100%) (PWMDC 0..319)
     */
    PWM = PWM * 319 / 100;
    if (Direction ==1){
        PWM4CONbits.PWM4EN = 0; // REV pin = Disable
        RC4_SetLow(); // REV pin = Low
        PWM3_LoadDutyValue(PWM);
        PWM3CONbits.PWM3EN = 1; // FWD pin = Enable
    }
    if (Direction ==0){
        PWM3CONbits.PWM3EN = 0; // FWD pin = Disable
        RC5_SetLow(); // REV pin = Low
        PWM4_LoadDutyValue(PWM);
        PWM4CONbits.PWM4EN = 1; // REV pin = Enable
    }
}
void motor_off(void){
    PWM3CONbits.PWM3EN = 0; // FWD pin = Disable
    RC3_SetLow();
    PWM4CONbits.PWM4EN = 0; // REV pin = Disable
    RC4_SetLow();    
}
void motor_brake(void){
    PWM3CONbits.PWM3EN = 0; // FWD pin = Disable
    RC3_SetHigh();
    PWM4CONbits.PWM4EN = 0; // REV pin = Disable
    RC4_SetHigh();    
}
bool isPushed_REV_SW(void){
    if ( IO_RA4_GetValue() == 0 ){
        return true;
        }
    else {
        return false;
    }    
}
bool isPushed_FWD_SW(void){
    if ( IO_RA5_GetValue() == 0 ){
        return true;
        }
    else {
        return false;
    }    
}
bool isPushed_RA2_SW(void){         // for test
    if ( IO_RA2_GetValue() == 0 ){
        return true;
        }
    else {
        return false;
    }    
}
/*
                         Main application
 */


void main(void)
{
    char HEF_buffer [FLASH_ROWSIZE];
    unsigned r;
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    //INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
        
    __delay_ms(20);

    if ( is_I2C_Connected(LCD_ADDR) == 0 ) {
        LCD = true; 
    } else {
        LCD = false;
    }
    __delay_ms(20);

    // Test reading from HEF-block
    r = HEFLASH_readBlock (HEF_buffer, 0, FLASH_ROWSIZE);
    if (LCD) {
        LCD_Init();
        LCD_clear();
        ReadBuffer16 = HEF_buffer[1];
        ReadBuffer16 = (ReadBuffer16 << 8) | HEF_buffer[0];
        sprintf(DisplayData, "%04d", ReadBuffer16);
        LCD_xy(0,1); LCD_str2( DisplayData );
        sprintf(DisplayData, "%02x", SSP1ADD);  // 0x09=400KHz
        LCD_xy(6,1); LCD_str2( DisplayData );
        __delay_ms(1000);
    }
    
    motor_off();
    while (1)
    {
        // Add your application code
        if ( isPushed_FWD_SW() ){
            motor_on(FWD, 50);
            if ( RA0_GetValue() == 1 ){
                while( RA0_GetValue() == 1 ){
                    if( isPushed_REV_SW() ) break;
                    __delay_ms(1);
                }
            }
            while( RA0_GetValue()==0 ){
                if( isPushed_REV_SW() ) break;
                __delay_ms(1);
            }
            motor_brake();
        }
        __delay_ms(10);

        if ( isPushed_REV_SW() ){
                motor_on(REV, 100);
        } else {
            motor_brake();
        }
        
        if ( isPushed_RA2_SW() ){   // Test writing to HEF-block
            HEF_buffer[1] = ReadBuffer[2];
            HEF_buffer[0] = ReadBuffer[3];
            r = HEFLASH_writeBlock(0, HEF_buffer, 2);
        }
        
        I2C_ReadDataBlock(AS5600_ADDR, 0x0c, ReadBuffer, 2);
        ReadBuffer[2] = ReadBuffer[0] & 0x0f;
        ReadBuffer16 = ReadBuffer[2];
        ReadBuffer[3] = ReadBuffer[1];
        ReadBuffer16 = (ReadBuffer16 << 8) | ReadBuffer[3];
        if (LCD){
            sprintf(DisplayData, "%04d", ReadBuffer16); 
            LCD_xy(0,0); LCD_str2( DisplayData );
        }
        __delay_ms(100);
        
        I2C_ReadDataBlock(AS5600_ADDR, 0x0b, ReadBuffer, 1);
        if (LCD){
            if ( (ReadBuffer[0] & STATUS_MD) != 0 ){
                sprintf(DisplayData, "Magnet");    
            } else {
                sprintf(DisplayData, "        ");
            }
            LCD_xy(0,1); LCD_str2( DisplayData );
            if ( (ReadBuffer[0] & STATUS_ML) != 0){
                sprintf(DisplayData, "L");
            } else {
                if ( (ReadBuffer[0] & STATUS_MH) != 0){
                    sprintf(DisplayData, "H");
                } else {
                    sprintf(DisplayData, " ");
                }    
            }
            LCD_xy(7,1); LCD_str2( DisplayData );
        }
    }
}
/**
 End of File
*/
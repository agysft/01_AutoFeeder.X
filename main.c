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


void motor_on(unsigned int Direction, int PWM){
    /*
     * PWM range 0..100 (0%..100%) (PWMDC 0..319)
     */
    PWM = PWM * 319 /100;
    if (Direction ==1){
        PWM4CONbits.PWM4EN = 0; // REV pin = Disable
        RC4_SetLow(); // REV pin = Low
        PWM3DCH = (unsigned char)(PWM >> 2);
        PWM3DCL = (unsigned char)(PWM << 6);
        PWM3CONbits.PWM3EN = 1; // FWD pin = Enable
    }
    if (Direction ==0){
        PWM3CONbits.PWM3EN = 0; // FWD pin = Disable
        RC5_SetLow(); // REV pin = Low
        PWM4DCH = (unsigned char)(PWM >> 2);
        PWM4DCL = (unsigned char)(PWM << 6);
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

/*
                         Main application
 */
void main(void)
{
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
            }
        else {
            motor_brake();
        }  

    }
}
/**
 End of File
*/
/*
 * File:   newmain.c
 * Author: Kaone Thobega
 *
 * Created on 22 February 2024, 20:11
 */

#pragma config FOSC = HS   // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF  // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON  // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF   // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF   // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF   // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF    // Flash Program Memory Code Protection bit (Code protection off)

#include <xc.h>
#define _XTAL_FREQ 4000000
#define Delay 1000

#define complete 1
#define incomplete 0

void ADC_Initialisation()
{
    ADCON0= 0b00000001; //Select AN0 channel and turn on ADC module
    ADCON1 = 0b00001110; //// Set Vref- to VSS, Vref+ to VDD
}

 uint16_t adc()
{
    while(ADCON0bits.GO);
   ADCON0bits.GO = 1; // starting the adc conversion
        return ((ADRESH<<8) | ADRESL); //bit shifting operation
      
}



void main(void) {
   
    TRISA=0xFF; // making portA input
    TRISB=0x00; //making portB output
    TRISC=0x00; //making portC output
   
    ADC_Initialisation();
    while(1)
    {
      
        PORTB =  (uint8_t) adc(); // initialising adc result to portB. Also casting adc to 8 bit.
        __delay_ms(100);
    }
    return;
}


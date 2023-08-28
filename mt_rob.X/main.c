/*
 * File:   main.c
 * Author: hirot
 *
 * Created on 2023/08/27, 1:39
 */

// PIC16F1779 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover Mode (Internal/External Switchover Mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit cannot be cleared once it is set by software)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR)
#pragma config PLLEN = ON       // Phase Lock Loop enable (4x PLL is always enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), high trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ 32000000
#include <xc.h>
#include<conio.h>
#include<stdio.h>
#include<stdbool.h>

unsigned char g_ReadData;

void __interrupt() ISR(void);
void motorA(int duty);
void motorB(int duty);
void motorC(int duty);
void motorD(int duty);
void motorE(int duty);
void motorF(int duty);
bool SwitchA_Read(void);
bool SwitchB_Read(void);
bool SwitchC_Read(void);
bool SwitchD_Read(void);
bool Signal1_Read(void);
bool Signal2_Read(void);
bool Signal3_Read(void);
bool Signal4_Read(void);
void DataWrite(unsigned char data);
void putch(unsigned char data);
void LEDON(void);
void LEDOFF(void);

void main(void) {
    OSCCON = 0b11110000;
    ANSELA = 0x00;
    ANSELB = 0x00;
    ANSELC = 0x00;
    ANSELD = 0x00;
    ANSELE = 0x00;
    TRISA = 0b11000000;
    TRISB = 0b00100000;
    TRISC = 0b00001111;
    TRISD = 0b00000011;
    LATA = 0x00;
    LATB = 0x00;
    LATC = 0x00;
    LATD = 0x00;
    LATE = 0x00;
    
    WPUA = 0x00;
    WPUB = 0x00;
    WPUC = 0b00001100;
    WPUD = 0b00000011;
    OPTION_REGbits.nWPUEN = 0;
    
    
    
    //TMR2設定
    T2CLKCONbits.CS = 0b0001;
    T2CON = 0b10000000;
    T2PR = 0xFF;
    
    //PWM3設定
    RA0PPS = 0b011001;
    //RA1PPS = 0b011001;
    PWM3CON = 0b10000000;
    CCPTMRS2bits.P3TSEL = 0b00;
    PWM3DCH = 0x00;
    PWM3DCL = 0x00;
    
    //PWM4設定
    RA2PPS = 0b011010;
    //RA3PPS = 0b011010;
    PWM4CON = 0b10000000;
    CCPTMRS2bits.P4TSEL = 0b00;
    PWM4DCH = 0x00;
    PWM4DCL = 0x00;
    
    //CCP1(PWM利用)設定
    //RB3PPS = 0b010101;
    RB2PPS = 0b010101;
    CCP1CON = 0b10011100;
    CCPTMRS1bits.C1TSEL = 0b00;
    CCPR1H = 0x00;
    CCPR1L = 0x00;
    
    //CCP2(PWM利用)設定
    //RB1PPS = 0b010110;
    RB0PPS = 0b010110;
    CCP2CON = 0b10011100;
    CCPTMRS1bits.C2TSEL = 0b00;
    CCPR2H = 0x00;
    CCPR2L = 0x00;
    
    //CCP7(PWM利用)設定
    //RD7PPS = 0b010111;
    RD6PPS = 0b010111;
    CCP7CON = 0b10011100;
    CCPTMRS1bits.C7TSEL = 0b00;
    CCPR7H = 0x00;
    CCPR7L = 0x00;
    
    //CCP8(PWM利用)設定
    //RD5PPS = 0b011000;
    RD4PPS = 0b011000;
    CCP8CON = 0b10011100;
    CCPTMRS1bits.C8TSEL = 0b00;
    CCPR8H = 0x00;
    CCPR8L = 0x00;
    
    //EUSART設定
    RXPPS = 0b001101;
    RB4PPS = 0b100100;
    TX1STA = 0b00100100;
    RC1STA = 0b10010000;
    BAUD1CON = 0b00001000;
    SP1BRGL = 416 & 0x00FF;
    SP1BRGH = (416 >> 8) & 0x00FF;
    
    PIR1bits.RCIF = 0;
    PIE1bits.RCIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    
    int sum = 0;
    
    motorA(0);
    motorB(0);
    motorC(0);
    motorD(0);
    motorE(0);
    motorF(0);
    while(SwitchA_Read());
    
    while(1){
        
        sum = 0;
        if(Signal1_Read()){
            sum += 1;
        }
        else;
        
        if(Signal2_Read()){
            sum += 2;
        }
        else;
        
        if(Signal3_Read()){
            sum += 4;
        }
        else;
        
        if(Signal4_Read()){
            sum += 8;
        }
        else;
        
        switch(sum){
            case 0:
                motorA(0);
                motorB(0);
                motorC(0);
                motorD(0);
                motorE(0);
                motorF(600);
                break;
            case 1:
                motorA(0);
                motorB(0);
                motorC(0);
                motorD(0);
                motorE(600);
                motorF(0);
                break;
            case 2:
                motorA(0);
                motorB(0);
                motorC(0);
                motorD(0);
                motorE(-600);
                motorF(0);
                break;
            case 3:
                motorA(0);
                motorB(0);
                motorC(0);
                motorD(0);
                motorE(0);
                motorF(-600);
                break;
            case 4:
                motorA(-600);
                motorB(0);
                motorC(0);
                motorD(600);
                motorE(0);
                motorF(0);
                break;
            case 5:
                motorA(0);
                motorB(600);
                motorC(-600);
                motorD(0);
                motorE(0);
                motorF(0);
                break;
            case 6:
                motorA(0);
                motorB(-600);
                motorC(600);
                motorD(0);
                motorE(0);
                motorF(0);
                break;
            case 7:
                motorA(600);
                motorB(0);
                motorC(0);
                motorD(-600);
                motorE(0);
                motorF(0);
                break;
            case 8:
                motorA(-600);
                motorB(600);
                motorC(-600);
                motorD(600);
                motorE(0);
                motorF(0);
                break;
            case 9:
                motorA(600);
                motorB(600);
                motorC(-600);
                motorD(-600);
                motorE(0);
                motorF(0);
                break;
            case 10:
                motorA(-600);
                motorB(-600);
                motorC(600);
                motorD(600);
                motorE(0);
                motorF(0);
                break;
            case 11:
                motorA(600);
                motorB(-600);
                motorC(600);
                motorD(-600);
                motorE(0);
                motorF(0);
                break;
            case 12:
                motorA(-600);
                motorB(-600);
                motorC(-600);
                motorD(-600);
                motorE(0);
                motorF(0);
                break;
            case 13:
                motorA(600);
                motorB(600);
                motorC(600);
                motorD(600);
                motorE(0);
                motorF(0);
                break;
            case 14:
                motorA(0);
                motorB(0);
                motorC(0);
                motorD(0);
                motorE(0);
                motorF(0);
                break;
            default:
                motorA(0);
                motorB(0);
                motorC(0);
                motorD(0);
                motorE(0);
                motorF(0);
                break;
        }
        /*for(int i = -600; i < 600; i++){
            //motorA(i);
            //motorB(i);
            //motorC(i);
            //motorD(i);
            //motorE(i);
            motorF(i);
            //printf("%c:%d\r\n", g_ReadData, i);
            LEDON();
            __delay_ms(1);
        }
        for(int i = 600; i > -600; i--){
            //motorA(i);
            //motorB(i);
            //motorC(i);
            //motorD(i);
            //motorE(i);
            motorF(i);
            //printf("i:%d\r\n", i);
            LEDOFF();
            __delay_ms(1);
        }*/
        
        /*if(!SwitchD_Read()){
            LEDON();
        }
        else
            LEDOFF();
        */
        
        /*if(Signal4_Read()){
            LEDON();
        }
        else
            LEDOFF();
        */
    }
    return;
}

void __interrupt() ISR(void){
    if(PIR1bits.RCIF){
        PIR1bits.RCIF = 0;
        if(RC1STAbits.FERR || RC1STAbits.OERR){
            RC1STA = 0x00;
            RC1STA = 0x90;
        }
        else{
            g_ReadData = RC1REG;
        }
    }
    else;
}

void motorA(int duty){
    
    duty > 600 ? (duty = 600) : duty;
    duty < -600 ? (duty = 600) : duty;
    
    if(duty > 0){
        PWM3DCL = (duty << 8) & 0x00FF;
        PWM3DCH = (duty >> 2) & 0x00FF;
        RA0PPS = 0b011001;
        RA1PPS = 0x00;
        LATAbits.LATA1 = 0;
    }
    else if(duty < 0){
        duty *= -1;
        PWM3DCL = (duty << 8) & 0x00FF;
        PWM3DCH = (duty >> 2) & 0x00FF;
        RA1PPS = 0b011001;
        RA0PPS = 0x00;
        LATAbits.LATA0 = 0;
    }
    else{
        RA0PPS = 0x00;
        RA1PPS = 0x00;
        LATAbits.LATA0 = 0;
        LATAbits.LATA1 = 0;
    }
    
}

void motorB(int duty){
    
    duty > 600 ? (duty = 600) : duty;
    duty < -600 ? (duty = 600) : duty;
    
    if(duty > 0){
        PWM4DCL = (duty << 8) & 0x00FF;
        PWM4DCH = (duty >> 2) & 0x00FF;
        RA2PPS = 0b011010;
        RA3PPS = 0x00;
        LATAbits.LATA3 = 0;
    }
    else if(duty < 0){
        duty *= -1;
        PWM4DCL = (duty << 8) & 0x00FF;
        PWM4DCH = (duty >> 2) & 0x00FF;
        RA3PPS = 0b011010;
        RA2PPS = 0x00;
        LATAbits.LATA2 = 0;
    }
    else{
        RA2PPS = 0x00;
        RA3PPS = 0x00;
        LATAbits.LATA2 = 0;
        LATAbits.LATA3 = 0;
    }
    
}

void motorC(int duty){
    
    duty > 600 ? (duty = 600) : duty;
    duty < -600 ? (duty = 600) : duty;
    
    if(duty > 0){
        CCPR1L = (duty << 8) & 0x00FF;
        CCPR1H = (duty >> 2) & 0x00FF;
        RB2PPS = 0b010101;
        RB3PPS = 0x00;
        LATBbits.LATB3 = 0;
    }
    else if(duty < 0){
        duty *= -1;
        CCPR1L = (duty << 8) & 0x00FF;
        CCPR1H = (duty >> 2) & 0x00FF;
        RB3PPS = 0b010101;
        RB2PPS = 0x00;
        LATBbits.LATB2 = 0;
    }
    else{
        RB2PPS = 0x00;
        RB3PPS = 0x00;
        LATBbits.LATB2 = 0;
        LATBbits.LATB3 = 0;
    }
    
    return;
    
}

void motorD(int duty){
    
    duty > 600 ? (duty = 600) : duty;
    duty < -600 ? (duty = 600) : duty;
    
    if(duty > 0){
        CCPR2L = (duty << 8) & 0x00FF;
        CCPR2H = (duty >> 2) & 0x00FF;
        RB0PPS = 0b010110;
        RB1PPS = 0x00;
        LATBbits.LATB1 = 0;
    }
    else if(duty < 0){
        duty *= -1;
        CCPR2L = (duty << 8) & 0x00FF;
        CCPR2H = (duty >> 2) & 0x00FF;
        RB1PPS = 0b010110;
        RB0PPS = 0x00;
        LATBbits.LATB0 = 0;
    }
    else{
        RB0PPS = 0x00;
        RB1PPS = 0x00;
        LATBbits.LATB0 = 0;
        LATBbits.LATB1 = 0;
    }
    
    return;
    
}

void motorE(int duty){
    
    duty > 600 ? (duty = 600) : duty;
    duty < -600 ? (duty = 600) : duty;
    
    if(duty > 0){
        CCPR7L = (duty << 8) & 0x00FF;
        CCPR7H = (duty >> 2) & 0x00FF;
        RD6PPS = 0b010111;
        RD7PPS = 0x00;
        LATDbits.LATD7 = 0;
    }
    else if(duty < 0){
        duty *= -1;
        CCPR7L = (duty << 8) & 0x00FF;
        CCPR7H = (duty >> 2) & 0x00FF;
        RD7PPS = 0b010111;
        RD6PPS = 0x00;
        LATDbits.LATD6 = 0;
    }
    else{
        RD6PPS = 0x00;
        RD7PPS = 0x00;
        LATDbits.LATD6 = 0;
        LATDbits.LATD7 = 0;
    }
    
    return;
    
}

void motorF(int duty){
    
    duty > 600 ? (duty = 600) : duty;
    duty < -600 ? (duty = 600) : duty;
    
    if(duty > 0){
        CCPR8L = (duty << 8) & 0x00FF;
        CCPR8H = (duty >> 2) & 0x00FF;
        RD4PPS = 0b011000;
        RD5PPS = 0x00;
        LATDbits.LATD5 = 0;
    }
    else if(duty < 0){
        duty *= -1;
        CCPR8L = (duty << 8) & 0x00FF;
        CCPR8H = (duty >> 2) & 0x00FF;
        RD5PPS = 0b011000;
        RD4PPS = 0x00;
        LATDbits.LATD4 = 0;
    }
    else{
        RD4PPS = 0x00;
        RD5PPS = 0x00;
        LATDbits.LATD4 = 0;
        LATDbits.LATD5 = 0;
    }
    
    return;
    
}

bool SwitchA_Read(void){
    return PORTCbits.RC2;
}

bool SwitchB_Read(void){
    return PORTCbits.RC3;
}

bool SwitchC_Read(void){
    return PORTDbits.RD0;
}

bool SwitchD_Read(void){
    return PORTDbits.RD1;
}

bool Signal1_Read(void){
    return PORTAbits.RA7;  
}

bool Signal2_Read(void){
    return PORTAbits.RA6;  
}

bool Signal3_Read(void){
    return PORTCbits.RC0;  
}

bool Signal4_Read(void){
    return PORTCbits.RC1;  
}

void DataWrite(unsigned char data){
    while(!PIR1bits.TXIF);
    PIR1bits.TXIF = 0;
    TX1REG = data;
}

void putch(unsigned char data){    
    DataWrite(data);
}

void LEDON(void){
    LATDbits.LATD2 = 1;
}

void LEDOFF(void){
    LATDbits.LATD2 = 0;
}
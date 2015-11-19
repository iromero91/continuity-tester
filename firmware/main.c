/* s
 * File:   main.c
 * Author: José I. Romero
 *
 * Created on November 13, 2015, 5:49 PM
 */

#include <xc.h>
#include <stdint.h>

// CONFIG
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select bit (MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Detect (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)

#define POWER_DOWN 0
#define CONT_IDLE 1
#define CONT_BEEP 2

static uint8_t state;

void beep() {
    T2CON = 0; // No prescaler or postscaler on timer 2
    PR2 = 5; // 7.75KHz/(5+1) = 1.292KHz tone signal (roughly 1/3 subharmonic of resonant freq)
    T2CONbits.TMR2ON = 1; // Turn timer 2 on
    CCPR1L = 3; // 50% duty cycle (3)
    CCP1CONbits.DC1B = 0; // (0) lsb
    CCP1CONbits.CCP1M = 0b1100; // PWM Active High
}

void unbeep() {
    T2CONbits.TMR2ON = 0; // Timer 2 off
    CCP1CONbits.CCP1M = 0; // CCP off
}

void adc_init() {
    ANSELbits.ADCS = 0b011; // Use free running ADC clock source
    ANSELbits.ANS = 0b0001; // Use AN0 for analog acquisition, all other pins digital
    ADCON0bits.ADFM = 1; // Right justified format
    ADCON0bits.VCFG = 0; // Reference tied to VCC
    ADCON0bits.CHS = 0;  // Channel 0 (GP0)
}

uint16_t adc_read () {
    uint16_t result;
    // Acquire ADC sample
    ADCON0bits.ADON = 1; // Turn on ADC
    ADCON0bits.GO = 1; // Start conversion
    while (ADCON0bits.GO_nDONE) NOP(); // Wait for result
    ADCON0bits.ADON = 0; // Turn off ADC
    result = (ADRESL + ADRESH << 8);
    return result;
}

void power_down() {
    /*** Enter State ***/
    // Configure IO
    TRISIO = 0b00011000; // Only enable button (GP3) and probe (GP4) inputs
    GPIO   = 0b00000011; // Pull up GP0 (27K) and GP1 (10k)
    IOC    = 0b00011000; // Enable wake up on pin change for probe and button
    // Configure ADC
    ANSEL  = 0; // All pins digital
    ADCON0 = 0; // Turn off ADC
    // Configure Comparator Module
    CMCON0 = 0b00000111; // Use configuration 111, lowest power off
    // Configure CCP
    CCP1CON = 0; // CCP1 off
    // Configure interrupts
    INTCONbits.GPIE = 1; // Enable GPIO wake up.
    INTCONbits.GPIF = 0; // Clear flag
    /*** In State ***/
    do {
        SLEEP(); // Sleep until either the probe or button wakes us up.
        NOP();
        INTCONbits.GPIF = 0; // Clear flag
    } while (GPIObits.GP3 && GPIObits.GP4);
    /*** Exit State ***/
    INTCONbits.GPIE = 0; // Disable GPIO interrupts
    IOC = 0;
    state = CONT_IDLE;
    return;
}

void cont_idle() {
    uint8_t next = POWER_DOWN;

    /*** Enter State ***/
    unbeep(); // Silence BEEP
    // Configure IO
    TRISIO = 0b00001001; // Set up GP0 (probe) and GP3 (button) as inputs
    GPIO   = 0b00100010; // Activate bias current for resistor test (27K series R)
    adc_init(); // Configure ADC

    /*** In State ***/
    while(1) {
        if (adc_read() < 2) {
            next = CONT_BEEP; // Jump on first sample (glitch capture)
            break;
        }
    }
    /*** Exit State ***/
    state = next;
    return;
}

void cont_beep() {
    uint8_t next = POWER_DOWN;
    uint8_t samples = 0;

    /*** Enter State ***/
    beep(); // Start beeper
    GPIObits.GP5 = 0;

    /*** In State ***/
    while(1) {
        if (adc_read() > 3) {
            samples++;
        } else {
            samples = 0;
        }
        if (samples > 5) {  // ~ 60 ms debounce period (latching)
            next = CONT_IDLE;
            break;
        }
    }
    /*** Exit State ***/
    state = next;
    return;
}

int main () {
    OSCCON = 0; // 31KHz Internal Oscillator
    state = POWER_DOWN;
    for (uint8_t i=250; i--; i > 0) NOP();
    for (;;) {
        switch (state) {
            case POWER_DOWN: power_down(); break;
            case CONT_IDLE: cont_idle(); break;
            case CONT_BEEP: cont_beep(); break;
            default: power_down();
        }
    }
}
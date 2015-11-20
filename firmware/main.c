/*
 * Copyright 2015 José I. Romero
 *
 * This file is part of "Low Power Continuity Tester".
 *
 * "Low Power Continuity Tester" is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY ormain.c:204: error: (192) undefined identifier "STATE"
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this software. If not, see http://www.gnu.org/licenses/.
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

// Time before falling into sleep mode
#define IDLE_TIME 5000 /* ~100s */

// Continuity test constants
#define OPEN_TRESHOLD 3  /* ~200Ω */
#define SHORT_TRESHOLD 2 /* ~150Ω */
#define LATCH_TIME 4     /* ~60ms */

// Diode test constants
#define DIODE_TRESHOLD_MAX 250 /* ~0.7v */
#define DIODE_TRESHOLD_MIN 25  /* ~0.075v */
#define DIODE_BEEP_TIME 7     /* ~0.25s */

// Tester states
typedef enum state_t {
    ST_SLEEP,
    ST_CONTINUITY_IDLE,
    ST_CONTINUITY_BEEP,
    ST_INVERSE_IDLE,
    ST_INVERSE_BEEP,
    ST_DIODE_IDLE,
    ST_DIODE_BEEP,
    ST_DIODE_OK
} state_t;

static state_t State;  // Current state

void beep() {
    T2CON = 0; // No prescaler or postscaler on timer 2
    PR2 = 5; // 7.75KHz/(5+1) = 1.292KHz tone signal
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
    ANSELbits.ANS = 0b0001; // Use AN0 for ADC, all other pins digital
    ADCON0bits.ADFM = 0; // LEFT justified format
    ADCON0bits.VCFG = 0; // Reference tied to VCC
}

uint16_t adc_read () {
    // Acquire ADC sample
    ADCON0bits.CHS = 0;  // Channel 0 (GP0)
    ADCON0bits.ADON = 1; // Turn on ADC
    ADCON0bits.GO = 1; // Start conversion
    while (ADCON0bits.GO_nDONE) NOP(); // Wait for result
    ADCON0bits.ADON = 0; // Turn off ADC
    return (ADRESH<<2) + (ADRESL>>6);
}

void blink_mode(uint16_t time, uint8_t mode) {
    uint8_t t = time & 127;
    if (t > 128 - mode * 16) {
        GPIObits.GP5 = ((t & 15) < 5)? 1 : 0;
    } else {
        GPIObits.GP5 = 0;
    }
}

uint8_t check_button() {
    static int8_t Debounce = 0;
    if (GPIObits.GP3 == 0) {
        if (Debounce >= 0) Debounce++;
    } else {
        Debounce = 0;
    }
    if (Debounce > LATCH_TIME) {
        Debounce = -1; // Block the button until it's released.
        return 1;
    }
    return 0;
}

/*
 * Sleep State
 */
void state_sleep() {
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
    State = ST_CONTINUITY_IDLE;
    return;
}

///// Continuity Mode States /////

void state_continuity_idle() {
    state_t next = ST_SLEEP;
    uint16_t time = 0;

    /*** Enter State ***/
    unbeep(); // Silence BEEP
    // Configure IO
    TRISIO = 0b00001001; // Set up GP0 (probe) and GP3 (button) as inputs
    GPIO   = 0b00000010; // Activate bias current for cont. test (22k series R)
    adc_init(); // Configure ADC

    /*** In State ***/
    while(time < IDLE_TIME) {  // Go back to sleep if idling too long
        if (State == ST_CONTINUITY_IDLE) {
            blink_mode(time, 1);
            if (check_button()){
                next = ST_INVERSE_IDLE; // Cycle to next mode
                break;
            } else if (adc_read() < SHORT_TRESHOLD) {
                next = ST_CONTINUITY_BEEP; // Jump immediately (glitch capture)
                break;
            }
        } else { // ST_INVERSE_IDLE
            blink_mode(time, 2);
            if (check_button()){
                next = ST_DIODE_IDLE; // Cycle to next mode
                break;
            } else if (adc_read() > OPEN_TRESHOLD) {
                next = ST_INVERSE_BEEP; // Jump immediately (glitch capture)
                break;
            }
        }
        time++;
    }
    /*** Exit State ***/
    State = next;
    return;
}

void state_continuity_beep() {
    state_t next = ST_CONTINUITY_IDLE;
    uint8_t samples = 0;

    /*** Enter State ***/
    beep(); // Start beeper
    GPIObits.GP5 = 1; // Turn on LED

    /*** In State ***/
    while(samples < LATCH_TIME) {
        if (State == ST_CONTINUITY_BEEP) {
            if (check_button()) {
                next = ST_INVERSE_IDLE; // Cycle to next mode
                break;
            }else if (adc_read() > OPEN_TRESHOLD) {
                samples++;
            } else {
                samples = 0;
            }
        } else { // ST_INVERSE_BEEP
            if (check_button()) {
                next = ST_DIODE_IDLE; // Cycle to next mode
                break;
            }else if (adc_read() < SHORT_TRESHOLD) {
                samples++;
                next = ST_INVERSE_IDLE;
            } else {
                samples = 0;
            }
        }
    }

    /*** Exit State ***/
    State = next;
    return;
}

///// Diode Mode States /////

void state_tes() {
    state_t next = ST_SLEEP;
    uint16_t voltage;
    uint8_t i;

    /*** Enter State ***/
    unbeep(); // Silence BEEP
    // Configure IO
    TRISIO = 0b00001001; // Set up GP0 (probe) and GP3 (button) as inputs
    GPIO   = 0b00010000; // Activate bias current for diode test (4k7 series R)
    adc_init(); // Configure ADC

    /*** In State ***/
    while(1) {
        voltage = adc_read();
        GPIObits.GP5 = 0;
        GPIObits.GP5 = 1;
        NOP();NOP();NOP();NOP();NOP();NOP();
        GPIObits.GP5 = 0;
        for (i=0;i<16;i++) {
            GPIObits.GP5 = 0;
            GPIObits.GP5 = 1;
            GPIObits.GP5 = 0;
            GPIObits.GP5 = voltage & 1;
            voltage = voltage / 2;
        }
        GPIObits.GP5 = 0;

        if (check_button()) {
            next = ST_CONTINUITY_IDLE;  // Cycle to next mode
            break;
        }
    }

    /*** Exit State ***/
    State = next;
    return;
}

void state_diode_idle() {
    state_t next = ST_SLEEP;
    uint16_t time = 0;

    /*** Enter State ***/
    unbeep(); // Silence BEEP
    // Configure IO
    TRISIO = 0b00001001; // Set up GP0 (probe) and GP3 (button) as inputs
    GPIO   = 0b00010000; // Activate bias current for diode test (4k7 series R)
    adc_init(); // Configure ADC

    /*** In State ***/
    while(time < IDLE_TIME) { // Go back to sleep if idling too long
        blink_mode(time, 3);
        if (check_button()) {
            next = ST_CONTINUITY_IDLE;  // Cycle to next mode
            break;
        } else if (adc_read() < DIODE_TRESHOLD_MAX) {
            next = ST_DIODE_BEEP;
            break;
        }
        time++;
    }

    /*** Exit State ***/
    State = next;
    return;
}

void state_diode_beep() {
    state_t next = ST_DIODE_OK;
    uint8_t samples = 0;
    uint16_t voltage;

    /*** Enter State ***/
    beep(); // Start beeper
    GPIObits.GP5 = 1; // Turn on LED

    /*** In State ***/
    while(samples < DIODE_BEEP_TIME) {
        voltage = adc_read();
        if (check_button()) {
            next = ST_CONTINUITY_IDLE; // Cycle to next mode
            break;
        }else if (voltage > DIODE_TRESHOLD_MAX) {
            next = ST_DIODE_IDLE; // Probes open, abort
            break;
        } else if (voltage > DIODE_TRESHOLD_MIN){
            samples++;
        } else {
            samples = 0;
        }
    }

    /*** Exit State ***/
    State = next;
    return;
}

void state_diode_ok() {
    state_t next  = ST_DIODE_IDLE;
    /*** Enter State ***/
    unbeep(); // Silence beeper

    /*** In State ***/
    while (adc_read() <= DIODE_TRESHOLD_MAX) {
        if (check_button()){
            next = ST_CONTINUITY_IDLE;
            break;
        };
    }

    /*** Exit State ***/
    State = next;
    return;
}

int main () {
    OSCCON = 0; // 31KHz Internal Oscillator
    State = ST_SLEEP;
    for (uint8_t i=250; i--; i > 0) NOP();
    for (;;) {
        switch (State) {
        case ST_SLEEP:           state_sleep();           break;

        case ST_INVERSE_IDLE: // Fall through
        case ST_CONTINUITY_IDLE: state_continuity_idle(); break;
        case ST_INVERSE_BEEP: // Fall through
        case ST_CONTINUITY_BEEP: state_continuity_beep(); break;

        case ST_DIODE_IDLE:      state_diode_idle();      break;
        case ST_DIODE_BEEP:      state_diode_beep();      break;
        case ST_DIODE_OK:        state_diode_ok();        break;

        default:
            state_sleep();
        }
    }
}

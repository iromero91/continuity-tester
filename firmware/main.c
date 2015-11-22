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

// CONFIG1
#pragma config FOSC = INTOSC    //  (INTOSC oscillator; I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOREN = OFF    // Low Power Brown-out Reset enable bit (LPBOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// Time before falling into sleep mode
#define IDLE_TIME 5000 /* ~100s */

// Continuity test constants
#define OPEN_TRESHOLD (3<<6)  /* ~200Ω */
#define SHORT_TRESHOLD (2<<6) /* ~150Ω */
#define LATCH_TIME 4     /* ~60ms */

// Diode test constants
#define DIODE_TRESHOLD_MAX (290<<6) /* ~0.7v */
#define DIODE_TRESHOLD_MIN (25<<6)  /* ~0.075v */
#define DIODE_BEEP_TIME 7     /* ~0.12s */

#define BEEP_FREQ 1333.33
#define BEEP_PERIOD (31000 / BEEP_FREQ)

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
    PWM3CON    = 0b11000000; // Enable PWM
    PWM3CLKCON = 0b00000010; // LFINTOSC clock source, no prescaler
    PWM3PH = 0;
    PWM3PR = BEEP_PERIOD; // Set oscillator period
    PWM3DC = BEEP_PERIOD / 2; // 50% duty cycle
    PWM3LDCONbits.PWM3LD = 1; // Load registers!
}

void unbeep() {
    PWM3CON = 0; // PWM 3 OFF
    PWM3CLKCON = 0;
}

void adc_init() {
    ANSELA = 0b00000001; // Use AN0
    ADCON1bits.ADCS = 0b111; // Use FRC clock
    ADCON1bits.ADFM = 0; // LEFT justified format
    ADCON1bits.ADPREF = 0b00; // Reference tied to VCC
}

// Time: 2.6ms sample acq, 5.7ms including call.
uint16_t adc_read () {
    // Acquire ADC sample
    ADCON0bits.CHS = 0;  // Channel 0 (GP0)
    ADCON0bits.ADON = 1; // Turn on ADC
    ADCON0bits.GO = 1; // Start conversion
    while (ADCON0bits.GO_nDONE) NOP(); // Wait for result
    ADCON0bits.ADON = 0; // Turn off ADC
    return (ADRESH<<8) + ADRESL;
}

void blink_mode(uint8_t t, uint8_t mode) {
    if (t < mode) {
        LATAbits.LATA5 = ((t & 8) && (t & 4))? 1 : 0;
    } else {
        LATAbits.LATA5 = 0;
    }
}

uint8_t check_button() {
    static int8_t Debounce = 0;
    if (PORTAbits.RA3 == 0) {
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
    TRISA = 0b00011000; // Only enable button (RA3) and probe (RA4) inputs
    LATA  = 0b00000011; // Pull up RA0 (27K) and RA1 (10k)
    IOCAN = 0b00011000; // Enable wake up on negative edge for probe and button
    WPUA = 0; // No weak pullups
    OPTION_REGbits.nWPUEN = 1;
    ODCONA = 0; // All IO is push-pull
    SLRCONA = 0; // Slew rate at maximum
    // Configure ADC
    ANSELA  = 0; // All pins digital
    ADCON0 = 0; // Turn off ADC
    // Configure Comparator Module
    CM1CON0 = 0; // Comparator off
    // Configure PWM
    PWM3CON = 0; // PWM3 (buzzer) off
    CWG1CON0 = 0; // CWG off.
    // Configure interrupts
    INTCONbits.IOCIE = 1; // Enable wake up on pin change.
    INTCONbits.IOCIF = 0; // Clear flag.
    /*** In State ***/
    do {
        SLEEP(); // Sleep until either the probe or button wakes us up.
        NOP();
        INTCONbits.IOCIF = 0; // Clear flag
        IOCAF = 0; // And flags
    } while (PORTAbits.RA3 && PORTAbits.RA4);
    /*** Exit State ***/
    INTCONbits.IOCIE = 0; // Disable GPIO interrupts
    IOCAN = 0;
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
    TRISA = 0b00001001; // Set up GP0 (probe) and GP3 (button) as inputs
    LATA  = 0b00000010; // Activate bias current for cont. test (22k series R)
    adc_init(); // Configure ADC

    /*** In State ***/
    while(time < IDLE_TIME) {  // Go back to sleep if idling too long
        if (State == ST_CONTINUITY_IDLE) {
            blink_mode(time&127, 1*16);
            if (check_button()){
                next = ST_INVERSE_IDLE; // Cycle to next mode
                break;
            } else if (adc_read() < SHORT_TRESHOLD) {
                next = ST_CONTINUITY_BEEP; // Jump immediately (glitch capture)
                break;
            }
        } else { // ST_INVERSE_IDLE
            blink_mode(time&127, 2*16);
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
    LATAbits.LATA5 = 1; // Turn on LED

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

void state_test_ADC() {
    state_t next = ST_SLEEP;
    uint16_t voltage;
    uint8_t i;

    /*** Enter State ***/
    unbeep(); // Silence BEEP
    // Configure IO
    TRISA = 0b00001001; // Set up GP0 (probe) and GP3 (button) as inputs
    LATA  = 0b00010000; // Activate bias current for diode test (4k7 series R)
    adc_init(); // Configure ADC

    /*** In State ***/
    while(1) {
        voltage = adc_read();
        LATAbits.LATA5 = 0;
        LATAbits.LATA5 = 1;
        NOP();NOP();NOP();NOP();NOP();NOP();
        LATAbits.LATA5 = 0;
        for (i=0;i<16;i++) {
            LATAbits.LATA5 = 0;
            LATAbits.LATA5 = 1;
            LATAbits.LATA5 = 0;
            LATAbits.LATA5 = voltage & 1;
            voltage = voltage / 2;
        }
        LATAbits.LATA5 = 0;

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
    TRISA = 0b00001001; // Set up GP0 (probe) and GP3 (button) as inputs
    LATA = 0b00010000; // Activate bias current for diode test (4k7 series R)
    adc_init(); // Configure ADC

    /*** In State ***/
    while(time < IDLE_TIME) { // Go back to sleep if idling too long
        blink_mode(time&127, 3*16);
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
    LATAbits.LATA5 = 1; // Turn on LED

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

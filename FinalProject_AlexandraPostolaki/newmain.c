/* 
 * File:   newmain.c
 * Author: grumpybee
 *         Alexandra Postolaki
 * Created on November 10, 2024, 3:07 PM
 * 
 * Balancing Beam [PID controller]
 *      This code sets up HC-SR04 ultrasonic sensor and the MS24 20 kg servo motor
 *      to balance a ping pong ball on a beam using PID controller.
 * 
 */


/* ORIGINAL CODE FOR OVERALL PROJECT */

#ifndef F_CPU
#define F_CPU 3333333  // clock frequency
#endif
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// PID constants (you may need to tune these values)
#define KP 90.0f       // Proportional Gain: corrects the error (difference between the target distance and the current distance) by a factor. The larger the error, the bigger the correction. 
#define KI 0.009f         // Integral Gain: accumulates (or integrates) error over time. It helps the system get rid of any steady, small error that the "P" term couldn't fix on its own
                        // or reset the integral when the error is small or within acceptable range. -> Try lowering by 0.1-0.5
#define KD 70.0f       // Derivative Gain: predicts future errors by looking at how fast the error is changing. It tries to smooth out the movement by responding to the rate of change of the error, helping to prevent overshooting.

// Global variables for PID control
float target_distance = 12.7; // Target distance in cm
float previous_error = 0.0f;
float integral = 0.0f;
float distance_cm = 0.0f;

// Servo PWM variables
volatile uint16_t pwm_pulse_width = 2550;  // Start with 2550ms (neutral position) - By calculations, neutral is at 2500, but I had to adjust to make up for the motor "teeth"
volatile uint16_t pulse_start = 0;
volatile uint16_t pulse_end = 0;
volatile uint16_t pulse_duration = 0;
volatile uint8_t edge_detected = 0;

//#define MAX_INTEGRAL 5.0f  // Originally was used for integral "clamping" for smoothness.
//#define MIN_INTEGRAL -5.0f // Can be used for slight improvement.
                             // Uncomment this and correlating code in PID_control function

void setupServo() {
    
    // Set pin C0 as output
    PORTC.DIR |= PIN0_bm;
    
    
	// Set pre-scaler to 2
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV2_gc; // 3333333 / 2 = 1666667
    
	// Set period
	// 1,666,667 (Hz) * 0.02 (sec) = 33333 (cycles)
	TCA0.SINGLE.PER = 33333;
    
	// Enable Timer A in single-slope PWM mode and enable compare match output
	TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    
	// Begin timer
	TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
    
	// Configure PORTMUX to route Timer A output to pin C0
	PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTC_gc;
    
}

void setupSensor() {
    
	// Set TRIG_PIN as output
	PORTD.DIRSET = PIN1_bm;  // setting PD1 as output

	// Set ECHO_PIN as input
	PORTC.DIRCLR = PIN1_bm;  // setting PC1 as input

	// Set Red LED as output
	PORTF.DIRSET = PIN5_bm;
	PORTF.OUTSET = PIN5_bm;
    
	// Set Green LED as output
	PORTF.DIRSET = PIN4_bm;
	PORTF.OUTSET = PIN4_bm;
    
	// Enable interrupt on rising for ECHO_PIN (PC1)
	PORTC.PIN1CTRL = PORT_ISC_RISING_gc; // Rising interrupt

	// Configure TCB0
	TCB0.CTRLB = TCB_CNTMODE_SINGLE_gc;             	// Single shot mode
	TCB0.CCMP = 0xFFFF;                             	// Setting CCMP register to max value
	TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;	// Enable TCB with a pre-scaler of 2 - easier calculations
	                                                    // 3333333 / 2 = 1666667
	// Enable interrupts globally
	sei();
}

void trigger_ultrasonic() {
    
	// Send a 10 microsecond pulse to the trigger pin
	PORTD.OUTSET = PIN1_bm;
	_delay_us(10);
	PORTD.OUTCLR = PIN1_bm;
}

ISR(PORTC_PORT_vect) {
	if (PORTC.IN & PIN1_bm) {                                   // Rising edge detected  PORTC.IN reg holds current input vals of pin on Port C (represents state high/low of pin on portC) -- Rising edge from low to high (0 to 1))
    	pulse_start = TCB0.CNT;                                 // Record the start time
    	PORTC.PIN1CTRL = PORT_ISC_FALLING_gc;                   	// Set to look for falling edge - echo detection
	} else {  // Falling edge detected
    	pulse_end = TCB0.CNT;                                   	// Record end time
    	if (pulse_end < pulse_start) {                          	// Because of overflow above max 0xFFFF
        	pulse_duration = (0xFFFF - pulse_start) + pulse_end;	// Calculate Pulse Width
    	} else {
        	pulse_duration = pulse_end - pulse_start;           	// Calculate Pulse Width
    	}
    	PORTC.PIN1CTRL = PORT_ISC_RISING_gc;                        // Set back to detect rising edge.
	}    

	PORTC.INTFLAGS = PIN1_bm;  // Clear interrupt flag
}


// PID Controller function
float PID_Control(float current_distance) {
    
    float error = target_distance - current_distance;       // If the current distance is greater than the target distance, error will be negative and PID output will be reduced.
    
    
    integral += error;
   
    // Clamping integral to prevent windup
//    if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
//    if (integral < MIN_INTEGRAL) integral = MIN_INTEGRAL;
    
    float derivative = error - previous_error;

    // Compute the PID output
    float output = KP * error + KI * integral + KD * derivative;
    
    // Update the previous error for the next cycle
    previous_error = error;

    return output;
}

int main() {
    
	setupServo();
    setupSensor();
    
    TCA0.SINGLE.CMP0 = pwm_pulse_width; 
    
	while (1) {
        
    	TCB0.CNT = 0;   // Resetting count
    	trigger_ultrasonic();
        _delay_ms(20);

    	//  Calculate distance in centimeters
    	distance_cm = (pulse_duration * 600e-9 * 34300) / 2;  // Pulse duration (in "ticks") converted to nanoseconds (1 "tick" = 600 ns | (1/(3333333 / 2)-> distance (in cm) using speed of sound (34300 cm/s)
        
        // changing pwm pulse width according to the current distance
   	    pwm_pulse_width = pwm_pulse_width + PID_Control(distance_cm);
        
        // Keeps range of servo motor rotation angle.
        if (pwm_pulse_width > 2800) {  
            pwm_pulse_width = 2800;   
        } else if (pwm_pulse_width < 2350) {  
            pwm_pulse_width = 2350; 
        }
        
        // Set duty cycle to correct pulse width.
        TCA0.SINGLE.CMP0 = pwm_pulse_width;
        
    	/* DEBUGGING TESTS: 
         * Create if/else statements: 
         * if distance_cm < 10 cm = red zone
         * if distance_cm between 10 and 14 cm  = mid zone (green and red on)
         * between 14 and 25 = green
         * otherwise (out of range) -> no LEDS on 
         */
        
    	if(distance_cm < 10) {
        	PORTF.OUTSET = PIN4_bm; // Turn off green LED
        	PORTF.OUTCLR = PIN5_bm; // Turn on red LED
    	}
    	else if(distance_cm >= 10 && distance_cm < 14) { // If within target distance range, red and greed LED should be on.
        	PORTF.OUTCLR = PIN4_bm; // Turn on green LED
        	PORTF.OUTCLR = PIN5_bm; // Turn on red LED
    	}
    	else if(distance_cm >= 14 && distance_cm < 25) {
        	PORTF.OUTCLR = PIN4_bm; // Turn on green LED
        	PORTF.OUTSET = PIN5_bm; // Turn off red LED
    	}
    	else {
        	PORTF.OUTSET = PIN4_bm; // Turn off green LED
        	PORTF.OUTSET = PIN5_bm; // Turn off red LED
    	}
	}
	return 0;
} 

/* SETTING UP ULTRASONIC SENSOR: */

/*#ifndef F_CPU
#define F_CPU 3333333  // clock frequency
#endif
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile uint16_t pulse_start = 0;
volatile uint16_t pulse_end = 0;
volatile uint16_t pulse_duration = 0;
volatile uint8_t edge_detected = 0;


void setup() {
	// Set TRIG_PIN as output
	PORTD.DIRSET = PIN1_bm;  // setting PD1 as output

	// Set ECHO_PIN as input
	PORTC.DIRCLR = PIN1_bm;  // setting PC1 as input

	// Set Red LED as output
	PORTF.DIRSET = PIN5_bm;
	PORTF.OUTSET = PIN5_bm;
    
	// Set Green LED as output
	PORTF.DIRSET = PIN4_bm;
	PORTF.OUTSET = PIN4_bm;
    
	// Enable interrupt on rising for ECHO_PIN (PC1)
	PORTC.PIN1CTRL = PORT_ISC_RISING_gc; // Rising interrupt

	// Configure TCB0
	TCB0.CTRLB = TCB_CNTMODE_SINGLE_gc;             	// Single shot mode
	TCB0.CCMP = 0xFFFF;                             	// Setting CCMP register to max val
	TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;	// Enable TCB with a prescaler of 2
    
	// Enable interrupts globally
	sei();
}

void trigger_ultrasonic() {
	// Send a 10 microsecond pulse to the trigger pin
	PORTD.OUTSET = PIN1_bm;
	_delay_us(10);
	PORTD.OUTCLR = PIN1_bm;
}

ISR(PORTC_PORT_vect) {
	if (PORTC.IN & PIN1_bm) {                             	// Rising edge detected  PORTC.IN reg holds current input vals of pin on Port C (represents state high/low of pin on portC) -- Rising edge from low to high (0 to 1))
    	pulse_start = TCB0.CNT;                           	// Record the start time
    	PORTC.PIN1CTRL = PORT_ISC_FALLING_gc;                   	// Set to look for falling edge - echo detection
	} else {  // Falling edge detected
    	pulse_end = TCB0.CNT;                                   	// Record end time
    	if (pulse_end < pulse_start) {                          	// Because of overflow above max 0xFFFF
        	pulse_duration = (0xFFFF - pulse_start) + pulse_end;	// Calculate Pulse Width
    	} else {
        	pulse_duration = pulse_end - pulse_start;           	// Calculate Pulse Width
    	}
    	PORTC.PIN1CTRL = PORT_ISC_RISING_gc;
	}    

	PORTC.INTFLAGS = PIN1_bm;  // Clear interrupt flag
}

int main() {
	setup();

	while (1) {
    	TCB0.CNT = 0;   // Resetting count
    	trigger_ultrasonic();
    	_delay_ms(50); // Wait for 1 second before sending the next trigger

    	//  Calculate distance in centimeters
    	float distance_cm = (pulse_duration * 600e-9 * 34300) / 2;  // Pulse duration (in "ticks") converted to nanoseconds (1 "tick" = 600 ns | (1/(3333333 / 2)-> distance (in cm) using speed of sound (34300 cm/s)
   	 
    	// TEST: Create if/else statement if distance_cm < 3 = red, distance_cm between 3 and 5 = both,
    	//  	between 10 and 15 = green, otherwise (out of range) -> no LEDS
    	if(distance_cm < 5.0) {
        	PORTF.OUTSET = PIN4_bm; // Turn off green LED
        	PORTF.OUTCLR = PIN5_bm; // Turn on red LED
    	}
    	else if(distance_cm > 5 && distance_cm <= 10) {
        	PORTF.OUTCLR = PIN4_bm; // Turn on green LED
        	PORTF.OUTCLR = PIN5_bm; // Turn on red LED
    	}
    	else if(distance_cm > 10 && distance_cm <= 15) {
        	PORTF.OUTCLR = PIN4_bm; // Turn on green LED
        	PORTF.OUTSET = PIN5_bm; // Turn off red LED
    	}
    	else {
        	PORTF.OUTSET = PIN4_bm; // Turn off green LED
        	PORTF.OUTSET = PIN5_bm; // Turn off red LED
    	}
	}
	return 0;
} */

/* SETTING UP MOTOR */
/*#ifndef F_CPU
#define F_CPU 3333333  // clock frequency 10 MHz
#endif

#include <avr/io.h>
#include <util/delay.h>

int main(void) {
	// Set pin C0 as output
    PORTC.DIR |= PIN0_bm;
//	PORTD.OUT |= PIN1_bm;
    
    
	// Set pre-scaler to 64
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV2_gc; // 3333333 / 2 = 1666667
    
	// Set period
	// 1,666,667 (Hz) * 0.02 (sec) = 33333 (cycles)
	TCA0.SINGLE.PER = 33333;
    
	// Enable Timer A in single-slope PWM mode and enable compare match output
	TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    
	// Begin timer
	TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
    
	// Config PORTMUX to route Timer A output to pin C0
	PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTC_gc;
    
	while(1){
    	TCA0.SINGLE.CMP0 = 1450; // = 45 degrees WAS
    	_delay_ms(5000); // Delay for 2 seconds
    	TCA0.SINGLE.CMP0 = 2550; // = neutral (135 degrees)  
    	_delay_ms(5000); // Delay for 2 seconds
    	TCA0.SINGLE.CMP0 = 3600; //  = 180 degrees WAS 3600
    	_delay_ms(5000); // Delay for 2 seconds
	}
}*/

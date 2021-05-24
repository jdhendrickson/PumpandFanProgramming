/** The final project.
* Will have two motors, one 3-6v and one 3-9v.
*   The first will be variable-speed, for use as a fan.
*   The second will be fixed speed, intended to be used for a pump.
* Has an RGB LED used for output types.
*   Blue output means the fixed-speed pump will be on
*   Green output means the variable-speed fan will be on automatic mode
*   Red output means the variable-speed fan will be on full-on mode
* Author: Joseph Hendrickson
* Date: 11-5-2020
*/

// ------- Preamble -------- //
#include <avr/io.h>        /* Defines pins, ports, etc */
#include <util/delay.h>    /* Functions to waste time */
#include "USART.h"         /* Functions for communication */
#include <util/setbaud.h>  /* Defines the baud rate */
#include <avr/interrupt.h> /* Functions for interupts */
// ------- Pin mapping -------- //
/*
PB0: Fixed-speed Pump output
PB1: Variable-motor Fan output
PD5: Red of the RGB
PD6: Green of the RGB
PD7: Blue of the RGB
*/

// ------- Globals -------- //
//Decides which state the system is in.
//  000: Everything off
//  001: Pump is on, blue LED is on
//  010: Fan is on automatic mode, green LED is on
//  100: Fan is on full-speed mode, red LED is on
//  011: Fan is on automatic mode, Pump is on, green and blue LEDs are on
//  101: Fan is on full-speed mode, Pump is on, red and blue LEDs are on
uint8_t state;

////////////////////
//Init functions below here
////////////////////

/**
* Initializes the UART
*/
void uart_init() {
  //Automatically adjust settings for BAUD rate depending on CPU clock speed and selected rate:
  #if USE_2X
    UCSR0A |= (1 << U2X0);
  #else
    UCSR0A &= ~(1 << U2X0);
  #endif

  //Set Baud Rate
  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;

  //Enable USART transmitter / receiver:
  UCSR0B |= (1 << TXEN0);
  UCSR0B |= (1 << RXEN0);
  //Set packet size (8 data bits):
  UCSR0C |= (1 << UCSZ01);
  UCSR0C |= (1 << UCSZ00);
  //Set stop bit amount (1 stop bit):
  UCSR0C &= ~(1 << USBS0);
}
/**
* Sets up the PWM1 registers
*/
void pwm_init() {
  //PWM needs to have outputs
  DDRB |= (1 << PB2);//PWM2
  //Set waveform to Fast PWM 8-bit mode;
  TCCR1A |= (1<<WGM10);
  TCCR1B |= (1<<WGM12);
  //Set the Prescaler to 8
  TCCR1B |= (1<<CS11);
  //Set to non-inverted output mode
  TCCR1A |= (1<<COM1B1);
}
/**
* A function to initialize the interrupt 0 for buttons
*/
void intr_init() {
  //Enable pullup for interrupt pins
  DDRD &= ~(1<<PD2); //Ensure that interrupt 0 pin is input
  PORTD |= 1<<PD2;  //Enable pullup
  DDRD &= ~(1<<PD3); //Ensure that interrupt 1 pin is input
  PORTD |= 1<<PD3;  //Enable pullup

  //Set interrupt 0 to trigger on button press
  EICRA |= (1 << ISC01);
  EICRA &= ~(1 << ISC00);
  //Enable interrupt 0 flag
  EIMSK |= 1<<INT0;
  //Set interrupt 1 to trigger on button press
  EICRA |= (1 << ISC11);
  EICRA &= ~(1 << ISC10);
  //Enable interrupt 1 flag
  EIMSK |= 1<<INT1;
  //Enable interrupts
  sei();
}

/**
* Sets up the ADC registers
* Note: Changes the DDRC, PRR, ADCSR and ADMUX registers
*/
void adc_init() {
  //Ensure the input of ADC0 is input
  DDRC &= ~(1<<PC5);
  //Enable power to the ADC
  PRR &= ~(1<<PRADC);
  //Set the ADCSRA
  ADCSRA = 0x00;
    //Enable the ADC
  ADCSRA |= 1<<ADEN;
    //Don't start the conversion yet
    //Enable the autotrigger
  ADCSRA |= 1<<ADATE;
    //Bit 4 is interrupt flag
    //Keep the interrupt off
    //Prescaler 8 used 011
  ADCSRA |= 0b011<ADPS0;
  //Set the ADMUX
  ADMUX = 0b00000000;      //Clear the Admux
  ADMUX |= (0b01<<REFS0);  //Ref Voltage is AVcc with externaal capaciter
  ADMUX |= (0b0101<<MUX0); //Input Pin is ADC5
  //Set the ADCSRB
    //Set the auto-trigger source
    //Freerunning is 000
  ADCSRB = 0b00000000;
  //Input on ADC5, digital buffer not needed there
  DIDR0 |= 1<<ADC5D;
  //Start the conversion
  ADCSRA |= 1<<ADSC;
}
////////////////////
//Helper functions below here
////////////////////
/**
* A function to update the fan speed values
*/
void updateFanSpeed(uint8_t in) {
  //Disable interrupts to prevent fan from restarting
  cli();
  //Convert the in to 0-255 range
  uint8_t temp = (255 * in) / 100;
  printString("Fan Speed:");
  printByte(temp);
  printString("\n\r");
  //Set the fan speed values
  OCR1A = temp;
  OCR1B = temp;
  //Reenable interrupts
  sei();
}
/**
* A function to update the LED brightness values
* LED Mapping:
*   PD5: Red of the RGB
*   PD6: Green of the RGB
*   PD7: Blue of the RGB
*/
void updateLED() {
  uint8_t temp = state;
  if(temp & 1) {//001, Blue is turned on
    PORTD |= 1<<PD7;
  } else {
    PORTD &= ~(1<<PD7);
  }
  temp = temp >> 1;
  if(temp & 1) {//010, Green is turned on
    PORTD |= 1<<PD6;
  } else {
    PORTD &= ~(1<<PD6);
  }
  temp = temp >> 1;
  if(temp & 1) {//100, Red is turned on
    PORTD |= 1<<PD5;
  } else {
    PORTD &= ~(1<<PD5);
  }
}

/**
* Returns the ADC output as an unsigned 16-bit int
*/
uint16_t adc_getOutput() {
    uint16_t out = (unsigned int) ADCL;
    out |= ((uint16_t)ADCH)<<8;
    return out;
}
/*
* Checks if the data is ready
*/
char adc_is_data_ready() {
  //Check if the ADC interrupt flag is tripped
    if((ADCSRA & (1<<ADIF)) == 1)
        return 1;
    return 0;
}
////////////////////
//Main function below here
////////////////////
int main(void) {
  pwm_init();
  intr_init();
  adc_init();
  uart_init();
  printString("\n\rRebooting\n\r");
  //LED pin enabling
  DDRD |= 1<<PD5;
  DDRD |= 1<<PD6;
  DDRD |= 1<<PD7;
  //Pump pin enabling
  DDRB |= 1<<PB0;
  state = 0;
  uint16_t adcOutput = 0;
  uint8_t percentSpeed = 0;
  updateLED();
  while(1) {//Main loop
    updateLED();//Update state indicator
    //Check if state is on automatic mode
    if(state & 0b010) {
      //Fan state is on automatic
      //Get tempurature value
      adcOutput = adc_getOutput();
      printString("ADC Output:");
      printWord(adcOutput);
      //Convert the voltage to the speed the fan should turn at.
      //Done directly due to the fact that the equation on the datasheet did not work
      //According to the datasheet, my house would be at -8C, not 21C...
      /*Table of values for the thermoresistor:
        21C: ADC output of 480
        Body temp on my fingers (~36C): ADC Output of 630
      */
      if(adcOutput <= 480) {//Ensure it's above the minimum temp
        percentSpeed = 0;
      } else { //Do the conversion
        percentSpeed = (100*(adcOutput-460))/150;
      }
      //Make sure that it didn't go over 100%
      if (percentSpeed > 100) {
        percentSpeed = 100;
      }

      //Set fan output value to converted value
      printString("; % Speed:");
      printByte(percentSpeed);
      printString("; ");
      updateFanSpeed(percentSpeed);
      _delay_ms(100);
    }
  }//End main loop
}
////////////////////
//Interrupt helper functions below here
////////////////////


////////////////////
//Interrupt functions below here
////////////////////
/**
* Button 0 Press Interrupt
*   From Fan-Off mode, sets fan to full-on mode
*   From Fan-Full-On mode, sets fan to automatic mode
*   From automatic mode, sets fan to off mode.
*/
ISR(INT0_vect) {
  ///Debouncing
  _delay_ms(20);
  //Check that the interrupt input reads 0
  if(!((PIND>>PD2) & 1)) {
    if(state & 0b010) {
      //Fan state is on automatic
      state &= 0b001;//Set mode to off
      updateFanSpeed(0);//Turn off the fan
    } else if (state & 0b100) {
      //Fan state is on full-on mode
      state &= 0b011;//Turn off the full-on mode
      state |= 0b010;//Turn on automatic mode
    } else {
      //Fan state is on off mode
      state |= 0b100;//Set the fan to full-on mode
      updateFanSpeed(100);
    }
  }//End of debouncing section
}
/**
* Button 1 Press Interrupt
*   From pump-off mode, turns the pump on.
*   From pump-on mode, turns the pump off.
*/
ISR(INT1_vect) {
  ///Debouncing
  _delay_ms(20);
  //Check that the interrupt input reads 0
  if(!((PIND>>PD3) & 1)) {
    if (state & 0b001) {//Pump is on
      state &= ~0b001;//Turn pump off
      PORTB &= ~(1<<PB0);
    } else {//Pump is off
      state |= 0b001;//Turn pump on
      PORTB |= 1<<PB0;
    }
  }//End of debouncing section
}

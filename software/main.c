#include <stdint.h>
#include <stdbool.h>

#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/atomic.h>

#include "i2c.h"
#include "ws2812/ws2812_avr.h"
#include "veml7700/veml7700.h"

#define LIGHT_SENSOR_ADDR 0x20
#define AXEL_ADDR 0x32
#define MAGNETOMETER_ADDR 0x3c
#define RTC_ADDR 0x68

#define WAKE PINB3
#define PWR_EN PORTB1

#define WAKE_UNKNOWN 0
#define	WAKE_BUTTON1 1
#define WAKE_BUTTON2 2
#define WAKE_BOTH_BUTTONS (WAKE_BUTTON1 | WAKE_BUTTON2)
#define WAKE_AXEL 4

//LEDs start from LED1 (1/3 pi, 60°) and go counter-clockwise
struct cRGB leds[12] = {{ 0 }};

bool light_sensor_present;
bool axel_magnetometer_present;

uint16_t ambient_light;

//Returns the Vcc value in mV
uint16_t get_Vcc()
{
	ADMUX = _BV(MUX3) | _BV(MUX2);			//Vcc as reference, single-ended measure of Vbg, right adjust
	ADCSRA = _BV(ADEN);						//Turn on the ADC, auto trigger disabled, interrupt disabled, prescaler = 2
	_delay_ms(1);							//Wait for 1ms, as suggested by the datasheet
	ADCSRA |= _BV(ADIF) | _BV(ADSC);		//Clear the interrupt flag and start the conversion
	loop_until_bit_is_clear(ADCSRA, ADIF);	//Wait for the end of the measurement (max 6.25 us with 8MHz clock)
	ADCSRA &= ~_BV(ADEN);					//Turn off the ADC

	return (uint32_t)1100 * 1024 / ADC;
}

uint8_t get_wake_status()
{
	ADMUX = _BV(MUX1) | _BV(MUX0);			//Vcc as reference, single-ended measure of ADC3 (PB3), right adjust
	ADCSRA = _BV(ADEN);						//Turn on the ADC, auto trigger disabled, interrupt disabled, prescaler = 2
	ADCSRA |= _BV(ADIF) | _BV(ADSC);		//Clear the interrupt flag and start the conversion
	loop_until_bit_is_clear(ADCSRA, ADIF);	//Wait for the end of the measurement (max 6.25 us with 8MHz clock)
	ADCSRA &= ~_BV(ADEN);					//Turn off the ADC

	if (ADC < 25)
		return WAKE_AXEL;
	else if (ADC < 100)
		return WAKE_BUTTON1;
	else if (ADC < 175)
		return WAKE_BUTTON2;
	else if (ADC < 250)
		return WAKE_BOTH_BUTTONS;
	else
		return WAKE_UNKNOWN;
}

//Power down the AVR
//During power down the AVR can be waken up by PCINT, INT0, WDT or USI
void power_down()
{
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);	//Set PWR_DOWN as sleep mode
    cli();							//Stop interrupts to ensure the BOD timed sequence executes as required
    sleep_enable();					//Set the SE bit
    sleep_bod_disable();			//Disable the BOD
    sei();							//Ensure interrupts enabled so we can wake up again
    sleep_cpu();					//Go to sleep

	sleep_disable();				//Wake up here
}

int main ()
{
	DDRB |= _BV(DDB1) | _BV(DDB4);	//PB1 and PB4 outputs; PB0, PB2 and PB3 inputs
	GIMSK = _BV(PCIE);				//Enable PCINT interrupts, disable INT0
	PCMSK = _BV(PCINT3);			//Enable PCINT3 interrupt (WAKE pin)
	i2c_init();

	PORTB |= _BV(PWR_EN);			//Power the LEDs

	sei();							//Enable the interrupts

	axel_magnetometer_present = i2c_detect(AXEL_ADDR);
	light_sensor_present = i2c_detect(LIGHT_SENSOR_ADDR);

	if (light_sensor_present)
		if (!veml7700_set(LIGHT_SENSOR_ADDR, true) || !veml7700_get(LIGHT_SENSOR_ADDR, &ambient_light))
			light_sensor_present = false;

	while (true) {
		//Magic

		power_down();				//Power down; after the wake ISR the program will resume the execution here
	}

	return 0;
}

//Button pressed or the axel triggered INT1
ISR(PCINT0_vect)
{
	if (bit_is_clear(PINB, WAKE))	{ //Ignore rising edges
	
	}
}


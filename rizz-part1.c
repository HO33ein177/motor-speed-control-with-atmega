/*
 * GccApplication7.c
 *
 * Created: 6/24/2024 2:19:49 AM
 * Author : Hosein
 */ 

#include <avr/io.h>
#include <util/delay.h>

// Declare your global variables here

// Voltage Reference: AREF pin
#define ADC_VREF_TYPE ((0<<REFS1) | (0<<REFS0) | (1<<ADLAR))

// Define PWM2 and CTC2 (assuming these are macros to be used in TCCR2 setup)
#define PWM2 WGM20
#define CTC2 WGM21

// Read the 8 most significant bits
// of the AD conversion result
unsigned char read_adc(unsigned char adc_input)
{
    ADMUX = adc_input | ADC_VREF_TYPE;
    // Delay needed for the stabilization of the ADC input voltage
    _delay_ms(10);
    // Start the AD conversion
    ADCSRA |= (1<<ADSC);
    // Wait for the AD conversion to complete
    while ((ADCSRA & (1<<ADIF)) == 0);
    ADCSRA |= (1<<ADIF);
    return ADCH;
}

void main(void)
{
    // Declare your local variables here

    // Input/Output Ports initialization
    DDRA = 0x00;
    PORTA = 0x00;

    DDRB = 0x00;
    PORTB = 0x00;

    DDRC = 0xFF;
    PORTC = 0x00;

    DDRD = 0xFF;
    PORTD = 0x00;

    // Timer/Counter 0 initialization
    TCCR0 = 0x00;
    TCNT0 = 0x00;
    OCR0 = 0x00;

    // Timer/Counter 1 initialization
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TCNT1H = 0x00;
    TCNT1L = 0x00;
    ICR1H = 0x00;
    ICR1L = 0x00;
    OCR1AH = 0x00;
    OCR1AL = 0x00;
    OCR1BH = 0x00;
    OCR1BL = 0x00;

    // Timer/Counter 2 initialization
    ASSR = 0x00;
    TCCR2 = (1<<PWM2) | (1<<COM21) | (0<<COM20) | (0<<CTC2) | (0<<CS22) | (1<<CS21) | (0<<CS20);
    TCNT2 = 0x00;
    OCR2 = 0x00;

    // Timer(s)/Counter(s) Interrupt(s) initialization
    TIMSK = 0x00;

    // External Interrupt(s) initialization
    MCUCR = 0x00;
    MCUCSR = 0x00;

    // USART initialization
    UCSRB = 0x00;

    // Analog Comparator initialization
    ACSR = (1<<ACD);

    // ADC initialization
    ADMUX = ADC_VREF_TYPE;
    ADCSRA = (1<<ADEN) | (1<<ADPS0);
    SFIOR = 0x00;

    // SPI initialization
    SPCR = 0x00;

    // TWI initialization
    TWCR = 0x00;

    OCR2 = 0;

    while (1)
    {
        OCR2 = read_adc(0);
        if (PINB & (1<<PINB2)) {
            PORTC |= (1<<PORTC0);
        } else {
            PORTC &= ~(1<<PORTC0);
        }
    }
}

/**
 * @author Andre Menezes de Freitas Vale
 *
 * @brief Código para a matéria de dinamica.
 * Controlar um braço articulado
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdint.h>

// O servo MG995 tem um periodo de 50 Hz
// com variancia do duty cycle
#define TOP_COUNTER_VALUE 40000

// O MG995 tem uma liberdade de 120 Graus
// Partindo do centro: -+60 Graus
#define MG995_MAX_DEGREE 4000

/// Mapeamento do Duty Cycle do PWM para o servo
#define MAP_DUTY_CYCLE(ADC_VALUE) ((ADC_VALUE << 2) + 1000)

//===================================================
//  VARIAVEIS
//===================================================

//===================================================
//  PROTOTIPOS
//===================================================

/// @brief Configuracao do Pino PWM
void PWM_setup(void);

/// @brief Configuracao do ADC
void ADC_setup(void);

/// @brief Leitura do canal ADC
/// @param pino Canal de leitura
/// @return Valor do adc em uma variavel de 16 bits
uint16_t ADC_read(uint8_t pino);

//===================================================
//  MAIN
//===================================================
int main()
{
    PWM_setup();
    ADC_setup();
    sei(); // Ativa as interrupcoes globais

    for (;;)
    {
        OCR1A = MAP_DUTY_CYCLE(ADC_read(0x00));
        OCR1B = MAP_DUTY_CYCLE(ADC_read(0x00));
    }

    cli(); // Desativa as interrupcoes globais

    return 0;
}
//===================================================
//  FUNCOES
//===================================================

void PWM_setup(void)
{
    // GPIO OC1A como output
    DDRB = (1 << DDB1) | (1 << DDB2);
    PORTB = 0x00;

    // Configura o TIMER 1 como FAST PWM com
    // configuracao do TOp em ICR1 e mudanca do
    // duty cycle no OCR1A (Prescale de 8)
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

    ICR1 = TOP_COUNTER_VALUE;

    OCR1A = MG995_MAX_DEGREE;
    OCR1B = MG995_MAX_DEGREE / 2;
}

void ADC_setup(void)
{
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    ADMUX = (1 << REFS0);
}

uint16_t ADC_read(uint8_t pino)
{
    static uint8_t adc_LSB;
    static uint8_t adc_MSB;

    ADMUX |= pino;

    ADCSRA |= (1 << ADSC);

    while (!(ADCSRA &= ~(1 << ADIF)))
        ;

    ADCSRA |= (1 << ADIF);

    adc_LSB = ADCL;
    adc_MSB = ADCH;

    ADCSRA |= (1 << ADSC);

    return (adc_MSB << 8) | adc_LSB;
}
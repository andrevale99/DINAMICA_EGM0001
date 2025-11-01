/**
 * @author Andre Menezes de Freitas Vale
 *
 * @brief Código para a matéria de dinamica.
 * Controlar um braço articulado.
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <stdint.h>
#include <stdio.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// O servo MG995 tem um periodo de 50 Hz
// com variancia do duty cycle
#define TOP_COUNTER_VALUE 40000

/// Mapeamento do Duty Cycle do PWM para o servo
#define MAP_DUTY_CYCLE(ADC_VALUE) ((ADC_VALUE << 2) + 1000)

/// Converte o valor do contador do timer para ms
#define TicksTIMER_to_MS(value) (value / 20)

// Bauda rate da comunicacao serial
#define BAUD 9600

// Calculo do UBRR, especificado no datasheet
#define MYUBRR F_CPU / 16 / BAUD - 1

#define LINEAR_A_SERVO ((float)0.0440)
#define LINEAR_B_SERVO ((float)-44.0480)

//===================================================
//  VARIAVEIS
//===================================================

volatile uint16_t i8CounterAmostragem = 0;

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

/// @brief Mapeamento do angulo do servo de theta(graus) para PWM
/// @param a valor da constante da linearizacao (a*x + b = y)
/// @param b valor da constante independente (a*x + b = y)
/// @param theta valor em graus
/// @return Retorna um valor de 16 bits do mapeamento
uint16_t angle(float a, float b, float theta);

/**
 * @brief Inicializacao do USART
 * @param ubrr Valor calculado do baud rate
 *
 * @note A definicao MYUBRR ja faz o calculo (datasheet)
 * e inicializa somento o TX.
 */
void USART_setup(unsigned int ubrr);

/**
 * @brief Transmissao de um byte pelo USART0
 * @param data caractere
 *
 * @note Essa funcao é utilizada quando nao for utilizada
 * as interrupcoes para envio de dado
 */
void USART_Transmit(unsigned char data);

ISR(TIMER1_OVF_vect);
//===================================================
//  MAIN
//===================================================
int main()
{
    PWM_setup();
    ADC_setup();
    USART_setup(MYUBRR);

    sei(); // Ativa as interrupcoes globais

    for (;;)
    {
        // OCR1A = angle(LINEAR_A_SERVO, LINEAR_B_SERVO, theta[(idx++) % 10]);
        // _delay_ms(100);
    }

    cli(); // Desativa as interrupcoes globais

    return 0;
}
//===================================================
//  FUNCOES
//===================================================

void PWM_setup(void)
{
    // GPIO OC1A e OC1B como output
    DDRB = (1 << DDB1) | (1 << DDB2);
    PORTB = 0x00;

    // Configura o TIMER 1 como FAST PWM com
    // configuracao do TOP em ICR1 e mudanca do
    // duty cycle no OCR1A e OCR1B, com o Prescale de 8
    // do clock
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

    // Interrupcao do overflow do timer 1
    TIMSK1 = (1 << TOIE1);

    ICR1 = TOP_COUNTER_VALUE;

    // numero magico para comecar no menor angulo
    // dos servos
    OCR1A = 1000;
    OCR1B = 1000;
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

uint16_t angle(float a, float b, float theta)
{
    return ((theta - b) / a);
}

void USART_setup(unsigned int ubrr)
{
    /*Set baud rate */
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;

    // Enable transmitter
    UCSR0B = (1 << TXEN0);

    /* Set frame format: 8data, 1 stop bit */
    UCSR0C = (3 << UCSZ00) | (0 << USBS0);
}

void USART_Transmit(unsigned char data)
{
    /* Wait for empty transmit buffer */
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    /* Put data into buffer, sends the data */
    UDR0 = data;
}

ISR(TIMER1_OVF_vect)
{
    if (i8CounterAmostragem > TicksTIMER_to_MS(100))
    {
    }
    i8CounterAmostragem++;
}
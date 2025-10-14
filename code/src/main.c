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

//===================================================
//  VARIAVEIS
//===================================================

//===================================================
//  PROTOTIPOS
//===================================================

/// @brief Configuraca do Pino PWM
void PWM_setup(void);

//===================================================
//  MAIN
//===================================================
int main()
{
    PWM_setup();
    sei(); // Ativa as interrupcoes globais

    for (;;)
    {
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
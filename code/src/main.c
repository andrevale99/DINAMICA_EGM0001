/**
 * @author Andre Menezes de Freitas Vale
 *
 * @brief Codigo que utiliza da interrupcao externas PCINTXX
 * para criar uma logica de semaforo simples
 *
 * @note O PWM é gerado pelo pino PB1 (OC1A) sem o auxilio de interrupcao,
 * pois a configuracao do TIMER 1 permite isso, ou seja, a mudanca ocorrera
 * de forma assincrona.
 *
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdint.h>

#define TOP_COUNTER_VALUE 40000

//===================================================
//  VARIAVEIS
//===================================================

//===================================================
//  PROTOTIPOS
//===================================================

//===================================================
//  MAIN
//===================================================
int main()
{
    DDRB = (1 << DDB1);
    PORTB = 0x00;

    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

    ICR1 = TOP_COUNTER_VALUE;

    OCR1A = 2000;

    sei(); // Ativa as interrupcoes globais

    uint16_t temp = 20000;
    for (;;)
    {
        if (temp >= 4000)
            temp = 400;
        OCR1A = temp;
        temp += 100;
        _delay_ms(100);
    }

    cli(); // Desativa as interrupcoes globais

    return 0;
}
//===================================================
//  FUNCOES
//===================================================

/* Copyright 2014, Iván Talijancic <italijancic@outlook.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief
 **
 **
 **
 **/

/** \addtogroup project
 ** @{ */
/** \addtogroup module
 ** @{ */

/*==================[inclusions]=============================================*/
#include "mcu_pwm.h"
#include "os.h"
#include "chip.h"
#include "mcu_gpio.h"
#include "stdint.h"
#include <stdio.h>
/*==================[macros and definitions]=================================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
mcu_gpio_pinId_enum PinPWM;
/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
uint32_t pwm_period;
/*==================[external functions definition]==========================*/

/** \brief
 *
 *
 */
extern void mcu_pwm_Init(void)
{
	/* Inicializo el TMR1 */
	Chip_TIMER_Init(LPC_TIMER1);
	/**
	 *  Seteo el prescaler del TIMER1
	 *  Con esta configuracion el TIMER1 se incrementa cada 1uS
	 *  */
	Chip_TIMER_PrescaleSet(LPC_TIMER1,Chip_Clock_GetRate(CLK_MX_TIMER1)/1000000 - 1);
}

/** \brief
 *
 *
 */
extern void mcu_pwm_Config(mcu_gpio_pinId_enum pin, uint32_t period)
{
	/**
	 * Paso el periodo a ms
	 */
	period = 1000*period;
	pwm_period = period;

	/* Pongo en bajo la salida PWM */
	mcu_gpio_setOut(pin, false);

	/**
	 * Cargo el pin que voy a utilizar para hacer el pwm
	 * */
	PinPWM = pin;

	/**
	 * Match 0 - Define el período
	 * */
	Chip_TIMER_MatchEnableInt(LPC_TIMER1,0);		/*Habilito la Interrupcion por match del TMR1*/
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER1,0);	/*Habilito el reset on match*/
	Chip_TIMER_StopOnMatchDisable(LPC_TIMER1,0);	/*Deshabilito el Stop on match*/
	Chip_TIMER_SetMatch(LPC_TIMER1,0,period);		/*Seteo el valor para el match 0*/

	/**
	* Match 1 - Define el duty cycle
	* */
	Chip_TIMER_MatchEnableInt(LPC_TIMER1, 1);
	Chip_TIMER_ResetOnMatchDisable(LPC_TIMER1, 1);
	Chip_TIMER_StopOnMatchDisable(LPC_TIMER1, 1);
	Chip_TIMER_SetMatch(LPC_TIMER1, 1, 0);			/*por defecto inicializo al 0%*/

	/*Reseteo el TMR1*/
	Chip_TIMER_Reset(LPC_TIMER1);
	/*Habilito el TMR1*/
	Chip_TIMER_Enable(LPC_TIMER1);
}

/** \brief
 *
 *
 */
extern void mcu_pwm_SetDutyCycle(uint32_t duty)
{
	/**
	 * Declaracion de variables locales
	 * */
	float time_high = 0;	/* Tiempo en alto de la salida PWM */

	/**
	 * Calculo el Match del Tiempo en alto del PIN
	 *
	 * */
	time_high = (float)pwm_period * ((float)duty / 100);

	/**
	 * Match 1
	 * Define el duty cycle del PWM
	 * */
	Chip_TIMER_SetMatch(LPC_TIMER1, 1, (uint32_t)time_high);

	/*Limpio los Match*/
	Chip_TIMER_ClearMatch(LPC_TIMER1, 0);
	Chip_TIMER_ClearMatch(LPC_TIMER1, 1);

	/**
	 * Ver con carlos si no combiene aqui plantear un case
	 * para ver si duty = 0, duty = 100 o el resto.
	 * */
	if(duty == 0)
	{
		/* Deshabilito la Interrupcion*/
		NVIC_DisableIRQ(TIMER1_IRQn);
		/* Pongo en bajo la salida PWM */
		mcu_gpio_setOut(PinPWM, false);
	}
	else
	{
		/* Habilito la Interrrupcion */
		NVIC_EnableIRQ(TIMER1_IRQn);
		/* Pongo en alto la salida PWM */
		mcu_gpio_setOut(PinPWM, true);
	}
}

/** \brief
 *
 *	Interrupcion por match en el TMR1
 */
ISR(TMR1_IRQHandler)
{
	/*Si la Interrupcion fue en el Match 0*/
	if (Chip_TIMER_MatchPending(LPC_TIMER1, 0) && !Chip_TIMER_MatchPending(LPC_TIMER1, 1))
	{
		Chip_TIMER_ClearMatch(LPC_TIMER1, 0);
	    mcu_gpio_setOut(PinPWM, true);
	}

	/*Si la Interrupcion fue en el Match 1*/
	if (Chip_TIMER_MatchPending(LPC_TIMER1, 1) && !Chip_TIMER_MatchPending(LPC_TIMER1, 0))
	{
		Chip_TIMER_ClearMatch(LPC_TIMER1, 1);
		mcu_gpio_setOut(PinPWM, false);
	}

	/* Si se dan las dos interrupciones en simultaneo el duty = 100  */
	if(Chip_TIMER_MatchPending(LPC_TIMER1, 0) && Chip_TIMER_MatchPending(LPC_TIMER1, 1))
	{
		Chip_TIMER_ClearMatch(LPC_TIMER1, 0);
		Chip_TIMER_ClearMatch(LPC_TIMER1, 1);
		mcu_gpio_setOut(PinPWM, true);
	}
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

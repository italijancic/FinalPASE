/* Copyright 2014, Your Name <youremail@domain.com>
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

/** \brief Short description of this file
 **
 ** Long description of this file
 **
 **/

/** \addtogroup project
 ** @{ */
/** \addtogroup module
 ** @{ */

/*==================[inclusions]=============================================*/
#include "mcu_timestamp.h"
#include "os.h"
#include "chip.h"
#include "stdint.h"
#include "limits.h"


/*==================[macros and definitions]=================================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

/** \brief
 *
 *
 */
extern void mcu_timestamp_Init(void)
{
//	/* Inicializo el TMR1 */
//	Chip_TIMER_Init(LPC_TIMER2);
//
//	/**
//	 *  Seteo el prescaler del TIMER2
//	 *  Con esta configuracion el TIMER2 se incrementa cada 1uS
//	 *  */
//	Chip_TIMER_PrescaleSet(LPC_TIMER2,Chip_Clock_GetRate(CLK_MX_TIMER2)/1000000 - 1);
//
//	/**
//	 * Match 0
//	 * */
//	Chip_TIMER_MatchEnableInt(LPC_TIMER2,0);		/* Habilito la Interrupcion por match 0 del TMR2 */
//	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER2,0);	/* Habilito el reset on match */
//	Chip_TIMER_StopOnMatchDisable(LPC_TIMER2,0);	/* Deshabilito el Stop on match */
//	/* Configuro para que genere una
//	 * interrupcion cada 1ms */
//	Chip_TIMER_SetMatch(LPC_TIMER2,0,1000);			/* Seteo el valor para el match 0 en us */
//
//	/* Reseteo el TMR2 */
//	Chip_TIMER_Reset(LPC_TIMER2);
//	/* Habilito el TMR2 */
//	Chip_TIMER_Enable(LPC_TIMER2);
//
//	/* Habilito la Interrrupcion */
//	NVIC_EnableIRQ(TIMER2_IRQn);
}

/** \brief
 *
 *
 */
extern char* mcu_timestamp_GetTimestamp(reloj stclock1)
{
	/*
	 * Declaración de varaibles locales
	 * */
	static char strMseg[10];
	static char strSeg[20];
	static char strMin[20];
	static char strHs[50];
	static char strOut[150];
	int i =0;

	/* Limpio el buffer de salida */
	for (i = 0; i < strlen(strOut);i++)
		strOut[i] = '\0';

	/* Paso los enteros a ascii*/
	itoa(stclock1.mseg,strMseg,10);
	itoa(stclock1.seg,strSeg,10);
	itoa(stclock1.min,strMin,10);
	itoa(stclock1.hs,strHs,10);

	/* Concateno strings para formar el mensaje de salida */
	strcat(strOut,strHs);
	strcat(strOut,":");
	strcat(strOut,strMin);
	strcat(strOut,":");
	strcat(strOut,strSeg);
	strcat(strOut,":");
	strcat(strOut,strMseg);

	return (char *)strOut;
}

/** \brief
 *
 *	Interrupcion por match en el TMR2
 *	Entramos cada 1ms
 */
ISR(TMR2_IRQHandler)
{
//	/* Incremento el contador de mseg */
//	stclock.mseg++;
//	/* Si paso 1seg */
//	if(stclock.mseg >= 1000)
//	{
//		stclock.mseg = 0;
//		/* Incremento el contador de segundos */
//		stclock.seg++;
//		/* Si paso un minuto */
//		if(stclock.seg >= 60)
//		{
//			stclock.seg = 0;
//			/* Incremento el contador de minutos */
//			stclock.min++;
//			/* Si paso una hora */
//			if(stclock.min >= 60)
//			{
//				stclock.min = 0;
//				stclock.hs++;
//				if(stclock.hs >= LONG_MAX)
//					stclock.hs = 0;
//			}
//		}
//	}
//
//	Chip_TIMER_ClearMatch(LPC_TIMER2, 0);
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

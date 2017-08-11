/* Copyright 2017, Gustavo Muro
 *
 * This file is part of CIAA Firmware.
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

/** \brief PASE APP EXAMPLE
 **
 ** ejemplo de aplicación usando CIAA Firmware
 **
 **/

/** \addtogroup
 ** @{ */
/** \addtogroup
 ** @{ */
/** \addtogroup
 ** @{ */

/*==================[inclusions]=============================================*/
#include "os.h"
#include "pase_app_example.h"
#include "mcu.h"
#include "bsp.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
/*==================[macros and definitions]=================================*/

enum{
	STOP_STATE = 0,
	PLAY_STATE,
	PAUSE_STATE
}progam_state;

/*==================[internal data declaration]==============================*/
uint32_t duty=0;
uint8_t state_sec=0;
uint8_t flg_play = 0;
char str[50];
char *str_timestamp;
/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */
int main(void)
{
   /* Starts the operating system in the Application Mode 1 */
   /* This example has only one Application Mode */
   StartOS(AppMode1);

   /* StartOs shall never returns, but to avoid compiler warnings or errors
    * 0 is returned */
   return 0;
}

/** \brief Error Hook function
 *
 * This fucntion is called from the os if an os interface (API) returns an
 * error. Is for debugging proposes. If called this function triggers a
 * ShutdownOs which ends in a while(1).
 *
 * The values:
 *    OSErrorGetServiceId
 *    OSErrorGetParam1
 *    OSErrorGetParam2
 *    OSErrorGetParam3
 *    OSErrorGetRet
 *
 * will provide you the interface, the input parameters and the returned value.
 * For more details see the OSEK specification:
 * http://portal.osek-vdx.org/files/pdf/specs/os223.pdf
 *
 */
void ErrorHook(void)
{
   ShutdownOS(0);
}

/** \brief Initial task
 *
 * This task is started automatically in the application mode 1.
 */
TASK(InitTask)
{
   /*Tarea Inicial*/
   bsp_init();

   /*Seteo alarma que activa la tarea que atiende al teclado*/
   SetRelAlarm(ActivateKeyboardTask, 10, KEYBOARD_TASK_TIME_MS);

   /*Seteo alarma que activa la tarea definida por el usuario*/
   SetRelAlarm(ActivateUserTask, 50, 50);

   /*Configuro el modulo de pwm*/
   mcu_pwm_Config(MCU_GPIO_PIN_ID_75,2);

   /*Seteo el DutyCycle del PWM en 0% */
   mcu_pwm_SetDutyCycle(0);

   progam_state = STOP_STATE;

   TerminateTask();
}

/** \brief SecuenciaTask
 * Tarea para hacerla secuencia de encendido
 * y apagado de los LEDs
 * */
TASK(SecuenciaTask)
{
	/** \brief State Machine for led actions
	 * Máquina de estado para generar la secuencia requerida
	 * */
	switch(state_sec)
	{
		case 0:
			if(duty == 0)
			{
				/* Obtengo el timestamp */
				str_timestamp = mcu_timestamp_GetTimestamp();

				/* Imprimo msj por la UART */
				sprintf(str,": Encendiendo Led Rojo \n\r");
				strcat(str_timestamp ,str);
				if(progam_state != STOP_STATE)
					mcu_uart_write(str_timestamp, strlen(str_timestamp));
			}
			if(duty < 100)
			{
				mcu_pwm_SetDutyCycle(duty);
				duty += 1;
			}
			/* Llego a la máxima intensidad */
			else
			{
				/* Obtengo el timestamp */
				str_timestamp = mcu_timestamp_GetTimestamp();

				/* Imprimo msj por la UART */
				sprintf(str,": Intensidad Máxima Led Rojo \n\r");
				strcat(str_timestamp ,str);
				mcu_uart_write(str_timestamp, strlen(str_timestamp));

				/* Incremento el estado en la SM */
				state_sec = 1;
			}
			break;

		case 1:
			if(duty >= 1)
			{
				mcu_pwm_SetDutyCycle(duty);
				duty -= 1;
			}
			/* Llego a la minima intensidad */
			else
			{
				/* Modifico el el pin del PWM */
				mcu_pwm_Config(MCU_GPIO_PIN_ID_81,2);
				/*Seteo el DutyCycle del PWM en 0% */
				mcu_pwm_SetDutyCycle(duty);

				/* Obtengo el timestamp */
				str_timestamp = mcu_timestamp_GetTimestamp();
				/* Imprimo msj por la UART */
				sprintf(str,": Encendiendo Led Verde \n\r");
				strcat(str_timestamp ,str);
				mcu_uart_write(str_timestamp, strlen(str_timestamp));

				/* Incremento el estado en la SM */
				state_sec = 2;
			}
			break;

		case 2:
			if(duty < 100)
			{
				mcu_pwm_SetDutyCycle(duty);
				duty += 1;
			}
			/* Llego a la máxima intensidad */
			else
			{
				/* Obtengo el timestamp */
				str_timestamp = mcu_timestamp_GetTimestamp();
				/* Imprimo msj por la UART */
				sprintf(str,": Intensidad Máxima Led Verde \n\r");
				strcat(str_timestamp ,str);
				mcu_uart_write(str_timestamp, strlen(str_timestamp));
				/* Incremento el estado en la SM */
				state_sec = 3;
			}
			break;

		case 3:
			if(duty >= 1)
			{
				mcu_pwm_SetDutyCycle(duty);
				duty -= 1;
			}
			/* Llego a la minima intensidad */
			else
			{
				/* Modifico el el pin del PWM */
				mcu_pwm_Config(MCU_GPIO_PIN_ID_84,2);
				/*Seteo el DutyCycle del PWM en 0% */
				mcu_pwm_SetDutyCycle(duty);

				/* Obtengo el timestamp */
				str_timestamp = mcu_timestamp_GetTimestamp();
				/* Imprimo msj por la UART */
				sprintf(str,": Encendiendo Led Azul \n\r");
				strcat(str_timestamp ,str);
				mcu_uart_write(str_timestamp, strlen(str_timestamp));
				/* Incremento el estado en la SM */
				state_sec = 4;
			}
			break;

		case 4:
			if(duty < 100)
			{
				mcu_pwm_SetDutyCycle(duty);
				duty += 1;
			}
			/* Llego a la máxima intensidad */
			else
			{
				/* Obtengo el timestamp */
				str_timestamp = mcu_timestamp_GetTimestamp();
				/* Imprimo msj por la UART */
				sprintf(str,": Intensidad Máxima Led Azul \n\r");
				strcat(str_timestamp ,str);
				mcu_uart_write(str_timestamp, strlen(str_timestamp));
				/* Incremento el estado en la SM */
				state_sec = 5;
			}
			break;

		case 5:
			if(duty >= 1)
			{
				mcu_pwm_SetDutyCycle(duty);
				duty -= 1;
			}
			/* Llego a la minima intensidad */
			else
			{
				/* Modifico el el pin del PWM */
				mcu_pwm_Config(MCU_GPIO_PIN_ID_75,2);
				/*Seteo el DutyCycle del PWM en 0% */
				mcu_pwm_SetDutyCycle(duty);
				/* Reinicializo la maquina de estados */
				state_sec = 0;
			}
			break;

		default:
			/* None */
			break;
	}

	/* Finalizo la tarea */
	TerminateTask();
}

/** \brief UserTask
 *	Tarea  que ejecuto cada 50ms para procesar la ultima
 *	tecla  precionada
 */
TASK(UserTask)
{
	/**
	 * Declaracion de variables locales
	 * */
	int32_t key;

    /**
     * Obtengo la ultima tecla presionada
     * */
    key = bsp_keyboardGet();

    /* Maquina de estado, para los estados del programa */
    switch(progam_state)
    {
    	/* Estado de stop */
    	case STOP_STATE:

    		/* Si se preciona la tecla play/stop */
    		if(key == BOARD_TEC_ID_1)
    		{
    			SetRelAlarm(ActivateSecuenciaTask, 0, 20);
    			/* Obtengo el timestamp */
    			str_timestamp = mcu_timestamp_GetTimestamp();
    			/* Imprimo msj por la UART */
    			sprintf(str,": Inicio Secuencia \n\r");
    			strcat(str_timestamp ,str);
    			mcu_uart_write(str_timestamp, strlen(str_timestamp));
    			/* Cambio de estado*/
    			progam_state = PLAY_STATE;
    		}
    		break;

    	case PLAY_STATE:

    		/* Si se preciona la tecla de play/stop */
    		if(key == BOARD_TEC_ID_1)
    		{
    			CancelAlarm(ActivateSecuenciaTask);
    			/* Reinicializo la  maquina de estado de secuencia led */
    			state_sec = 0;
    			/* Modifico el el pin del PWM */
  		   		duty = 0;
        		mcu_pwm_Config(MCU_GPIO_PIN_ID_75,2);
        		mcu_pwm_SetDutyCycle(duty);
    			/* Pongo en bajo todas las salidas*/
    			bsp_ledAction(BOARD_LED_ID_0_R,BOARD_LED_STATE_OFF);
    			bsp_ledAction(BOARD_LED_ID_0_G,BOARD_LED_STATE_OFF);
    			bsp_ledAction(BOARD_LED_ID_0_B,BOARD_LED_STATE_OFF);
    			/* Obtengo el timestamp */
    			str_timestamp = mcu_timestamp_GetTimestamp();
    			/* Imprimo msj por la UART */
    			sprintf(str,": Secuencia Finalizada \n\r");
    			strcat(str_timestamp ,str);
    			mcu_uart_write(str_timestamp, strlen(str_timestamp));
    			/* Cambio de estado*/
    			progam_state = STOP_STATE;
    		}

    		/* Si se preciona la tecla de pause/resume */
    		if(key == BOARD_TEC_ID_2)
    		{
    			CancelAlarm(ActivateSecuenciaTask);
    			/* Obtengo el timestamp */
    			str_timestamp = mcu_timestamp_GetTimestamp();
    			/* Imprimo msj por la UART */
    			sprintf(str,": Secuencia Pausada \n\r");
    			strcat(str_timestamp ,str);
    			mcu_uart_write(str_timestamp, strlen(str_timestamp));
    			/* Cambio de estado*/
    			progam_state = PAUSE_STATE;
    		}
    		break;

    	case PAUSE_STATE:
    		/* Si se preciona la tecla de play/stop */
    		if(key == BOARD_TEC_ID_1)
    		{
    			/* Reinicializo la  maquina de estado de secuencia led */
    			state_sec = 0;
    			/* Modifico el el pin del PWM */
  		   		duty = 0;
        		mcu_pwm_Config(MCU_GPIO_PIN_ID_75,2);
        		mcu_pwm_SetDutyCycle(duty);
    			/* Pongo en bajo todas las salidas*/
    			bsp_ledAction(BOARD_LED_ID_0_R,BOARD_LED_STATE_OFF);
    			bsp_ledAction(BOARD_LED_ID_0_G,BOARD_LED_STATE_OFF);
    			bsp_ledAction(BOARD_LED_ID_0_B,BOARD_LED_STATE_OFF);
    			/* Obtengo el timestamp */
    			str_timestamp = mcu_timestamp_GetTimestamp();
    			/* Imprimo msj por la UART */
    			sprintf(str,": Secuencia Finalizada \n\r");
    			strcat(str_timestamp ,str);
    			mcu_uart_write(str_timestamp, strlen(str_timestamp));
    			/* Cambio de estado*/
    			progam_state = STOP_STATE;
    		}

    		/* Si se preciona la tecla de pause/resume */
    		if(key == BOARD_TEC_ID_2)
    		{
    			SetRelAlarm(ActivateSecuenciaTask, 0, 20);
    			/* Obtengo el timestamp */
    			str_timestamp = mcu_timestamp_GetTimestamp();
    			/* Imprimo msj por la UART */
    			sprintf(str,": Secuencia Reanudada \n\r");
    			strcat(str_timestamp ,str);
    			mcu_uart_write(str_timestamp, strlen(str_timestamp));
    			/* Cambio de estado*/
    			progam_state = PLAY_STATE;
    		}

    		break;

    	default:
    		break;
    }

    /* Fin tarea */
    TerminateTask();
}

/** \brief KeyboardTask
 *
 * Tarea periodica que se ejecuta cada 2ms
 * y se encarga de hacer el poolin de las teclas
 * */
TASK(KeyboardTask)
{
    bsp_keyboard_task();

    /* Fin tarea */
    TerminateTask();
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/


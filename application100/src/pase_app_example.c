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

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/
int32_t duty=0;
uint8_t state_sec=0;
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

   TerminateTask();
}

/**
 * \brief ALARMCALLBACK
 *
 * */
ALARMCALLBACK(CallBack2ms)
{
	ActivateTask(SecuenciaTask);
}

/** \brief SecuenciaTask
 * Tarea para hacerla secuencia de encendido
 * y apagado de los LEDs
 * */
TASK(SecuenciaTask)
{
	/**
	 * Declaracion de variables locales
	 * */
	static uint32_t duty = 0;
	static unsigned long timestamp = 0;
	static char str[40];

	/** \brief State Machine for led actions
	 * Máquina de estado para generar la secuencia requerida
	 * */
	switch(state_sec)
	{
		case 0:
			if(duty == 0)
			{
				/* Obtengo el timestamp */

				sprintf(str,"TIMESTAMP: Encendiendo Led Rojo \n\r");
				/* Imprimo msj por la UART */
				mcu_uart_write(str, strlen(str));
			}
			if(duty < 100)
			{
				duty += 1;
				mcu_pwm_SetDutyCycle(duty);
			}
			/* Llego a la máxima intensidad */
			else
			{
				/* Obtengo el timestamp */

				sprintf(str,"TIMESTAMP: Intencidad Máxima Led Rojo \n\r");
				/* Imprimo msj por la UART */
				mcu_uart_write(str, strlen(str));
				/* Incremento el estado en la SM */
				state_sec = 1;
			}
			break;

		case 1:
			if(duty >= 1)
			{
				duty -= 1;
				mcu_pwm_SetDutyCycle(duty);
			}
			/* Llego a la minima intensidad */
			else
			{
				/* Modifico el el pin del PWM */
				mcu_pwm_Config(MCU_GPIO_PIN_ID_81,2);
				/*Seteo el DutyCycle del PWM en 0% */
				mcu_pwm_SetDutyCycle(duty);

				/* Imprimo msj por la UART */

				/* Incremento el estado en la SM */
				state_sec = 2;
			}
			break;

		case 2:
			if(duty < 100)
			{
				duty += 1;
				mcu_pwm_SetDutyCycle(duty);
			}
			/* Llego a la máxima intensidad */
			else
			{
				/* Imprimo msj por la UART */

				/* Incremento el estado en la SM */
				state_sec = 3;
			}
			break;

		case 3:
			if(duty >= 1)
			{
				duty -= 1;
				mcu_pwm_SetDutyCycle(duty);
			}
			/* Llego a la minima intensidad */
			else
			{
				/* Modifico el el pin del PWM */
				mcu_pwm_Config(MCU_GPIO_PIN_ID_84,2);
				/*Seteo el DutyCycle del PWM en 0% */
				mcu_pwm_SetDutyCycle(duty);

				/* Imprimo msj por la UART */

				/* Incremento el estado en la SM */
				state_sec = 4;
			}
			break;

		case 4:
			if(duty < 100)
			{
				duty += 1;
				mcu_pwm_SetDutyCycle(duty);
			}
			/* Llego a la máxima intensidad */
			else
			{
				/* Imprimo msj por la UART */

				/* Incremento el estado en la SM */
				state_sec = 5;
			}
			break;

		case 5:
			if(duty >= 1)
			{
				duty -= 1;
				mcu_pwm_SetDutyCycle(duty);
			}
			/* Llego a la minima intensidad */
			else
			{
				/* Modifico el el pin del PWM */
				mcu_pwm_Config(MCU_GPIO_PIN_ID_75,2);
				/*Seteo el DutyCycle del PWM en 0% */
				mcu_pwm_SetDutyCycle(duty);

				/* Imprimo msj por la UART */

				/* Incremento el estado en la SM */
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
	TaskStateType state;
	static uint8_t cont_tec_1 = 0;
	static uint8_t cont_tec_2 = 0;
    static char str[40];

    /**
     * Obtengo la ultima tecla presionada
     * */
    key = bsp_keyboardGet();

    /* TECLA 1
     * Da inicio y la finaliza
     * LED RGB
     * */
    if (key == BOARD_TEC_ID_1)
    {
    	/* Obtengo el Estado de la Tare de Secuencia de LEDs */
    	//GetTaskState(SecuenciaTask, &state);

    	if(cont_tec_1 == 0)
    	{
    		SetRelAlarm(ActivateSecuenciaTask, 0, 20);
    		//SetRelAlarm(RunCallBack2ms,0 , 20);

    		/* Imprimo msj por la UART */
    		sprintf(str,"TIMESTAMP: Inicio Secuencia \n\r");
    		mcu_uart_write(str, strlen(str));
    		cont_tec_1 = 1;
    	}
    	else
    	{
    		CancelAlarm(ActivateSecuenciaTask);
    		/* Reinicializo la  maquina de estado de secuencia led */
    		state_sec = 0;
    		duty = 0;
    	}
    }

    /* TECLA 2
     * Pausa o reinicia la secuencia de los LEDs
     * LE RGB
     * */
    if (key == BOARD_TEC_ID_2)
    {
    	if(cont_tec_2 == 0 && cont_tec_1 == 0)
    	{
    		SetRelAlarm(ActivateSecuenciaTask, 0, 20);

    		/* Imprimo msj por la UART */
    	    sprintf(str,"TIMESTAMP: Inicio Secuencia \n\r");
    	    mcu_uart_write(str, strlen(str));

    	    cont_tec_2 = 1;
    	 }
    	 else
    	 {
    		 CancelAlarm(ActivateSecuenciaTask);
    		 cont_tec_2 = 0;
    		 cont_tec_1 = 0;
    	}
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


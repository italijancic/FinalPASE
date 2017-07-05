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

   /*Test Funcionamiento UART*/
   char str[] = "Hello world! This is EDU-CIAA-NXP! \n\r";
   mcu_uart_write((uint8_t)str, strlen(str));

   /*Seteo alarma que activa la tarea que atiende al teclado*/
   SetRelAlarm(ActivateKeyboardTask, 10, KEYBOARD_TASK_TIME_MS);

   /*Seteo alarma que activa la tarea definida por el usuario*/
   SetRelAlarm(ActivateUserTask, 50, 50);

   /*Configuro el modulo de pwm*/
   mcu_pwm_Config(MCU_GPIO_PIN_ID_84,1000);
   /*Seteo el DutyCycle y DISPARO el pwm*/
   mcu_pwm_SetDutyCycle(500);

   TerminateTask();
}

/** \brief UserTask
 *
 *
 */
TASK(UserTask)
{
   int32_t key;

   key = bsp_keyboardGet();

   /* TECLA 1
    * Aumento intensidad LED
    * */
   if (key == BOARD_TEC_ID_1)
   {
	   bsp_ledAction(BOARD_LED_ID_0_R, BSP_LED_ACTION_TOGGLE);

	   if(duty < 1000)
		   duty += 100;
	   else
		   duty = 0;

	   mcu_pwm_SetDutyCycle(duty);
   }

   /* TECLA 2
    * Disminuye intensidad LED
    * */
   if (key == BOARD_TEC_ID_2)
   {
	   bsp_ledAction(BOARD_LED_ID_0_G, BSP_LED_ACTION_TOGGLE);

	   if(duty > 100)
		   duty -= 100;
	   else
	       duty = 1000;

	   mcu_pwm_SetDutyCycle(duty);
   }

   /* TECLA 3
    * Toggle LED 2
    * */
   if (key == BOARD_TEC_ID_3)
   {
      static char state = 0;

      state = 1-state;

      if (state)
         bsp_ledAction(BOARD_LED_ID_2, BSP_LED_ACTION_ON);
      else
         bsp_ledAction(BOARD_LED_ID_2, BSP_LED_ACTION_OFF);
   }

   /* TECLA 4
    * Si presiono la tecla por mas de 2seg prendo el led 3
    * por el contrario, si dejo de apretarla se apaga
    * */
   if (bsp_keyboardGetPressed(BOARD_TEC_ID_4, 10))
      bsp_ledAction(BOARD_LED_ID_3, BSP_LED_ACTION_ON);
   else
      bsp_ledAction(BOARD_LED_ID_3, BSP_LED_ACTION_OFF);


   TerminateTask();
}

/** \brief KeyboardTask
 *
 * Tarea periodica que se ejecuta cada 2ms
 * y se encarga de las tareas referidas al teclado
 * */
TASK(KeyboardTask)
{

    bsp_keyboard_task();

    TerminateTask();
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/


/* Copyright 2017, Iván Talijancic <italijancic@outlook.com>
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

#ifndef MCU_PWM_H
#define MCU_PWM_H
/** \brief Header para mcu_pwm.h
 **
 ** Declaración de funciones (prototipos),
 ** para el módulo pwm en la capa mcu
 **
 **/

/** \addtogroup project
 ** @{ */
/** \addtogroup module
 ** @{ */

/*==================[inclusions]=============================================*/
#include "chip.h"
#include "mcu_gpio.h"
#include "stdint.h"
#include "ciaaPOSIX_stdbool.h"
#include <stdio.h>
/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
/**
 * \brief labels para definir maximos y minimos del pwm
 * */
#define MAX_PWM_DUTY	1000
#define MIN_PWM_DUTY	1

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/** \brief Función de inicializacion del módulo
 * 		   PWM.
 * 		   La misma devuelve true si se inicializo
 * 		   correctamente el mpodulo
 *
 ** \param none
 **
 ** \return true or false
 **/
extern bool mcu_pwm_Init(void);

/** \brief Funcion de condiguración del
 * 		   mpodulo pwm.
 * 		   Configura el módilo y devuelve true
 * 		   si se configuro con exito, o false en caso
 * 		   contrario.
 *
 ** \param pin: Pin que voy a utilizar como salida PWM
 ** \param period: Periodo del PWM
 **
 ** \return true or false
 **/
extern bool mcu_pwm_Config(mcu_gpio_pinId_enum pin, uint32_t period);

/** \brief Funcion para setear el duty cycle o
 * 		   ciclo de trabajo del PWM
 *
 ** \param none
 **
 ** \return true or false
 **/
extern bool mcu_pwm_SetDutyCycle(uint32_t duty);

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
}
#endif
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef TEMPLATE_FILE_H */


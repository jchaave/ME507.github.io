/** @file motorcontrol.h
 *      This file contains the header file for the motorcontrol class
 *  @author  Peyton Ulrich, Jose Chavez, Matt Tagupa
 *  @date 2020-Nov-25    Original File
 */

#include "Arduino.h"

/** @brief  Constructor for MotorControl object
 * @details This object saves the pins that connect to the motor driver
 * @param   EN  pin on STM32 that connects to EN pin on VNH5019
 * @param   IN_A pin on STM32 that connects to INA pin on VNH5019
 * @param   IN_B pin on STM32 that connects to INB pin on VNH5019
 * @param   PWM pin on STM32 that connects to PWM pin on VNH5019
 */
class MotorControl
{
    protected:
        uint32_t EN_stored; //stores pin on STM32 that connects to EN on VNH5019
        uint32_t IN_A_stored; //stores pin on STM32 that connects to INA on VNH5019
        uint32_t IN_B_stored; //stores pin on STM32 that connects to INB on VNH5019
        uint32_t PWM_stored; //stores pin on STM32 that connects to PWM on VNH5019


    public:
        MotorControl (uint32_t EN, uint32_t IN_A, uint32_t IN_B, uint32_t PWM); //object of a motordriver
        void runMotor (uint32_t duty_cycle, bool clockwise);    //method to run the motor
};
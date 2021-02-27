/** @file motorcontrol.cpp
 *      This file contains a class that runs a motor through a VNH5019 motor driver.
 *      Code to use VNH5019 motor driver chip based on truth table in VNH5019 datasheet.
 *      https://www.pololu.com/file/0J504/vnh5019.pdf
 *  @author  Peyton Ulrich, Jose Chavez, Matt Tagupa
 *  @date 2020-Nov-25    Original File
*/

#include <Arduino.h>
#include <PrintStream.h>
#include "motorcontrol.h"

/** @brief  Constructor for MotorControl object
 * @details This object saves the pins that connect to the motor driver
 * @param   EN  pin on STM32 that connects to EN pin on VNH5019
 * @param   IN_A pin on STM32 that connects to INA pin on VNH5019
 * @param   IN_B pin on STM32 that connects to INB pin on VNH5019
 * @param   PWM pin on STM32 that connects to PWM pin on VNH5019
 */
MotorControl::MotorControl (uint32_t EN, uint32_t IN_A, uint32_t IN_B, uint32_t PWM){
    pinMode (EN, OUTPUT);   //sets pin on STM32 that connects to EN on VNH5019 as OUTPUT
    pinMode (IN_A, OUTPUT); //sets pin on STM32 that connects to INA on VNH5019 as OUTPUT
    pinMode (IN_B, OUTPUT); //sets pin on STM32 that connects to INB on VNH5019 as OUTPUT
    pinMode (PWM, OUTPUT); //sets pin on STM32 that connects to PWM on VNH5019 as OUTPUT

    //stores the pins on STM32 that are used
    EN_stored = EN; 
    IN_A_stored = IN_A;
    IN_B_stored = IN_B;
    PWM_stored = PWM;
}

/** @brief  Method in motorControl object that runs the motor
 * @details Runs the motor at a desired speed given a duty cycle and direction (clockwise vs ccw)
 * @param   duty_cycle duty cycle to run motor at
 * @param   clockwise Whether the motor should run clockwise (true) or counterclockwise (false)
 */
void MotorControl::runMotor(uint32_t duty_cycle, bool clockwise){
    
    digitalWrite (EN_stored, HIGH); // EN SET HIGH
    

    if(duty_cycle == 0){
        //INA SET HIGH, INB SET HIGH to brake motor
        digitalWrite (IN_A_stored, HIGH);
        digitalWrite (IN_B_stored, HIGH);
    }
    else{
        if(clockwise){
            //INA SET HIGH, INB SET LOW to run clockwise
            digitalWrite (IN_A_stored, HIGH);
            digitalWrite (IN_B_stored, LOW);
        }
        else{
            //INA SET LOW, INB SET HIGH to run clockwise
            digitalWrite (IN_A_stored, LOW);
            digitalWrite (IN_B_stored, HIGH);
        }
    }
    analogWrite (PWM_stored, duty_cycle);   //write PWM value to pin stored as PWM pin
    
}
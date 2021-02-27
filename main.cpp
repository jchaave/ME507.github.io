/** @file main.cpp
 *    This file contains a program containing 4 tasks to run a collision avoidance car.
 *    The car takes in inputs from an ultrasonic sensor and infrared sensor to detect objects.
 *    The car is driven by two motors connected to the back two wheels of the car.
 *    This program uses the sensors, processes sensor data, and determines how to run the two motors.
 *    Task structure based on framework provided by Prof. J.R. Ridgeley
 *    IR Sensor Code Calculations based on sample code at https://github.com/sparkfun/simple_sketches/blob/master/sharp/sharp.ino
 *    US Sensor Code Calculations based on sample code at https://cdn-learn.adafruit.com/downloads/pdf/distance-measurement-ultrasound-hcsr04.pdf
 *
 *  @author  Peyton Ulrich, Jose Chavez, Matt Tagupa
 * 
 *  @date    10 Nov 2020 Original file
 *  @date    01 Dec 2020 Updated file
 */

#include <Arduino.h>    //Arduino header file
#include <PrintStream.h>    //Printstream header file
#include <STM32FreeRTOS.h>  //FreeRTOS header file
#include "taskshare.h"      //taskshare header file
#include "motorcontrol.h"   //motor control header file


// Shares that carry information between tasks
Share<float_t> US_distance ("cm");  //carries ultasonic distance reading from sensor_scan task to process_sensor_data task
Share<float_t> IR_distance ("cm");  //carries infrared distance reading from sensor_scan task to process_sensor_data task
Share<int32_t> M1_duty_cycle ("PWM");   //carries PWM duty cycle from process_sensor_data task to RS_Motor1 task
Share<int32_t> M2_duty_cycle ("PWM");   //carries PWM duty cycle from process_sensor_data task to RS_Motor2 task
Share<bool> turn ("Turn");  //carries from process_sensor_data task to RS_Motor1 task whether the car shall be turning




 /**@brief   Task which processes sensor distance data and determines motor actions
 *  @details This task takes in distance readings from IR and US sensors and determines the correct
 *           action from the vehicle. It then places two PWM duty cycles to shares which will be carried
 *           to the RS_Motor1/RS_Motor2 tasks.  
 *  @param   p_params A pointer to function parameters which we don't use.
 */
 
void task_process_sensor_data (void* p_params)
{
    (void)p_params;            // Shuts up a compiler warning

    float us_distance;  //holds the distance reading from ultrasonic sensor
    float ir_distance;  //holds the distance reading from infrared sensor
    int full_speed = 255; //sets PWM duty cycle (as an integer from 0-255) for full speed operation of vehicle
    int partial_speed = 200; //sets PWM duty cycle (as an integer from 0-255) for partial speed operation of vehicle
    int state;  //current state of vehicle, 0 = running (ie. moving), 1 = stopped

    for (;;)
    {
        US_distance.get(us_distance);   //gets us distance reading from sensor_scan task
        IR_distance.get(ir_distance);   //gets ir distance reading from sensor_scan task
        if(us_distance > 40){   //If the distance is over 40cm (ie. far), keep running at full speed
            //Serial << "Keep Running" << endl; //code used to simulate motor actions in Serial Monitor
            state = 0;  //sets state to show that vehicle is running
            M1_duty_cycle.put(full_speed); //run M1 motor at 'full speed'
            M2_duty_cycle.put(full_speed); //run M2 motor at 'full speed'
            turn.put(false);    //place into turn share that vehicle shall not turn
        }
        else{   //If the distance is under 40cm (ie. near), use IR sensor to decide what to do
            if (ir_distance > 20){  //If the distance is over 20cm, continue to run motor at a reduced speed
                //Serial << "Slow down!!" << endl; //code used to simulate motor actions in Serial Monitor
                state = 0; //sets state to show that vehicle is running
                M1_duty_cycle.put(partial_speed); //run M1 motor at 'partial speed'
                M2_duty_cycle.put(partial_speed); //run M2 motor at 'partial speed'
                turn.put(false);    //place into turn share that vehicle shall not turn
            }
            else{   //If the distance is under 20cm, decide what to do
                if(state){  //If the car is already stopped, attempt to turn the vehicle
                    //Serial << "Turn!!" << endl; //code used to simulate motor actions in Serial Monitor
                    M1_duty_cycle.put(partial_speed); //run M1 motor at 'partial speed'
                    M2_duty_cycle.put(partial_speed); //run M2 motor at 'partial speed'
                    turn.put(true); //place into turn share that vehicle shall turn
                }
                else{   //If the car is running, bring it to a stop
                    //Serial << "Stop!!" << endl; //code used to simulate motor actions in Serial Monitor
                    state = 1; //sets state to show that vehicle is running
                    M1_duty_cycle.put(0); //run M1 motor at 'partial speed'
                    M2_duty_cycle.put(0); //run M1 motor at 'partial speed'
                    turn.put(false);    //place into turn share that vehicle shall not turn
                    delay(5000);    //pause 1s to allow motor/car to come to complete stop before person must turn it (due to turning not being functional)
                } 
            }
        }
    }
}



/** @brief   Task which scans two proximity sensors and places readings into shares
 *  @details This task runs an ultrasonic sensor and an infrared sensor to collect proximity readings.
 *           Code to use these sensors is borrowed from example code found online and cited within comments.
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void task_sensor_scan (void* p_params)
{
    (void)p_params;                             // Shuts up a compiler warning

    const TickType_t sim_period = 60;         // RTOS ticks (ms) between runs

    TickType_t xLastWakeTime = xTaskGetTickCount();

    float us_duration;  //store duration of ultrasonic echo pulse
    float us_distance;  //store distance reading from ultrasonic sensor
    int ir_reading;     //store voltage reading of infrared vout
    int ir_distance;    //store distance reading from infrared sensor

    for (;;)
    {

        //example code found at: https://cdn-learn.adafruit.com/downloads/pdf/distance-measurement-ultrasound-hcsr04.pdf
        pinMode (A1, INPUT);    //set pin connected to ultrasonic ECHO to input
        pinMode (A2, OUTPUT);   //set pin connected to ultrasonic TRIGGER to input
        
        //follow timing diagram specified at https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
        digitalWrite (A2, LOW);
        delayMicroseconds (2);
        digitalWrite (A2, HIGH);
        delayMicroseconds (10);
        us_duration = pulseIn(A1, HIGH, 30000); //read duration of echo pulse
        us_distance = us_duration / 58.2;   //divide by 58.2 as recomended in example code
        //Serial << "Ultrasonic: " << us_distance << endl;  //test ultrasonic reading in Serial monitor
        
        //example code found at: https://github.com/sparkfun/simple_sketches/blob/master/sharp/sharp.ino
        pinMode (A5, INPUT);    //set pin connected to infrared Vout to input
        ir_reading = analogRead(A5);    //read voltage from IR sensor
        ir_distance = (5269.8/(ir_reading-136.88)); //convert from voltage reading to distance (cm), refer to calibration plot in documentation
        //Serial << "Infrared: " << ir_distance << endl;    //test IR reading in Serial monitor

        US_distance.put(us_distance);   //put US distance reading in share
        IR_distance.put(ir_distance);   //put IR distance reading in share

        // This type of delay waits until it has been the given number of RTOS
        // ticks since the task previously began running. This prevents timing
        // inaccuracy due to not accounting for how long the task took to run
        vTaskDelayUntil (&xLastWakeTime, sim_period);
    }
}

/**@brief   Task which runs a motor
 *  @details This task uses the PWM duty cycle given to it and decides how to run the motor
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void task_RS_Motor1 (void* p_params)
{
    (void)p_params;            // Does nothing but shut up a compiler warning

    const TickType_t sim_period = 60;         // RTOS ticks (ms) between runs

    TickType_t xLastWakeTime = xTaskGetTickCount();

    
    int32_t duty_cycle_var; //stores the desired duty cycle of the PWM signal to motor
    float sim_A = 0.9;  //smoothing variable for motor speed
    float sim_B = 1 - sim_A;
    float motor_speed;    //stores the desired motor speed after smoothing

    MotorControl MotorA(D12, D7, D8, D10);  //sets up motor object with correct pins
    bool turn_stored;   //stores whether the vehicle shall turn


    for (;;)
    {
        // Gets duty_cycle_var from share
        M1_duty_cycle.get(duty_cycle_var);

        if(duty_cycle_var == 0){
            motor_speed = 0;  //if we want to stop the motor, just stop it
            turn_stored = true; //store that we want to turn the vehicle
        }
        else{
            motor_speed = motor_speed * sim_A + duty_cycle_var * sim_B;    //otherwise, smoothly change speed
            turn.get(turn_stored);  //gather from share whether the vehicle shall turn
        }
        
        //Serial << duty_cycle_var << endl;    //test duty_cycle_var in Serial monitor
        //Serial << motor_speed << endl;    //test motor_speed in Serial monitor
        
        MotorA.runMotor((uint32_t) motor_speed, turn_stored); //run motor at desired speed and direction
        //if the car is turning, running M1 clockwise, otherwise run counterclockwise
        
        vTaskDelayUntil (&xLastWakeTime, sim_period);
    }
}


/**@brief   Task which runs a motor
 *  @details This task uses the PWM duty cycle given to it and decides how to run the motor
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void task_RS_Motor2 (void* p_params)
{
    (void)p_params;            // Does nothing but shut up a compiler warning

    // Set up the variables of the simulation here
    const TickType_t sim_period = 60;         // RTOS ticks (ms) between runs

    // Initialise the xLastWakeTime variable with the current time.
    // It will be used to run the task at precise intervals
    TickType_t xLastWakeTime = xTaskGetTickCount();

    
    int32_t duty_cycle_var; //stores the desired duty cycle of the PWM signal to motor
    float sim_A = 0.9;  //smoothing variable for motor speed
    float sim_B = 1 - sim_A;
    float motor_speed;    //stores the desired motor speed after smoothing

    MotorControl MotorB(D6, D2, D4, D9);  //sets up motor object with correct pins


    for (;;)
    {
        // Gets duty_cycle_var from share
        M2_duty_cycle.get(duty_cycle_var);

        if(duty_cycle_var == 0){
            motor_speed = 0;  //if we want to stop the motor, just stop it
        }
        else{
            motor_speed = motor_speed * sim_A + duty_cycle_var * sim_B;    //otherwise, smoothly change speed
        }
        
        //Serial << duty_cycle_var << endl;    //test duty_cycle_var in Serial monitor
        //Serial << motor_speed << endl;    //test motor_speed in Serial monitor
        
        MotorB.runMotor((uint32_t) motor_speed, true); //run motor at desired speed and direction
        //always run this motor clockwise (even when turning)
        
        vTaskDelayUntil (&xLastWakeTime, sim_period);
    }
}


/** @brief   Arduino setup function which runs once at program startup.
 *  @details This function sets up a serial port for communication and creates
 *           the tasks which will be run.
 */
void setup () 
{
    // Start the serial port, wait a short time, then say hello. Use the
    // non-RTOS delay() function because the RTOS hasn't been started yet
    Serial.begin (115200);
    delay (2000);
    Serial << endl << endl << "ME507 UI Lab Starting Program" << endl;

    // Create a task which prints a more agreeable message
    xTaskCreate (task_sensor_scan,
                 "Scan",
                 1024,                            // Stack size
                 NULL,
                 4,                               // Priority
                 NULL);
    
    // Create a task which prints a more agreeable message
    xTaskCreate (task_process_sensor_data,
                 "Process",
                 1024,                            // Stack size
                 NULL,
                 2,                               // Priority
                 NULL);

    // Create a task which prints a more agreeable message
    xTaskCreate (task_RS_Motor1,
                 "RSMotor1",
                 1024,                            // Stack size
                 NULL,
                 3,                               // Priority
                 NULL);

    // Create a task which prints a more agreeable message
    xTaskCreate (task_RS_Motor2,
                 "RSMotor2",
                 1024,                            // Stack size
                 NULL,
                 3,                               // Priority
                 NULL);

    // If using an STM32, we need to call the scheduler startup function now;
    // if using an ESP32, it has already been called for us
    #if (defined STM32L4xx || defined STM32F4xx)
        vTaskStartScheduler ();
    #endif


}


/** @brief   Arduino's low-priority loop function, which we don't use.
 *  @details A non-RTOS Arduino program runs all of its continuously running
 *           code in this function after @c setup() has finished. When using
 *           FreeRTOS, @c loop() implements a low priority task on most
 *           microcontrollers, and crashes on some others, so we'll not use it.
 */
void loop () 
{
}

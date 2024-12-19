/**
 * @file Line_Follower_main.c
 * @brief Main source code for the Line_Follower program.
 *
 * This file contains the main entry point for the Line_Follower program.
 * The main controller demonstrates a Line Follower robot without using a specific algorithm.
 *
 * It interfaces the following peripherals using GPIO to demonstrate line following:
 *  - 8-Channel QTRX Sensor Array module
 *
 * Timers are used in this lab:
 *  - SysTick:  Used to generate periodic interrupts at a specified rate (1 kHz)
 *  - Timer A0: Used to generate PWM signals that will be used to drive the DC motors
 *  - Timer A1: Used to generate periodic interrupts at a specified rate (1 kHz)
 *
 * @note For more information regarding the 8-Channel QTRX Sensor Array module,
 * refer to the product page: https://www.pololu.com/product/3672
 *
 * @author Michael Granberry, Abdullah Hendy, Aaron Nanas
 *
 */

#include <stdint.h>
#include <math.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/GPIO.h"
#include "../inc/SysTick_Interrupt.h"
#include "../inc/EUSCI_A0_UART.h"
#include "../inc/Timer_A0_PWM.h"
#include "../inc/Timer_A1_Interrupt.h"
#include "../inc/Timer_A3_Capture.h"
#include "../inc/Motor.h"
#include "../inc/Tachometer.h"
#include "../inc/LPF.h"
#include "../inc/Analog_Distance_Sensor.h"
#include "../inc/Reflectance_Sensor.h"

#define CONTROLLER_1 1
//#define CONTROLLER_2 1

// Initialize constant PWM duty cycle values for the motors
#define PWM_NOMINAL         2500
#define PWM_SWING           1000
#define PWM_MIN             (PWM_NOMINAL - PWM_SWING)
#define PWM_MAX             (PWM_NOMINAL + PWM_SWING)
#define WHEEL_RADIUS    35000.0f   ///< wheel radius 0.00005cm //35000.0f / 1000  -> 35.0f
#define PI              3.14159f

//#define W 140000  ///< wheel base 0.0001 cm
//#define C 219910  ///< wheel circumference 0.0001cm
const uint16_t NO_BLACK = 335;
const uint16_t ALL_BLACK = 0;
float totalDistance = 0.0f;    // 총 이동 거리 (mm)
float currentSpeed = 0.0f;     // 현재 속도 (mm/s)
int funCallCnt = 0;
extern int isIntersectionIgnore;

uint8_t no_black_counter = 0;
uint8_t t_flag = 0;
uint8_t center_range = 100;
// Declare global variables used to update PWM duty cycle values for the motors
uint16_t Duty_Cycle_Left;
uint16_t Duty_Cycle_Right;

// Declare global variable used to store line sensor raw data
uint8_t Line_Sensor_Data;

// Declare global variable used to store line sensor position
int32_t Line_Sensor_Position;

// Global variable counter to keep track of the number of SysTick interrupts
// that have occurred. It increments in SysTick_Handler on each interrupt event.
uint32_t SysTick_counter = 0;

// Define the states for the Line Follower FSM
typedef enum
{
    CENTER  = 0,
    LEFT    = 1,
    RIGHT   = 2
} Line_Follower_State;

// Initialize the current state to CENTER
Line_Follower_State current_state = CENTER;
void moretask();

/**
 * @brief Implements the finite state machine (FSM) for a simple Line Follower robot.
 *
 * This function represents the FSM for the Line Follower robot. The robot's behavior is determined
 * by the current state, and it performs the following actions based on the state:
 * - CENTER: Moves forward and changes the RGB LED's color to green
 * - LEFT: Turns left and changes the RGB LED's color to blue
 * - RIGHT: Turns right and changes the RGB LED's color to yellow
 *
 * @return None
 */
void Line_Follower_FSM_1()
{
    switch(current_state)
    {
        case CENTER:
        {
            LED2_Output(RGB_LED_GREEN);
            Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
            break;
        }

        case LEFT:
        {
            LED2_Output(RGB_LED_BLUE);
            Motor_Left(PWM_NOMINAL, PWM_NOMINAL);
            break;
        }

        case RIGHT:
        {
            LED2_Output(RGB_LED_YELLOW);
            Motor_Right(PWM_NOMINAL, PWM_NOMINAL);
            break;
        }
    }
}

/**
 * @brief Implements the control logic for the Line Follower robot for a line-following robot based on sensor readings.
 *
 * This function is responsible for controlling the behavior of the Line Follower robot based on the values read from
 * from the 8-Channel QTRX Sensor Array module. It performs the following steps:
 *
 *  1. Increments the SysTick_counter by 1 every time the SysTick periodic interrupt occurs.
 *  2. Starts the process of reading the reflectance sensor array every 10 ms.
 *  3. Finishes reading the reflectance sensor array after 1 ms.
 *  4. Determines the position of the robot relative to the center of the line using Reflectance_Sensor_Position().
 *  5. Updates the current state of the robot based on the calculated position and predefined thresholds.
 *      - CENTER: The robot is at the center of the line (or very close to it).
 *      - RIGHT: The robot is on the left side of the line, and it needs to steer to the right to move back to the center.
 *      - LEFT: The robot is on the right side of the line, and it needs to steer to the left to move back to the center.
 *
 * @return None
 */
void Line_Follower_Controller_1()
{
    // Increment SysTick_counter by 1 every time the SysTick periodic interrupt occurs
    SysTick_counter = SysTick_counter + 1;

    // Start the process of reading the reflectance sensor array every 10 ms (i.e. 11, 21, 31, ...)
    if ((SysTick_counter % 10) == 1)
    {
        Reflectance_Sensor_Start();
    }

    // Finish reading the reflectance sensor sensor array after 1 ms (i.e. 12, 22, 32, ...)
    if ((SysTick_counter % 10) == 2)
    {
        Line_Sensor_Data = Reflectance_Sensor_End();
        Line_Sensor_Position = Reflectance_Sensor_Position(Line_Sensor_Data);

        // Check if the robot is at the center of the line (or very close to it)
        // Assign current_state to CENTER so that the robot will keep moving forward at the center
        if (Line_Sensor_Position > -47 && Line_Sensor_Position < 47)
        {
            current_state = CENTER;
        }

        // Check if the robot is on the left side of the line
        // Assign current_state to RIGHT to steer the robot to the right in order to move back to the center
        else if (Line_Sensor_Position >= 47 && Line_Sensor_Position < 332)
        {
            current_state = RIGHT;
            if (Line_Sensor_Position > -47 && Line_Sensor_Position < 47)
            {
                current_state = CENTER;
            }
        }

        // Check if the robot is on the right side of the line
        // Assign current_state to LEFT to steer the robot to the left in order to move back to the center
        else if (Line_Sensor_Position <= -47 && Line_Sensor_Position > -332)
        {
            current_state = LEFT;
            if (Line_Sensor_Position > -47 && Line_Sensor_Position < 47)
            {
                current_state = CENTER;
            }
        }

        // Otherwise, the robot will keep turning right at other positions (e.g. dead end)
        else
        {
            current_state = RIGHT;
        }
    }
}

/**
 * @brief Task 1: Implements the finite state machine (FSM) for a simple Line Follower robot and T intersection.
 * @return None
 */
void Line_Follower_FSM_2() {
    switch(current_state)
    {
        case CENTER:
        {
            LED2_Output(RGB_LED_GREEN);
            Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
            break;
        }

        case LEFT:
        {
            LED2_Output(RGB_LED_BLUE);
            Motor_Left(PWM_NOMINAL, PWM_NOMINAL);
            break;
        }

        case RIGHT:
        {
            LED2_Output(RGB_LED_YELLOW);
            Motor_Right(PWM_NOMINAL, PWM_NOMINAL);
            break;
        }
    }
}

/**
 * @brief Task 1: Implements the control logic for the Line Follower robot for a line-following and T intersection robot based on sensor readings. 
 * @return None
 */
void Line_Follower_Controller_2()
{
    // Increment SysTick_counter by 1 every time the SysTick periodic interrupt occurs
    SysTick_counter = SysTick_counter + 1;

    // Start the process of reading the reflectance sensor array every 10 ms (i.e. 11, 21, 31, ...)
    if ((SysTick_counter % 10) == 1)
    {
        Reflectance_Sensor_Start();
    }

    // Finish reading the reflectance sensor sensor array after 1 ms (i.e. 12, 22, 32, ...)
    if ((SysTick_counter % 10) == 2)
    {
        Line_Sensor_Data = Reflectance_Sensor_End();
        Line_Sensor_Position = Reflectance_Sensor_Position(Line_Sensor_Data);

        if (!t_flag) {
            // Check if the robot is at the center of the line (or very close to it)
            // Assign current_state to CENTER so that the robot will keep moving forward at the center

            if (Line_Sensor_Position > -center_range && Line_Sensor_Position < center_range)
            {
                current_state = CENTER;
            }
            else if (Line_Sensor_Position == ALL_BLACK) {
                t_flag = 1;
            }
            // Check if the robot is on the left side of the line
            // Assign current_state to RIGHT to steer the robot to the right in order to move back to the center
            else if (Line_Sensor_Position >= center_range && Line_Sensor_Position < 332)
            {
                current_state = RIGHT;
                if (Line_Sensor_Position > -center_range && Line_Sensor_Position < center_range)
                {
                    current_state = CENTER;
                }
            }

            // Check if the robot is on the right side of the line
            // Assign current_state to LEFT to steer the robot to the left in order to move back to the center
            else if (Line_Sensor_Position <= -center_range && Line_Sensor_Position > -332)
            {
                current_state = LEFT;
                if (Line_Sensor_Position > -center_range && Line_Sensor_Position < center_range)
                {
                    current_state = CENTER;
                }
            }

            // Otherwise, the robot will keep turning right at other positions (e.g. dead end)
            else
            {
                current_state = RIGHT;
            }
        } else {

            if (Line_Sensor_Position >= center_range && Line_Sensor_Position < 332)
            {
                current_state = RIGHT;
                if (Line_Sensor_Position > -center_range && Line_Sensor_Position < center_range)
                {
                    current_state = CENTER;
                }
            }
//            if (Line_Sensor_Position == NO_BLACK)
//            {
//                no_black_counter++;
//                if (no_black_counter == 2) {
//                    t_flag = 0;
//                    no_black_counter = 0;
//                }
//                current_state = RIGHT;
//                if (Line_Sensor_Position > -47 && Line_Sensor_Position < 47)
//                {
//                    current_state = CENTER;
//                }
//            }


        }
    }
}
uint16_t left_tach, right_tach;
enum Tachometer_Direction left_dir=STOPPED, right_dir=STOPPED;
int32_t left_steps, right_steps;


void enableIgnore() {
    static int funCallCnt=0;
    isIntersectionIgnore = 1;/*0이던 거를 1로 설정*/
    funCallCnt++;

}

void disableIgnore() {
    isIntersectionIgnore = 0;
}

void resetTotalDistance() {  totalDistance = 0.0f;  }

void doSome() {
    enableIgnore();
    resetTotalDistance();
}

void moretask() {

        static int32_t prev_left_steps = 0;
        static int32_t prev_right_steps = 0;

        // 타코미터 값 읽기
        Tachometer_Get(&left_tach, &left_dir, &left_steps,
                       &right_tach, &right_dir, &right_steps);

        // steps 변화량 계산 (방향 고려)
        int32_t delta_left = left_steps - prev_left_steps;
        int32_t delta_right = right_steps - prev_right_steps;

        // 평균 스텝 변화량 계산
        float avg_delta_steps = (delta_left + delta_right) / 2.0f;

        // 거리 계산 (mm) - steps를 회전수로 변환하여 계산
        float distance = (avg_delta_steps / 360.0f) * (2 * PI * (WHEEL_RADIUS / 1000.0f));
        totalDistance += distance;

        // 속도 계산 (mm/s) - 1ms 주기이므로 1000을 곱함
        // tach 값은 타이머 틱 수이므로, 실제 시간으로 변환 필요
        float avg_tach = (left_tach + right_tach) / 2.0f;
        if(avg_tach > 0) {  // 0으로 나누기 방지
            currentSpeed = distance * (48000000.0f / avg_tach);  // 48MHz 클럭 기준
        }

        // 현재 steps를 이전 값으로 저장
        prev_left_steps = left_steps;
        prev_right_steps = right_steps;

        // 주기적으로 거리와 속도 출력 (예: 1초마다)
        static uint32_t printCounter = 0;
        static uint32_t myCustomCounter = 0;


        if(++printCounter >= 1000) {  // 1초마다 출력
            printf("Distance: %.2f mm, Speed: %.2f mm/s\n",
                   totalDistance, currentSpeed);
            printf("Left Dir: %d, Right Dir: %d\n",
                   left_dir, right_dir);
            printCounter = 0;
            static int N = 10;
            static int FACTOR = 2;
            if(totalDistance > (10 * N * FACTOR)) {  //Threshold 도달시
                //Motor_Stop();
                //
                doSome(); // wrapper function
                //
                /*while(1) {
                    Motor_Stop();
                    LED1_Init();
                    LED2_Init();

                }*/
            }
        }
        if(++myCustomCounter > 8500 && isIntersectionIgnore==1) {
                    // 8.5s ?
                    disableIgnore();
                    myCustomCounter = 0;
                    if(printCounter+1 >= 1000) printCounter = 0;
                }









}


/**
 * @brief This function is the handler for the SysTick periodic interrupt with a rate of 1 kHz.
 *
 * The SysTick_Handler generates a periodic interrupt that calls a specific controller function based on the selected
 * active configuration. Only one of the options can be defined at a time: CONTROLLER_1 or CONTROLLER_2.
 *
 * @return None
 */
void SysTick_Handler(void)
{
#if defined CONTROLLER_1

    Line_Follower_Controller_1();

#elif defined CONTROLLER_2
    #if defined CONTROLLER_1
        #error "Only CONTROLLER_1 or CONTROLLER_2 can be active at the same time."
    #endif

    // Your function for Task 1 goes here (Line_Follower_Controller_2)
    Line_Follower_Controller_2();

#else
    #error "Define either one of the options: CONTROLLER_1 or CONTROLLER_2."
#endif
}

/**
 * @brief User-defined function executed by Timer A1 using a periodic interrupt at a rate of 1 kHz.
 *
 *
 * @return None
 */
void Timer_A1_Periodic_Task(void)
{
#if defined CONTROLLER_1

    Line_Follower_FSM_1();
    moretask();

#elif defined CONTROLLER_2
    #if defined CONTROLLER_1
        #error "Only CONTROLLER_1 or CONTROLLER_2 can be active at the same time."
    #endif

    // Your function for Task 1 goes here (Line_Follower_FSM_2)
    Line_Follower_FSM_2();
    moretask();

#else
    #error "Define either one of the options: CONTROLLER_1 or CONTROLLER_2."
#endif
}

int main(void)
{
    // Ensure that interrupts are disabled during initialization
    DisableInterrupts();

    // Initialize the 48 MHz Clock
    Clock_Init48MHz();

    // Initialize the built-in red LED
    LED1_Init();
    LED2_Init();

    // Initialize the buttons
    Buttons_Init();

    // Initialize EUSCI_A0_UART
    EUSCI_A0_UART_Init_Printf();

    // Initialize Timer A1 periodic interrupt with a rate of 1 kHz
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);

    // Initialize the tachometers
    Tachometer_Init();

    // Initialize the motors
    Motor_Init();

    // Initialize the 8-Channel QTRX Reflectance Sensor Array module
    Reflectance_Sensor_Init();

    // Initialize motor duty cycle values
    Duty_Cycle_Left  = PWM_NOMINAL;
    Duty_Cycle_Right = PWM_NOMINAL;

    // Initialize SysTick periodic interrupt with a rate of 1 kHz
    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);

    // Enable the interrupts used by Timer A1 and other modules
    EnableInterrupts();

    while(1)
    {
           printf("data: %d\n", Line_Sensor_Position);
           printf("flag: %d\n", t_flag);
           printf("counter %d\n", no_black_counter);
    }
}

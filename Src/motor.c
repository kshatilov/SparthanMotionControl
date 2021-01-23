
#include "gpio.h"
#include "cmsis_os.h"
#include "task.h"
#include "motor.h"
#include "sdadc.h"
#include <string.h>

#define ENCODER_MAX_COUNT			65535		//basically size of timer register
#define GEAR_REDUCTION		 		298			//n20 motor with 1:298 gear reduction
#define ROLLER_DIAMETER				0.006		//meters, aprox number
#define ENCODER_CNT_PER_REV		7 			 //on the motor spindle, one rev gives 7 encoder pulses

uint8_t uartTxProcessingBuffer[64];
#include "usart.h"
osThreadId motorControlThreadHandle;


float dt = 0.04f; 
extern float motorPosCmd[5];

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern TIM_HandleTypeDef htim19;

motor_t motors[5];
motorPid_t pids[5];

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static uint8_t abs_limit(float* a, float ABS_MAX)
{
  if (*a > ABS_MAX) {
	  *a = ABS_MAX;
	  return 1;
  }

  if (*a < -ABS_MAX){
	  *a = -ABS_MAX;
	  return 1;
  }
  return 0;
}

void motorInit(motor_t* motor) {
	HAL_TIM_Encoder_Start(motor->encTim, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(motor->pwmTim, motor->pwmCh);

}

void motorSetPwm(motor_t* motor, float pwmSet) {
  abs_limit(&pwmSet, ENCODER_MAX_COUNT);
  if (pwmSet > 0) {
    HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, 0);
    __HAL_TIM_SET_COMPARE(motor->pwmTim, motor->pwmCh, (uint16_t)pwmSet);} 
  else {
    HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, 1);
    __HAL_TIM_SET_COMPARE(motor->pwmTim, motor->pwmCh, (uint16_t)-pwmSet);
  }
}

void motorControlThreadFunction(void const* argument) {

	motors[0].dirPort = DIR5_GPIO_Port;		//mapping the motors from mechanical order to electrical connections
	motors[0].dirPin = DIR5_Pin;
	motors[0].encTim = &htim19;
	motors[0].pwmTim = &htim12;
	motors[0].pwmCh = TIM_CHANNEL_2;

	motors[1].dirPort = DIR3_GPIO_Port;
	motors[1].dirPin = DIR3_Pin;
	motors[1].encTim = &htim4;
	motors[1].pwmTim = &htim15;
	motors[1].pwmCh = TIM_CHANNEL_2;

	motors[2].dirPort = DIR1_GPIO_Port;
	motors[2].dirPin = DIR1_Pin;
	motors[2].encTim = &htim2;
	motors[2].pwmTim = &htim16;
	motors[2].pwmCh = TIM_CHANNEL_1;

	motors[3].dirPort = DIR2_GPIO_Port;
	motors[3].dirPin = DIR2_Pin;
	motors[3].encTim = &htim3;
	motors[3].pwmTim = &htim17;
	motors[3].pwmCh = TIM_CHANNEL_1;

	motors[4].dirPort = DIR4_GPIO_Port;
	motors[4].dirPin = DIR4_Pin;
	motors[4].encTim = &htim5;
	motors[4].pwmTim = &htim12;
	motors[4].pwmCh = TIM_CHANNEL_1;

	for(uint8_t i = 0; i < 5; i++) {
    motorInit(&motors[i]);
    pids[i].kP = 150; // 50 alone is not enough to bend fully
    pids[i].kI = 40; // 15 high values give persicion on small pulses (0.02), but compromise longer pulses (0.2+)
    pids[i].kD = 15; //15  large values allow sharper I falloffs - good for long pulses? large values give instabilities at short pulses
    pids[i].maxI = 60000; // 60000 with dt = 0.04, delay 20(ms?) at 9V - gives errors of <30 for pulses <0.1
    pids[i].maxOut = 65535; //65535;
    pids[i].lastSetPoint = 0;
		motors[i].encMeterToCount = ENCODER_CNT_PER_REV * GEAR_REDUCTION / (3.14159 * ROLLER_DIAMETER);	
    motors[i].encStall = 0;	    
    motors[i].encCountRaw = 10;   // init with smth to avoid stalling at initialization
  }

  int minEncDelta = 7; // if we don't see at least this number of encoder tics per dt - maybe it's stalled.
  uint8_t maxStall = 100; // max dt cycles to see the stall


  // init send
  float reply[5] = {0.f,0.f,0.f,0.f,0.f};
  uartSendBuffer(reply, sizeof(float)*5);

  while(1) {
		for(uint8_t i = 0; i < 5; i++) {
	
			//update encoder data
			motors[i].encCountRaw = __HAL_TIM_GET_COUNTER(motors[i].encTim);
			if 		 (motors[i].encCountRaw - motors[i].encCountPrev >  ENCODER_MAX_COUNT / 2) motors[i].encCountRound--;
			else if (motors[i].encCountRaw - motors[i].encCountPrev < -ENCODER_MAX_COUNT / 2) motors[i].encCountRound++;
			motors[i].encCountTotal = motors[i].encCountRaw + motors[i].encCountRound * ENCODER_MAX_COUNT;

			pids[i].setPoint = motorPosCmd[i] * motors[i].encMeterToCount;

      // handle cases when encoders fail to report or the motor can't rotate      
      if (pids[i].lastSetPoint == pids[i].setPoint) { // change only when command is same as prev
        if (motors[i].encCountRaw - motors[i].encCountPrev < minEncDelta && 
             motors[i].encCountRaw - motors[i].encCountPrev > -minEncDelta){
          motors[i].encStall += 1;
        }
        else if (motors[i].encStall > 0) { motors[i].encStall -= 1; } // if it's moving - give it a chance
      }
      else {
        motors[i].encStall = 0;
      }
 
      motors[i].encCountPrev = motors[i].encCountRaw;

      pids[i].error = pids[i].setPoint - motors[i].encCountTotal;

      uint8_t maxed = 0;    

      //calc PID    
      pids[i].P = pids[i].kP * pids[i].error;
      pids[i].D = pids[i].kD * (pids[i].error - pids[i].lastError) / dt;
      abs_limit(&pids[i].D, pids[i].maxOut/4);// huge spikes don't help
      pids[i].out = pids[i].P + pids[i].D + pids[i].I * pids[i].kI;

      // if target is too far, we don't increase the integral controller
      // aka anti-reset windup
      maxed = abs_limit(&pids[i].out, pids[i].maxOut);
      pids[i].I = (maxed) ? pids[i].error * dt: pids[i].error * dt + pids[i].I;
      abs_limit(&pids[i].I, pids[i].maxI);

      pids[i].lastError = pids[i].error;

      pids[i].lastSetPoint = pids[i].setPoint; // for controlling motor stall
      if (motors[i].encStall > maxStall) {
        motors[i].encStall -= 1; // stop growing so it doesn't overflow
        pids[i].out = 0;  
      //  pids[i].I = 0;
      }

      motorSetPwm(&motors[i], pids[i].out); // power up the motors!
      reply[i] = (pids[i].error) / ENCODER_CNT_PER_REV / GEAR_REDUCTION * 100;
    }
      
    uartSendBuffer(&reply, sizeof(float)*5);
	  osDelay(10);
  }

}

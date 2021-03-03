/********************************************************************
*           Obstacle Avoiding Robot Car with MPU6050 IMU
*
*            Copyright (C) 2015 by Ulrik Hørlyk Hjort 
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; If not, see <http://www.gnu.org/licenses/>.
**********************************************************************/
#include<Wire.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <limits.h>

#define PWM_MOTOR_1 6
#define IN1 9
#define IN2 8
#define IN3 7
#define IN4 10
#define PWM_MOTOR_2 5
#define MPU 0x68 

void imuTask(void *pvParameters);
void driveTask(void *pvParameters);

static TaskHandle_t driveTaskHandle = NULL;
static TaskHandle_t imuTaskHandle = NULL;

const float accThreshold = 3500.0; 
const float gyroThreshold = 4000.0;

 struct Mpu6050 {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t temp;
  int16_t gx;
  int16_t gy;
  int16_t gz;  
 };

void setup() {
  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // PWR_MGMT_1 
  Wire.write(0);    // Set PWR_MGMT_1 to 0 to wake up the MPU
  Wire.endTransmission(true);
    
  xTaskCreate(
    imuTask
    ,  "Dist"   
    ,  64 
    ,  NULL
    ,  3 
    ,  &imuTaskHandle);
  
  xTaskCreate(
    driveTask
    ,  "Drive" 
    ,  64  
    ,  NULL
    ,  3  
    ,  &driveTaskHandle);    
}

void readMPU(struct Mpu6050 *data){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start register ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true); // Read 14 registers
  data->ax=Wire.read()<<8|Wire.read();
  data->ay=Wire.read()<<8|Wire.read();
  data->az=Wire.read()<<8|Wire.read();
  data->temp=Wire.read()<<8|Wire.read();
  data->gx=Wire.read()<<8|Wire.read();
  data->gy=Wire.read()<<8|Wire.read();
  data->gz=Wire.read()<<8|Wire.read();
}

/*
 * imuTask: Check for obstacles with the MPU6050 IMU - that is acceleration and angular velocity 
 *  exceeds threshold limits from the impact with the obstacle - and notify driveTask if obstacle is deteced.
 */
void imuTask(void *pvParameters __attribute__((unused))) {
  
  Mpu6050 mpu;

  for(;;) {
        readMPU(&mpu);        
        float accSum = fabs(mpu.ax) + fabs(mpu.ay);
        float gyroSum = fabs(mpu.gx) + fabs(mpu.gy) + fabs(mpu.gz);
                
        if ((accSum > accThreshold) && (gyroSum > gyroThreshold)) {
          xTaskNotifyGive(driveTaskHandle);
          // Wait for xTaskNotifyGive from driveTask
          ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
        }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/*
 * driveTask runs the state machine:
 * Go FORWARD until an obstacle is detected (blocks in FORWARD state until notified
 * from deistanceTask that obstacle is ahead) then change to REVERSE state and reverse
 * for about 1 second and enter TURN state. Do a short turn in TURN state and return to FORWARD state.
 */
void driveTask(void *pvParameters __attribute__((unused))) {
  uint32_t ulNotifiedValue;
  enum States{FORWARD,REVERSE,TURN};
  States state=FORWARD;
  
  for(;;) {  
    switch(state) {
      case FORWARD:        
        forward();
        setSpeed(110);
        // Wait for xTaskNotifyGive from imuTask
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        state=REVERSE;
      break;
      
      case REVERSE:
        reverse();
        setSpeed(255);
        vTaskDelay(pdMS_TO_TICKS((500+random(-100, 50))));
        state=TURN;      
      break;

      case TURN:     
       turnRight();
       setSpeed(110);
       vTaskDelay(pdMS_TO_TICKS((1000+random(-500, 50))));
       state=FORWARD;
       xTaskNotifyGive(imuTaskHandle);      
      break; 
    }
  }
}

void _start() {
  digitalWrite(PWM_MOTOR_1,HIGH);
  digitalWrite(PWM_MOTOR_2,HIGH);
}

void _stop() {
  digitalWrite(PWM_MOTOR_1,LOW);
  digitalWrite(PWM_MOTOR_2,LOW);  
}

void forward() {
  _start();
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);    
  digitalWrite(IN4,LOW);  
}

void reverse() {
  _start();
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);  
  digitalWrite(IN3,LOW);  
  digitalWrite(IN4,HIGH);  
}

void turnRight() {
  _start();
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);  
  digitalWrite(IN3,LOW);  
  digitalWrite(IN4,HIGH);    
}

void setSpeed(int s) {
  analogWrite(PWM_MOTOR_1,s);
  analogWrite(PWM_MOTOR_2,s);
}

void loop() {}

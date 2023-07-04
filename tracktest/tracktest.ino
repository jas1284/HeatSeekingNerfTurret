/*This example will work with a Teensy 3.1 and above. The MLX90640 requires some
  hefty calculations and larger arrays. You will need a microcontroller with 20,000
  bytes or more of RAM.

  This relies on the driver written by Melexis and can be found at:
  https://github.com/melexis/mlx90640-library

  Hardware Connections:
  Connect the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  to the Qwiic board
  Connect the male pins to the Teensy. The pinouts can be found here: https://www.pjrc.com/teensy/pinout.html
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;
#include <ElementStorage.h>

#include <Adafruit_MotorShield.h>

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"



const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640

#define TA_SHIFT 8 //Default shift for MLX90640 in open air

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #1 (M1 and M2)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 1);

float mlx90640To[768];
paramsMLX90640 mlx90640;

float matrix[24][32];
float threshold = 30.5;
int colAboveCount[32];
float avgcol = (31.0/2);
float vector = 0;
float directionModifier = 2.85;
int steps = 0;
unsigned long lastFire = 0;
int idleCounter = 0;
int trackedConsecutively = 0;
bool idleDirection = 0;
int stepHistory[5];
int delayHistory[5];
unsigned long trackStartTime = 0;

void setup()
{
  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  Serial.begin(115200); //Fast serial as possible
  
  while (!Serial); //Wait for user to open terminal
  //Serial.println("MLX90640 IR Array Example");
  if (isConnected() == false)
  {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    while (1);
  }

  //Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)
    Serial.println("Failed to load system parameters");
  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
    Serial.println("Parameter extraction failed");

  //Once params are extracted, we can release eeMLX90640 array

  //Set refresh rate
  //A rate of 0.5Hz takes 4Sec per reading because we have to read two frames to get complete picture
  //MLX90640_SetRefreshRate(MLX90640_address, 0x00); //Set rate to 0.25Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x01); //Set rate to 0.5Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x02); //Set rate to 1Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x03); //Set rate to 2Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x04); //Set rate to 4Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x05); //Set rate to 8Hz effective - Works at 800kHz
  MLX90640_SetRefreshRate(MLX90640_address, 0x06); //Set rate to 16Hz effective - Works at 800kHz
  //MLX90640_SetRefreshRate(MLX90640_address, 0x07); //Set rate to 32Hz effective - fails
  
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  Wire.setClock(1000000); //Increase I2C clock speed to 1000kHz
  }
  Serial.println("Motor Shield found.");
  
  myMotor->setSpeed(10);  // 10 rpm
  
  pinMode(9, OUTPUT);
}

void loop()
{  
  long startTime = millis();
  digitalWrite(9,LOW);
  for (byte x = 0 ; x < 1 ; x++) // read twice to get all the data, but what if just read once?
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;
    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
  }
  long stopRead = millis();
  Serial.print("Time required to read from MLX90640: ");
  Serial.println(startTime - stopRead);

  // storing it as a matrix!
  //row, col notation - in y 0-> 24 and x 0 ->32,
  //should have right dimensions except left/right mirrored. 
  int i = 0;
  for(int y = 0; y < 24; y++){
    for (int x = 31; x >= 0; x--){
      matrix[y][x] = mlx90640To[i];
      i++;
    }
  }
  int totalAbove = 0;
  int weightedTally = 0;
  // calculating for each column how many values exceed threshold
  // screw that, directly calculating the weighted average
  for(int x = 0; x < 32; x++){
    //int count = 0;
    for(int y = 0; y < 24; y++){
      if(matrix[y][x] >= threshold){
        //count ++;
        totalAbove ++; 
        weightedTally += x;
      }
    }
    //colAboveCount[x] = count;
  }
  //long stopCalc = millis();
  // calculating average from weighted sum and total addends
  if(totalAbove > 0){ //target found!
    if(trackedConsecutively == 0){
      trackStartTime = millis();
    }
    avgcol = weightedTally / totalAbove;
    //Serial.print("avgcol is");
    //Serial.println(avgcol);
    vector = avgcol - (31.0/2);
    steps = round(vector * directionModifier);
    myMotor->release();
    if(steps > 0){
      //Serial.println("stepping Forward");
      myMotor->step(steps, FORWARD, DOUBLE);//track forward
     }
    else{
      //Serial.println("stepping Backward");
      myMotor->step(abs(steps), BACKWARD, DOUBLE);//track backwards
    }
    myMotor->release();
    unsigned long stepDone = millis();
    //Serial.println("step time: ");
    //Serial.println(stepDone - stopCalc);
    int thisTime = stepDone - trackStartTime;
    for (int i = 0; i < 4; i++){
      stepHistory[i] = stepHistory[i+1];
      delayHistory[i] = delayHistory[i+1];
    }
    delayHistory[4] = thisTime; //updating our delayhistory and stephistory
    stepHistory[4] = steps;

    trackedConsecutively++;
    if((trackedConsecutively >= 5) && ((millis() - lastFire) > 30000)){ //if tracked for 5+ cycles, let's do some math and fire!
      //we'll do 2 least squares regressions, one for delay and one for step
      // A^T * A x = A^T * b
      // regressions are of form c1 + xc2 + x^2 c3 = y
      BLA::Matrix<5,3> Adelay;
      BLA::Matrix<5,1>bdelay;
      Adelay.Fill(1);
      for(int i = 0; i < 5; i++){
        Adelay(i,1) = i+1;
        Adelay(i,2) = (i+1)*(i+1);
        bdelay(i) = delayHistory[i];
      }
      BLA::Matrix<3,5> Adelay_T = ~Adelay;
      BLA::Matrix<3,3> A_decomp_delay = Adelay_T * Adelay;  // LUDecompose will destroy A here so we'll pass in a copy so we can refer back to A later
      auto decomp_delay = LUDecompose(A_decomp_delay);
      BLA::Matrix<3> b_decomp_delay = Adelay_T * bdelay;
      BLA::Matrix<3> x_delay = LUSolve(decomp_delay, b_decomp_delay);

      BLA::Matrix<5,3> Astep;
      BLA::Matrix<5,1>bstep;
      Astep.Fill(1);
      for(int i = 0; i < 5; i++){
        Astep(i,1) = delayHistory[i];
        Astep(i,2) = delayHistory[i]*delayHistory[i];
        bstep(i) = stepHistory[i];
      }
      BLA::Matrix<3,5> Astep_T = ~Astep;
      BLA::Matrix<3,3> A_decomp_step = Astep_T * Astep;  // LUDecompose will destroy A here so we'll pass in a copy so we can refer back to A later
      auto decomp_step = LUDecompose(A_decomp_step);
      BLA::Matrix<3> b_decomp_step = Astep_T * bstep;
      BLA::Matrix<3> x_step = LUSolve(decomp_step, b_decomp_step);

      long predictTime = x_delay(0) + (6*x_delay(1)) + (36*x_delay(2));
      long stopCalc = millis();
      int calcTime = (stopCalc - stopRead);
      long predictSlewTime = max(predictTime - 240 - calcTime, 0);

      int predictSteps = round(x_step(0) + (predictSlewTime*x_step(1)) + (predictSlewTime * predictSlewTime * x_step(2)));

      if(predictSteps > 0){
        //Serial.println("stepping Forward");
        myMotor->step(predictSteps, FORWARD, DOUBLE);//track forward
      }
      else{
        //Serial.println("stepping Backward");
        myMotor->step(abs(predictSteps), BACKWARD, DOUBLE);//track backwards
      }
      //FIRE!!!
      digitalWrite(9, HIGH);
      digitalWrite(13, HIGH);
      delay(150);
      digitalWrite(9,LOW);
      digitalWrite(13, LOW);
      lastFire = millis();
    }
  }
  else{
    avgcol = 31.0/2; //no target found! time to search idly. 
    //Serial.println("None above threshold.");
    vector = 0.0;
    trackedConsecutively = 0;
    memset(stepHistory, 0, 5); // clear step and delay histories
    memset(delayHistory, 0, 5);
    trackStartTime = 0;
    myMotor->release();
    if (idleCounter < 5){ //go in each direction up to 5 times
      if(idleDirection){
        myMotor->step(35, FORWARD, DOUBLE);
      }
      else{
        myMotor->step(35, BACKWARD, DOUBLE);
      }
      myMotor->release();
      idleCounter++;
    }
    else{ // if we've gone in that direction too many times, turn around!
      idleDirection = !idleDirection;
      idleCounter = 0;
    }
  }  
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}

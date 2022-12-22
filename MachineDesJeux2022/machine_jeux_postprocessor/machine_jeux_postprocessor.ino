
/**
  * @autor: Gabriel Tremblay
  * 
  * @brief: Machine des jeux 2021: post processor for serial commands to angles for stepper/pwms
  **/
#include <Servo.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include <Derivs_Limiter.h>

/*==================================================================================================
*                                        DEFINES AND MACRO
==================================================================================================*/
#define RX_BUFFER_SIZE    (64U)
#define SERIAL_TERMINATOR ('\n')
#define SERVO_MOVE_END    (5U)  // Threshold value (us) until a servo target is reached
#define STEPS_MOVE_END    (66U) // Threshold value (steps) until movement target of stepper axes is reached.

#define MIN_SERVO_PULSE       (500U)  //us
#define MID_SERVO_PULSE       (1500U) //us
#define MAX_SERVO_PULSE       (2500U) //us
#define SERVO_PULSE_RANGE     (2000U) //
#define SERVO_4_5_ANGLE_RANGE (270U)//deg
#define SERVO_6_ANGLE_RANGE   (300U)//deg

#define AXIS_1_DIR_PORT   (2U)
#define AXIS_1_PUL_PORT   (3U)
#define AXIS_2_DIR_PORT   (4U)
#define AXIS_2_PUL_PORT   (5U)
#define AXIS_3_DIR_PORT   (6U)
#define AXIS_3_PUL_PORT   (7U)
#define AXIS_4_PORT       (9U)
#define AXIS_5_PORT       (10U)
#define AXIS_6_PORT       (11U)

/**
 * Formula to transfer axis max speed from steps/seconds to angle/sec is:
 *    Deg/sec = StepSpeed * 360 / (TransmissionRatio * StepPerRevolution )
 */
#define STEP_AXIS_CONFIG_EEPROM_ADDR    (0U)
#define MOVE_THRESHOLD_EEPROM_ADDR      (sizeof(StepperConfigType))
#define SERVO_AXIS_CONFIG_EEPROM_ADDR   (MOVE_THRESHOLD_EEPROM_ADDR + sizeof(MoveThresholdConfigType))
#define STEPPER_DRIVER_STEPS_PER_REV    (400U)  // This setup is called "Default" on the driver red switches
#define AXIS_1_DEFAULT_MAX_SPEED        (60U)   // deg/seconds
#define AXIS_2_DEFAULT_MAX_SPEED        (80U)   // deg/seconds
#define AXIS_3_DEFAULT_MAX_SPEED        (80U)   // deg/seconds

#define AXIS_1_DEFAULT_ACCEL      (50U) // deg/seconds^2
#define AXIS_2_DEFAULT_ACCEL      (60U) // deg/seconds^2
#define AXIS_3_DEFAULT_ACCEL      (60U) // deg/seconds^2

#define AXIS_1_TRANS_RATIO        (10U) // 1/ratio
#define AXIS_2_TRANS_RATIO        (40U) // 1/ratio
#define AXIS_3_TRANS_RATIO        (50U) // 1/ratio

#define AXIS_4_DEFAULT_MAX_SPEED  (60U) // angle/seconds
#define AXIS_5_DEFAULT_MAX_SPEED  (60U) // angle/seconds
#define AXIS_6_DEFAULT_MAX_SPEED  (60U) // angle/seconds

#define AXIS_4_DEFAULT_ACCEL      (40U)  // angle/seconds^2
#define AXIS_5_DEFAULT_ACCEL      (40U) // angle/seconds^2
#define AXIS_6_DEFAULT_ACCEL      (40U) // angle/seconds^2

#define TARGET_CMD                (1U)
#define MOVE_CMD                  (2U)
#define STOP_CMD                  (3U)
#define WRITE_STEP_AXIS_CFG_CMD   (4U)
#define READ_STEP_AXIS_CFG_CMD    (5U)
#define WRITE_MOVE_END_CFG_CMD    (6U)
#define READ_MOVE_END_CFG_CMD     (7U)
#define WRITE_SERVO_AXIS_CFG_CMD  (8U)
#define READ_SERVO_AXIS_CFG_CMD   (9u)


#define END_OF_MOVEMENT (69U) // nice

/*==================================================================================================
*                                        Type definitions
==================================================================================================*/
typedef struct
{
  uint16_t axis1MaxSpeed; // deg/sec
  uint16_t axis1Accel;    // deg/sec^2
  uint16_t axis2MaxSpeed; // deg/sec
  uint16_t axis2Accel;    // deg/sec^2
  uint16_t axis3MaxSpeed; // deg/sec
  uint16_t axis3Accel;    // deg/sec^2
}StepperConfigType;

typedef struct
{
  uint16_t axis4MaxSpeed; // deg/sec
  uint16_t axis4Accel;    // deg/sec^2
  uint16_t axis5MaxSpeed; // deg/sec
  uint16_t axis5Accel;    // deg/sec^2
  uint16_t axis6MaxSpeed; // deg/sec
  uint16_t axis6Accel;    // deg/sec^2
}ServoConfigType;

typedef struct
{
  uint8_t stepperThreshold; // deg
  uint8_t servoThreshold;   // deg
}MoveThresholdConfigType;
/*==================================================================================================
*                                        Global Variables
==================================================================================================*/
StepperConfigType StepperConfig = 
{
  .axis1MaxSpeed =  AXIS_1_DEFAULT_MAX_SPEED,
  .axis1Accel =     AXIS_1_DEFAULT_ACCEL,
  .axis2MaxSpeed =  AXIS_2_DEFAULT_MAX_SPEED,
  .axis2Accel =     AXIS_2_DEFAULT_ACCEL,
  .axis3MaxSpeed =  AXIS_3_DEFAULT_MAX_SPEED,
  .axis3Accel =     AXIS_3_DEFAULT_ACCEL,
};

ServoConfigType ServoConfig = 
{
  .axis4MaxSpeed =  AXIS_1_DEFAULT_MAX_SPEED,
  .axis4Accel =     AXIS_1_DEFAULT_ACCEL,
  .axis5MaxSpeed =  AXIS_2_DEFAULT_MAX_SPEED,
  .axis5Accel =     AXIS_2_DEFAULT_ACCEL,
  .axis6MaxSpeed =  AXIS_3_DEFAULT_MAX_SPEED,
  .axis6Accel =     AXIS_3_DEFAULT_ACCEL,
};

MoveThresholdConfigType ThresholdConfig = 
{
  .stepperThreshold =   STEPS_MOVE_END,
  .servoThreshold =     SERVO_MOVE_END,
};

// Stepper objects with defined ports
AccelStepper axis1(AccelStepper::DRIVER,AXIS_1_DIR_PORT,AXIS_1_PUL_PORT);
AccelStepper axis2(AccelStepper::DRIVER,AXIS_2_DIR_PORT,AXIS_2_PUL_PORT);
AccelStepper axis3(AccelStepper::DRIVER,AXIS_3_DIR_PORT,AXIS_3_PUL_PORT);

// Servo objects with limiters to allow smooth angle control
Servo axis4;
Servo axis5;
Servo axis6;

Derivs_Limiter axis4Lim = Derivs_Limiter(ServoConfig.axis4MaxSpeed, ServoConfig.axis4Accel);
Derivs_Limiter axis5Lim = Derivs_Limiter(ServoConfig.axis5MaxSpeed, ServoConfig.axis5Accel);
Derivs_Limiter axis6Lim = Derivs_Limiter(ServoConfig.axis6MaxSpeed, ServoConfig.axis6Accel);

// Various state variables for main loop events
boolean new_command = false;

/*==================================================================================================
*                                        Local Functions
==================================================================================================*/

/** @Brief: Servo motors initialization
  **/
void Servo_AxisInit() 
{
  // Attach servo objects to configured ports and set pulse limits
  axis4.attach(AXIS_4_PORT, MIN_SERVO_PULSE, MAX_SERVO_PULSE);
  axis5.attach(AXIS_5_PORT, MIN_SERVO_PULSE, MAX_SERVO_PULSE);
  axis6.attach(AXIS_6_PORT, MIN_SERVO_PULSE, MAX_SERVO_PULSE);
  
  // Set limiter object's position limit and initial position
  axis4Lim.setPosLimits(MIN_SERVO_PULSE, MAX_SERVO_PULSE);
  axis4Lim.setPosition(MID_SERVO_PULSE);
  axis4Lim.setTarget(MID_SERVO_PULSE);
  axis4Lim.setPreventGoingWrongWay(false); // Allows to smooth if new command or stop command is sent
  
  axis5Lim.setPosLimits(MIN_SERVO_PULSE, MAX_SERVO_PULSE);
  axis5Lim.setPosition(MID_SERVO_PULSE);
  axis5Lim.setTarget(MID_SERVO_PULSE);
  axis5Lim.setPreventGoingWrongWay(false);
  
  axis6Lim.setPosLimits(MIN_SERVO_PULSE, MAX_SERVO_PULSE);
  axis6Lim.setPosition(MID_SERVO_PULSE);
  axis6Lim.setTarget(MID_SERVO_PULSE);
  axis6Lim.setPreventGoingWrongWay(false);
}

/*==================================================================================================
*                                        Local Functions
==================================================================================================*/

/** @Brief: Change the max speed and acceleration of servo axis with data of ServiConfig struct.
 *  This function should be called after the ServoConfig struct was updated by another function.
 *  The config values from the ServoConfig struct are in deg/sec and deg/(sec*sec), thus they 
 *  are translated to usec values
  **/
void Servo_AxisConfig() 
{
  // Temp values to compute without floats
  uint32_t tempSpeed = 0;
  uint32_t tempAccel = 0; 

  tempSpeed = (uint32_t)ServoConfig.axis4MaxSpeed * SERVO_PULSE_RANGE / SERVO_4_5_ANGLE_RANGE;
  tempAccel = (uint32_t)ServoConfig.axis4Accel * SERVO_PULSE_RANGE / SERVO_4_5_ANGLE_RANGE;
  axis4Lim.setVelAccelLimits((uint16_t)tempSpeed, (uint16_t)tempAccel);

  tempSpeed = (uint32_t)ServoConfig.axis5MaxSpeed * SERVO_PULSE_RANGE / SERVO_4_5_ANGLE_RANGE;
  tempAccel = (uint32_t)ServoConfig.axis5Accel * SERVO_PULSE_RANGE / SERVO_4_5_ANGLE_RANGE;
  axis5Lim.setVelAccelLimits((uint16_t)tempSpeed, (uint16_t)tempAccel);
  
  tempSpeed = (uint32_t)ServoConfig.axis6MaxSpeed * SERVO_PULSE_RANGE / SERVO_6_ANGLE_RANGE;
  tempAccel = (uint32_t)ServoConfig.axis6Accel * SERVO_PULSE_RANGE / SERVO_6_ANGLE_RANGE;
  axis6Lim.setVelAccelLimits((uint16_t)tempSpeed, (uint16_t)tempAccel);
}

/** @Brief: Configure the max speed and acceleration of stepper motors.
  **/
void Stepper_AxisConfig() 
{
  long temp = 0; // variable to store 32bit operations
  
  // Axis 1 max speed/Acceleration
  temp = ((long)StepperConfig.axis1MaxSpeed * STEPPER_DRIVER_STEPS_PER_REV * AXIS_1_TRANS_RATIO)/360;
  axis1.setMaxSpeed((int)temp);
  temp = ((long)StepperConfig.axis1Accel * STEPPER_DRIVER_STEPS_PER_REV * AXIS_1_TRANS_RATIO)/360;
  axis1.setAcceleration((int)temp);

  // Axis 2 max speed/Acceleration
  temp = ((long)StepperConfig.axis2MaxSpeed * STEPPER_DRIVER_STEPS_PER_REV * AXIS_2_TRANS_RATIO)/360;
  axis2.setMaxSpeed((int)temp);
  temp = ((long)StepperConfig.axis2Accel * STEPPER_DRIVER_STEPS_PER_REV * AXIS_2_TRANS_RATIO)/360;
  axis2.setAcceleration((int)temp);

  // Axis 3 max speed/Acceleration
  temp = ((long)StepperConfig.axis3MaxSpeed * STEPPER_DRIVER_STEPS_PER_REV * AXIS_3_TRANS_RATIO)/360;
  axis3.setMaxSpeed((int)temp);
  temp = ((long)StepperConfig.axis3Accel * STEPPER_DRIVER_STEPS_PER_REV * AXIS_3_TRANS_RATIO)/360;
  axis3.setAcceleration((int)temp);
}

/**
 * @Brief Function called when a the serial RX buffer is not empty. This does not seem to be
 * called in an interrupt context, but rather somewhere after the main loop(), thus it is at
 * worst a periodically called function which is not called when a blocking call is
 * executing in the main loop
 */
void serialEvent()
{
  char rxBuffer[RX_BUFFER_SIZE];
  // Copy data from serial buffer to global buffer
  Serial.readBytesUntil(SERIAL_TERMINATOR, rxBuffer, RX_BUFFER_SIZE);
  
  // Extract first char received as the command ID
  char* command = strtok(rxBuffer, ",");
  int temp = atoi(command);
  
  // Handle the received command
  if(temp == TARGET_CMD)
  {
      /* Target absolute angle received from handler. These are transformed from 6 angles to steps and 
       *  pulse width, then passed to the corresponding servo/stepper object for processing in
       *  the main loop
      */
      // Extract 6 angle values with C like strings for performance. Values are separated by ','
      command = strtok(0, ",");
      axis1.moveTo((long)atoi(command) * STEPPER_DRIVER_STEPS_PER_REV * AXIS_1_TRANS_RATIO / 360);
      command = strtok(0, ",");
      axis2.moveTo((long)atoi(command) * STEPPER_DRIVER_STEPS_PER_REV * AXIS_2_TRANS_RATIO / 360);
      command = strtok(0, ",");
      axis3.moveTo((long)atoi(command) * STEPPER_DRIVER_STEPS_PER_REV * AXIS_3_TRANS_RATIO / 360);
      command = strtok(0, ",");
      axis4Lim.setTarget(constrain(((long)atoi(command) * SERVO_PULSE_RANGE / SERVO_4_5_ANGLE_RANGE) + MID_SERVO_PULSE, MIN_SERVO_PULSE, MAX_SERVO_PULSE)) ;
      command = strtok(0, ",");
      axis5Lim.setTarget(constrain(((long)atoi(command) * SERVO_PULSE_RANGE / SERVO_4_5_ANGLE_RANGE) + MID_SERVO_PULSE, MIN_SERVO_PULSE, MAX_SERVO_PULSE)) ;
      command = strtok(0, ",");
      axis6Lim.setTarget(constrain(((long)atoi(command) * SERVO_PULSE_RANGE / SERVO_6_ANGLE_RANGE) + MID_SERVO_PULSE, MIN_SERVO_PULSE, MAX_SERVO_PULSE)) ;
      new_command = true; // Notify this new serial command event for main loop
  }
  else if(temp == MOVE_CMD)
  {   
      /* Move relative to current position of angle values received. These are transformed from 6 
       *  angles to steps and pulse width, then passed to the corresponding servo/stepper object 
       *  for processing in the main loop
      */
      command = strtok(0, ",");
      axis1.move((long)atoi(command) * STEPPER_DRIVER_STEPS_PER_REV * AXIS_1_TRANS_RATIO / 360);
      command = strtok(0, ",");
      axis2.move((long)atoi(command) * STEPPER_DRIVER_STEPS_PER_REV * AXIS_2_TRANS_RATIO / 360);
      command = strtok(0, ",");
      axis3.move((long)atoi(command) * STEPPER_DRIVER_STEPS_PER_REV * AXIS_3_TRANS_RATIO / 360);
      command = strtok(0, ",");
      axis4Lim.setTarget(axis4Lim.getPosition()+ constrain(((long)atoi(command) * SERVO_PULSE_RANGE / SERVO_4_5_ANGLE_RANGE) + MID_SERVO_PULSE, MIN_SERVO_PULSE, MAX_SERVO_PULSE)) ;
      command = strtok(0, ",");
      axis5Lim.setTarget(axis5Lim.getPosition()+ constrain(((long)atoi(command) * SERVO_PULSE_RANGE / SERVO_4_5_ANGLE_RANGE) + MID_SERVO_PULSE, MIN_SERVO_PULSE, MAX_SERVO_PULSE)) ;
      command = strtok(0, ",");
      axis6Lim.setTarget(axis6Lim.getPosition()+ constrain(((long)atoi(command) * SERVO_PULSE_RANGE / SERVO_6_ANGLE_RANGE) + MID_SERVO_PULSE, MIN_SERVO_PULSE, MAX_SERVO_PULSE)) ;
      new_command = true;// Notify this new serial command event for main loop
      
  }
  else if(temp == STOP_CMD)
  {   
      /* Stop signal sent to the stepper motors. This sets a new target and use configured
       *  acceleration to reach 0 movement.
      */
      uint8_t dir;
      
      Serial.println("Stop received");
      // Stop stepper motor progressively
      axis1.stop();
      axis2.stop();
      axis3.stop();

      // Stop servo axis progressively
      if (axis4Lim.getVelocity() >= 0)
        axis4Lim.setTarget(axis4Lim.getPosition() - 1);
      else
        axis4Lim.setTarget(axis4Lim.getPosition() + 1);
        
      if (axis5Lim.getVelocity() >= 0)
        axis5Lim.setTarget(axis5Lim.getPosition() - 1);
      else
        axis5Lim.setTarget(axis5Lim.getPosition() + 1);
        
      if (axis6Lim.getVelocity() >= 0)
        axis6Lim.setTarget(axis6Lim.getPosition() - 1);
      else
        axis6Lim.setTarget(axis6Lim.getPosition() + 1);
      Serial.println(axis4Lim.getVelocity() > 0);
  }
  else if(temp == WRITE_STEP_AXIS_CFG_CMD)
  {   
      /* Write new stepper speed configuration to Arduino's EEPROM and also update them for 
       * current runtime. The values received are degrees/sec for max speed and degrees/(sec*sec) 
       * for acceleration.
      */
      command = strtok(0, ",");
      StepperConfig.axis1MaxSpeed = atoi(command);
      command = strtok(0, ",");
      StepperConfig.axis1Accel = atoi(command);
      command = strtok(0, ",");
      StepperConfig.axis2MaxSpeed = atoi(command);
      command = strtok(0, ",");
      StepperConfig.axis2Accel = atoi(command);
      command = strtok(0, ",");
      StepperConfig.axis3MaxSpeed = atoi(command);
      command = strtok(0, ",");
      StepperConfig.axis3Accel = atoi(command);
      EEPROM.put(STEP_AXIS_CONFIG_EEPROM_ADDR, StepperConfig);
      Stepper_AxisConfig();
  }
  else if(temp == READ_STEP_AXIS_CFG_CMD)
  {  
      /* Read current configuration written to EEPROM for the stepper motors axis speed
      */
      String txBuffer;
      EEPROM.get(STEP_AXIS_CONFIG_EEPROM_ADDR, StepperConfig);
      txBuffer.concat(StepperConfig.axis1MaxSpeed);
      txBuffer.concat(',');
      txBuffer.concat(StepperConfig.axis1Accel);
      txBuffer.concat(',');
      txBuffer.concat(StepperConfig.axis2MaxSpeed);
      txBuffer.concat(',');
      txBuffer.concat(StepperConfig.axis2Accel);
      txBuffer.concat(',');
      txBuffer.concat(StepperConfig.axis3MaxSpeed);
      txBuffer.concat(',');
      txBuffer.concat(StepperConfig.axis3Accel);
      Serial.println(txBuffer);
  }
  else if(temp == WRITE_MOVE_END_CFG_CMD)
  {  
      /* Write new stepper end movement threshold configuration to Arduino's EEPROM and also 
       *  update them for current runtime
      */
      int temp;
      command = strtok(0, ",");
      temp = atoi(command);

      // Check if the value is out of reasonable bounds
      if(temp > 255 || temp < 0)
      { // Set value to 0 if its out of bound
        temp = 0U;
      }
      ThresholdConfig.stepperThreshold = (uint8_t)temp;

      command = strtok(0, ",");
      temp = atoi(command);

      // Check if the value is out of reasonable bounds
      if(temp > 255 || temp < 0)
      { // Set value to 0 if its out of bound
        temp = 0U;
      }
      ThresholdConfig.servoThreshold = (uint8_t)temp;
      // Write this confing to EEPROM at address after AxisConfig
      EEPROM.put(MOVE_THRESHOLD_EEPROM_ADDR, ThresholdConfig);
  }
  else if(temp == READ_MOVE_END_CFG_CMD)
  {  
      /* Read current configuration written to EEPROM for the stepper motors end
       *  movement threshold
      */
      String txBuffer;
      EEPROM.get(MOVE_THRESHOLD_EEPROM_ADDR, ThresholdConfig);
      txBuffer.concat(ThresholdConfig.stepperThreshold);
      txBuffer.concat(',');
      txBuffer.concat(ThresholdConfig.servoThreshold);
      Serial.println(txBuffer);
  }
    else if(temp == WRITE_SERVO_AXIS_CFG_CMD)
  {   
      /* Write new servo speed configuration to Arduino's EEPROM and also update them for 
       * current runtime. The values received are degrees/sec for max speed and degrees/(sec*sec) 
       * for acceleration.
      */
      command = strtok(0, ",");
      ServoConfig.axis4MaxSpeed = atoi(command);
      command = strtok(0, ",");
      ServoConfig.axis4Accel = atoi(command);
      command = strtok(0, ",");
      ServoConfig.axis5MaxSpeed = atoi(command);
      command = strtok(0, ",");
      ServoConfig.axis5Accel = atoi(command);
      command = strtok(0, ",");
      ServoConfig.axis6MaxSpeed = atoi(command);
      command = strtok(0, ",");
      ServoConfig.axis6Accel = atoi(command);
      EEPROM.put(SERVO_AXIS_CONFIG_EEPROM_ADDR, ServoConfig);
      Servo_AxisConfig();
  }
  else if(temp == READ_SERVO_AXIS_CFG_CMD)
  {  
      /* Read current configuration written to EEPROM for the stepper motors axis speed
      */
      String txBuffer;
      EEPROM.get(SERVO_AXIS_CONFIG_EEPROM_ADDR, ServoConfig);
      txBuffer.concat(ServoConfig.axis4MaxSpeed);
      txBuffer.concat(',');
      txBuffer.concat(ServoConfig.axis4Accel);
      txBuffer.concat(',');
      txBuffer.concat(ServoConfig.axis5MaxSpeed);
      txBuffer.concat(',');
      txBuffer.concat(ServoConfig.axis5Accel);
      txBuffer.concat(',');
      txBuffer.concat(ServoConfig.axis6MaxSpeed);
      txBuffer.concat(',');
      txBuffer.concat(ServoConfig.axis6Accel);
      Serial.println(txBuffer);
  }
  else
  {
    Serial.println("bad command received");
  }
}


/*==================================================================================================
*                                        Runtime Functions
==================================================================================================*/

void setup() 
{ 
  // Start Serial communication
  Serial.begin(115200);
  while (!Serial){}; // wait for serial port to connect. Needed for native USB

  // Init Servo axis
  Servo_AxisInit();
  Servo_AxisConfig();
  // Init Stepper axis with last config data written to EEPROM
  EEPROM.get(STEP_AXIS_CONFIG_EEPROM_ADDR, StepperConfig);
  Stepper_AxisConfig();

  // Read EEPROM to use last written end movement threshold configuration from memory
  EEPROM.get(sizeof(STEP_AXIS_CONFIG_EEPROM_ADDR), ThresholdConfig);
}

void loop() // Main loop
{
  /* run each stepper axis in parallel, thus avoiding to wait for 
   *  each axe to finish before processing the next
  */
  axis1.run();
  axis2.run();
  axis3.run();

  /* run each servo with a limiter for accel/decel 
  */
  axis4.writeMicroseconds(axis4Lim.calc());
  axis5.writeMicroseconds(axis5Lim.calc());
  axis6.writeMicroseconds(axis6Lim.calc());
  //Serial.println(abs(axis4Lim.distToTarget()) <= SERVO_MOVE_END);
  
  /* Chec if each axis reached target threshold and send end of movement signal to jetson */
  if(new_command &&
      abs(axis1.distanceToGo()) <= ThresholdConfig.stepperThreshold &&
      abs(axis2.distanceToGo()) <= ThresholdConfig.stepperThreshold &&
      abs(axis3.distanceToGo()) <= ThresholdConfig.stepperThreshold &&
      abs(axis4Lim.distToTarget()) <= ThresholdConfig.servoThreshold &&
      abs(axis5Lim.distToTarget()) <= ThresholdConfig.servoThreshold &&
      abs(axis6Lim.distToTarget()) <= ThresholdConfig.servoThreshold)
  {
    Serial.println(69, DEC);
    new_command = false;
  }
}

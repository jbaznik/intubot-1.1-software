/*
    intuBot-1.1 
    Dave Gage - 27.04.2022
    (\/)(;,,;)(\/)
  
    For use with:
      - Teensy 3.2 (https://www.pjrc.com/teensy/td_download.html)
      - 6x DC Motors with current limit up to 0.1A each
      - motorDriver1.2
      - motorSensors1.1
      - Lenovo 720S-14IKBR laptop PC
      - intuBot 1.1 Software
      - intuBot 1.1 hardware UI

    Teensy commands:
      - recieves absolute rotary position from AS5048 via I2C.
      - monitors all UI analog and digital inputs.
      - directly drives the Button LED.
      - directly drives the display backlite LED.
      - directly drives the laryngoscope LED.
      - reports all values to the laptop PC over serial com.
      - recieves motor commands from workstation serial com.
      - sends I2C commands to PCA9634s to drive the motors.
      
    Teensy is connected to:
      - laptop via USB
      - Motor 5 Encoder
          - Encoder A --> Interrupt
          - Encoder B --> Digital Input
      - Positioning sensor via Interrupt 
      - PCA9634 Output Enable --> Digital Output
      - motorDriver1.2 via I2C
      - motorSensors1.1 via I2C
      - Auto mode button via Digital Input
      - Auto mode LED via Digital Output
      - Display backlite via Digital Output
      - Laryngoscope LED via Digital Output
      - 2x Thumbsticks + 1x analogRocker via 5x Analog Input
      - Power Switch via Analog Input
 */

/* 
 * Debugging nonsense
 * set true to output strings to serial com for human interface
 * set false for normal operation
 */
bool DEBUG = false;
bool PLOT = false;
bool MOTOR_ZERO = false;

//Libraries
#include <ams_as5048b.h> //https://github.com/sosandroid/AMS_AS5048B
#include <string.h>
#include <Wire.h>
#include <PCA9634.h>
#include <STUSB4500.h> // https://github.com/ardnew/STUSB4500
const float pi = 3.14159265359;

//System Parameters and Pins: as per intuBot-1.1 schematic
#define rx_pin             0
#define tx_pin             1
#define push_2_pin         2
#define push_1_pin         3
#define display_PWM_pin    4
#define auto_LED_pin       5  //Active LOW
#define auto_mode_pin      6
#define motor5_channel_A   7  //Interrupt
#define motor5_channel_B   8
#define OE_pin             9  //Active LOW
//#define cs_pin            10
//#define dout_pin          11
//#define din_pin           12

// Analog pin 14 assigned to pin 40. See lines 73 & 84:
// https://github.com/PaulStoffregen/cores/blob/master/teensy3/pins_arduino.h#L73
#define left_push_pin     A14
#define sck_pin           13
#define left_vert_pin     14
#define left_horz_pin     15
#define home_pos_pin      16 
#define power_sw_pin      17  
#define rocker_pin        20
#define right_horz_pin    21
#define right_vert_pin    22
#define right_push_pin    23

#define BUILTINLED     13
#define CAMLIGHT       32
//Bottom Pins
#define laryngo_LED_pin   32


//System Variables  
bool STOP = true;
bool AUTO_MODE = false;
bool RIGHT_BUTT = false;
bool LEFT_BUTT = false;
bool ROCKER_ALT1 = false;
bool ROCKER_ALT2 = false;
bool SLED_ZERO = false;
uint16_t left_horz = 0;
uint16_t left_vert = 0;
uint16_t right_horz = 0;
uint16_t right_vert = 0;
uint16_t analog_rocker = 0;
uint16_t power_switch = 0;
uint8_t auto_LED = 0;
uint8_t display_LED = 0;
uint8_t laryngo_LED = 255;

// Motor Sensor Constants for AS5048B
#define fourteenBit     16384.0 // per revolution
#define rotationBuffer   9000   // difference per step
const float sensorRatio = fourteenBit/360.0;

// Motor quadrature encoder
#define encoderRes       1470   // per revolution 

// Control Constants
float control_cap = 180;

// PWM driver Setup
PCA9634 motor_driver_1(0x20, OE_pin);
PCA9634 motor_driver_2(0x60, OE_pin);

// Flip between 1 and 0 to switch motor polarity
const int motor_polarity[5] = {-1, -1, -1, -1, 1};

// Container for controls written to the PWM driver
int motor_controls[5] = {0, 0, 0, 0, 0};

// Motor Constants
#define PWM_channel_F       0
#define PWM_channel_R       1
#define motorDriverNumber   2

const uint8_t motor1 = 0;
const uint8_t motor2 = 1;
const uint8_t motor3 = 2;
const uint8_t motor4 = 3;
const uint8_t motor5 = 4;

// Callable class i.e. motor_class[motor3][motorDriverNumber]
const uint8_t motor_class[5][3] ={{0, 1, 1},
                                  {2, 3, 1},
                                  {4, 5, 1},
                                  {6, 7, 1},
                                  {0, 1, 2}};

/*AS5048B Nonsense:
 *I2C address is from bits b'1 0 0 0 0' b'A2' b'A1'
 *  A2 = LOW,  A1 = LOW  --> b'1000000' = 0x40
 *  A2 = LOW,  A1 = HIGH --> b'1000001' = 0x41
 *  A2 = HIGH, A1 = LOW  --> b'1000010' = 0x42
 *  A2 = HIGH, A1 = HIGH --> b'1000011' = 0x43
 */
AMS_AS5048B motor1_sensor(0x40);
AMS_AS5048B motor2_sensor(0x41);
AMS_AS5048B motor3_sensor(0x42);
AMS_AS5048B motor4_sensor(0x43);

#define U_RAW 1

/*  //Motor Position Variables
 *  Only 4 motors use the AS5048B, motor5 uses a hall effect encoder 
 */
bool first_call = true;
float motor_positions[4] =     {0, 0, 0, 0};
float motor_positions_0[4] =   {0, 0, 0, 0};
float motor_rotations[4] =     {0, 0, 0, 0};
float motor_positions_out[4] = {0, 0, 0, 0};
//int16_t motor_positions_out[5] = {0, 0, 0, 0, 0};
float difference = 0;
float motor_sensor_midpoints[4] = {13401.18, 6865.21, 11570.79, 13100.67};//{13484.14, 12083.67, 11360.70, 13727.62};



 //Todo: center motors
volatile long motor5_position = 0;


// Serial Com
#define baudRate    115200  // Todo: too fast for long cable???
#define reportDelay       0    //40 Bytes * 8bits/byte / 115200bits/s ~ 3ms per systemReport
unsigned long report_time = 0;
String input_string = "";

long com_time = 0;


//Clock
long t0 = 0;
int dt = 1;
long encoder_clock = 0;

#define USBPD_RST_PIN  49
#define USBPD_ATCH_PIN 48
#define USBPD_ALRT_PIN 50

#define PDO_STR_LEN    48



STUSB4500 usbpd(USBPD_RST_PIN);

void usbpdCableAttached(void)   
{
  //Serial.println("cable attached");

  // set power to 5V 3A when cable re-attached
  usbpd.setPower(5000, 3000);
  usbpd.updateSinkCapabilities();
  usbpd.requestSourceCapabilities();
}

void usbpdCableDetached(void)
{
  //Serial.println("cable detached");
}

void usbpdCapabilitiesReceived(void)
{
  char pdoStr[PDO_STR_LEN];

  //Serial.println("source capabilities received:");

  size_t n = usbpd.sourcePDOCount();
  for (size_t i = 0U; i < n; ++i) {
    PDO pdo = usbpd.sourcePDO(i);
    snprintf(pdoStr, PDO_STR_LEN, "  %u: %umV %umA",
             pdo.number, pdo.voltage_mV, pdo.current_mA);
    //Serial.println(pdoStr);
  }

  //Serial.println("sink capabilities:");

  size_t m = usbpd.sinkPDOCount();
  for (size_t i = 0U; i < m; ++i) {
    PDO pdo = usbpd.sinkPDO(i);
    snprintf(pdoStr, PDO_STR_LEN, "  %u: %umV %umA",
             pdo.number, pdo.voltage_mV, pdo.current_mA);
    //Serial.println(pdoStr);
  }
}
bool setPD()
{   
  bool ret = false;
    
  // set power to 5V 3A
  ret = usbpd.setPower(5000, 3000);
  if (!ret)
   // Serial.println("usbpd.setPower(5000, 3000) failed");
    
  ret = usbpd.updateSinkCapabilities();
  if (!ret)
   // Serial.println("usbpd.updateSinkCapabilities() failed");

  ret = usbpd.requestSourceCapabilities();
  if (!ret)
   // Serial.println("usbpd.requestSourceCapabilities() failed");

  return ret;
}

void setup()
{
  //Initialize Serial
  Serial.begin(baudRate);
  input_string.reserve(40);

  //Initialize motorSensors1.1
  motor1_sensor.begin();
  motor2_sensor.begin();
  motor3_sensor.begin();
  motor4_sensor.begin();

  motor1_sensor.updateMovingAvgExp();
  motor2_sensor.updateMovingAvgExp();
  motor3_sensor.updateMovingAvgExp();
  motor4_sensor.updateMovingAvgExp();

  motor_positions[motor1] = motor1_sensor.getMovingAvgExp(U_RAW);
  motor_positions[motor2] = motor2_sensor.getMovingAvgExp(U_RAW);
  motor_positions[motor3] = motor3_sensor.getMovingAvgExp(U_RAW);
  motor_positions[motor4] = motor4_sensor.getMovingAvgExp(U_RAW);


  motor_positions_0[motor1] =   motor_positions[motor1];
  motor_positions_0[motor2] =   motor_positions[motor2];
  motor_positions_0[motor3] =   motor_positions[motor3];
  motor_positions_0[motor4] =   motor_positions[motor4];


  // Initialize Inputs
  pinMode(motor5_channel_A, INPUT);
  pinMode(motor5_channel_B, INPUT);
  pinMode(auto_mode_pin, INPUT);
  pinMode(left_horz_pin, INPUT);
  pinMode(left_vert_pin, INPUT);
  pinMode(left_push_pin, INPUT);
  pinMode(right_horz_pin, INPUT);
  pinMode(right_vert_pin, INPUT);
  pinMode(right_push_pin, INPUT);
  pinMode(rocker_pin, INPUT);
  pinMode(power_sw_pin, INPUT);
  pinMode(push_1_pin, INPUT);
  pinMode(push_2_pin, INPUT);
  pinMode(home_pos_pin, INPUT);

  // Initialize Outputs
  pinMode(OE_pin, OUTPUT);
  pinMode(auto_LED_pin, OUTPUT);
  pinMode(display_PWM_pin, OUTPUT);
  pinMode(laryngo_LED_pin, OUTPUT);

  digitalWrite(OE_pin, HIGH);
  analogWrite(auto_LED_pin, 255 - auto_LED);
  analogWrite(display_PWM_pin, display_LED);
  analogWrite(laryngo_LED_pin, laryngo_LED);
  


//  // Code for Available PWM frequency on D9 & D10: for PWM frequency of 31372.55 Hz
//  TCCR1B = TCCR1B & B11111000 | B00000001; 
//
//  //Code for Available PWM frequency on D3 & D11: for PWM frequency of 31372.55 Hz
//  TCCR2B = TCCR2B & B11111000 | B00000001; 

  //Interrupt for Encoder
  attachInterrupt(digitalPinToInterrupt(motor5_channel_A), motor5A, CHANGE);
  
  //Initialize motorDriver1.2, and make sure nothing is driving
  motor_driver_1.begin();
  motor_driver_2.begin();  
  killMotors();   

  usbpd.setCableAttached(usbpdCableAttached);
  usbpd.setCableDetached(usbpdCableDetached);
  usbpd.setSourceCapabilitiesReceived(usbpdCapabilitiesReceived);

  if (usbpd.begin(USBPD_ALRT_PIN, USBPD_ATCH_PIN))
  {
   // Serial.print("cee STUSB4500 v");
   // Serial.println(usbpd.version());

    // set power to default USB (5V 1.5A) initially
    usbpd.setPowerDefaultUSB();

    setPD();   
  }
  else
  {
    //Serial.println("failed to initialize STUSB4500");
    while (1) {
      delay(1000);
    }
  }   
  getPos(true);
  //Start clock
  t0 = millis();
}


void loop()
{
  // clock
  dt = millis()-t0;
  t0 = millis();

  // Prevent inf
  if(int(dt)==0){dt = 1.0;}
  
  // Read motor positions from sensors
  getPos(false);

  // Read the digital and analog inputs
  readUI();

  // Get incoming commands if serial available
  if (Serial.available())
  {
    com_time = millis();
    systemReceive();
  }

  // Shut down motors if STOP signal or com_time too long
  if(STOP)// || millis() - com_time > 500)
  {
    killMotors();
  }

  // Otherwise, drive the motors to the serial commands
  else
  {
    // Output Enable active LOW
    digitalWrite(OE_pin, LOW);

    // Send drive commands
    driveMotors();
  }

  // Update LEDs
  analogWrite(auto_LED_pin, 255 - auto_LED);
  analogWrite(display_PWM_pin, display_LED);
  analogWrite(laryngo_LED_pin, laryngo_LED);

  PDO pdo;
  
  // process interrupts
  usbpd.update();

//  ret = usbpd.requestSourceCapabilities();
//  if (!ret)
//    Serial.println("usbpd.requestSourceCapabilities failed");

  pdo = usbpd.requestedPDO();

  // Send variables to laptop PC
  systemReport();

  delay(10);
}

// Read motors 1 - 4 sensors
void getPos(bool first_call)
{
  //Read sensors
  motor1_sensor.updateMovingAvgExp();
  motor2_sensor.updateMovingAvgExp();
  motor3_sensor.updateMovingAvgExp();
  motor4_sensor.updateMovingAvgExp();
  
  motor_positions[motor1] = motor1_sensor.getMovingAvgExp(U_RAW);
  motor_positions[motor2] = motor2_sensor.getMovingAvgExp(U_RAW);
  motor_positions[motor3] = motor3_sensor.getMovingAvgExp(U_RAW);
  motor_positions[motor4] = motor4_sensor.getMovingAvgExp(U_RAW);

  // Check for rotation (0° --> 360°, or 360° --> 0°) for motors 1 - 4
  // Motor magnets should be set to minimize risk of rotation rollover
  if (first_call)
  {
      for(uint8_t motor = motor1; motor < motor5; motor++)
      {
        difference = motor_positions_0[motor] - motor_positions[motor];
        motor_positions_0[motor] = motor_positions[motor];
      }
  }
   
  for(uint8_t motor = motor1; motor < motor5; motor++)
  {
    difference = motor_positions_0[motor] - motor_positions[motor];
    motor_positions_0[motor] = motor_positions[motor];

    /* If the difference from the previous position to the current position is 
     * greater than the rotation buffer, then the sensor may have rolled over...
     * If the sensor rolled over from positive to negative, then the difference
     * will be a large positive value.
     * If the sensor rolled over from negative to positive, then the difference
     * will be a large negative value.
     */

    if(abs(difference) > rotationBuffer)
    {
      if(difference > 0){motor_rotations[motor] += 1;}
      else              {motor_rotations[motor] -= 1;}
    }
    
    // Account for midpoint and for number of rotations
     motor_positions_out[motor] = motor_polarity[motor] *( motor_positions[motor] - motor_sensor_midpoints[motor] + motor_rotations[motor]*fourteenBit);
    //motor_positions_out[motor] = int16_t(motor_positions[motor] - motor_sensor_midpoints[motor] + motor_rotations[motor]*fourteenBit);
  }
  
  // Motor 5 is just updated from the interrupt value
  //motor_positions_out[motor5] = motor5_position;
}


//Motor 5 Quadrature Encoder on A-channel change
void motor5A() 
{
  //Serial.print(digitalRead(motor5_channel_A));Serial.print(",");Serial.println(digitalRead(motor5_channel_B));
  if (digitalRead(motor5_channel_A) == digitalRead(motor5_channel_B))
  {
    motor5_position -= (long)1;
  }
  else
  {
    motor5_position += (long)1;
  }
}


// Kill all motors
void killMotors()
{
  //Output Enable inactive HIGH
  digitalWrite(OE_pin, HIGH);

  //Set all PWM channels to 0
  for(uint8_t motor = motor1; motor <= motor5; motor++)
  {
    //Reset Control Variables
    motor_controls[motor] = 0;

    if (motor_class[motor][motorDriverNumber] == 1)
    {
      motor_driver_1.pwm(motor_class[motor][PWM_channel_R], 0x00);
      motor_driver_1.pwm(motor_class[motor][PWM_channel_F], 0x00);  
    }
    else if (motor_class[motor][motorDriverNumber] == 2)
    {
      motor_driver_2.pwm(motor_class[motor][PWM_channel_R], 0x00);
      motor_driver_2.pwm(motor_class[motor][PWM_channel_F], 0x00);
    }
  }
}

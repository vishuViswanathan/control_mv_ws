#define VERSIONMSG "stepperControl01.ino"
#define ENAOBSTRUCTIONCHECK
#include <SPI.h>

// notes with [] show CNC shield pin


#define LED 5 // the blinking
#define NORMALBLINK 1000000
#define WARNINGBLINK 500000
#define PANICBLINK 100000

// pulse rate per second for different speeds
// pi sends the requires left and right speeds as pulses
// +ve if fwd and -ve is rev for both the wheels
 
#define MAXTIME 4.2E9
// remember micros wraps around 4,294,967,295


/*****************************************/
/****** setup Arduino ********************/
/*****************************************/
// The following of PORTB offset 0, specified here the digital pin number
// #define EN 8
#define ENOFFSET 0

// The following of PORTD The numbers match the digital Pin number
// Right Wheel - CNC Y-Axis,Left Wheel - Z-Axis
#define LDIRPINOFFSET 7 // Left Wheel stepper motor direction control 
#define RDIRPINOFFSET 6 // Right Wheel stepper motor direction control
#define LSTPPINOFFSET 4 // Left Wheel stepper control
#define RSTPPINOFFSET 3 // Right Wheel stepper control

#define LEDOFFSET 5 // [X_dir]

// Offsets for Analog pins fro PORTC
#define FRONTSENSOR1 0 // A0 [Abort]
#define FRONTSENSOR2 1 // A1 [Hold]

#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define XOR(x,y) (x=x^(1<<y))
#define READDIGITAL(x,y) (x&(1<<y))


// commands from Pi
#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define MOTOR_RAW_PWM  'o'
#define PING           'p'
#define RESET_ENCODERS 'r'
#define SERVO_WRITE    's'
#define SERVO_READ     't'
#define UPDATE_PID     'u'

#define LEFT_MOTOR  0
#define RIGHT_MOTOR 1

/* Stop the robot if it hasn't received a movement command
  in this number of milliseconds */
#define AUTO_STOP_INTERVAL 5000
long lastMotorCommand = AUTO_STOP_INTERVAL;

// Notes on CNC shield with DRV8825
// Resolution stepsize selectors M0, M1 and M2
// with M2 only ON, Microstep resolution is 1/16 step

void setupPins() {

  // setup motor pins
  SET(DDRD, LDIRPINOFFSET); CLR(PORTD, LDIRPINOFFSET);
  SET(DDRD, RDIRPINOFFSET); CLR(PORTD, RDIRPINOFFSET);
  SET(DDRD, LSTPPINOFFSET); CLR(PORTD, LSTPPINOFFSET);
  SET(DDRD, RSTPPINOFFSET); CLR(PORTD, RSTPPINOFFSET);
 
  SET(DDRD, LEDOFFSET); CLR(PORTD, LEDOFFSET);

  // enable CNC shield
  SET(DDRB, ENOFFSET); CLR(PORTB, ENOFFSET);
  
  // Front Obstruction Sensors, both are inputs
  CLR(DDRC, FRONTSENSOR1); 
  CLR(DDRC, FRONTSENSOR2);  
}

void defineTimer() {
	SET(PORTB, ENOFFSET); // disable stepper drivers
 
  //We are going to overwrite the Timer1 to use the stepper motors

  // STEPPER MOTORS INITIALIZATION
  // TIMER1 CTC MODE
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11); 
  TCCR1A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1A0); 
  TCCR1A &= ~(3<<COM1B0); 

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);  // pre-scaling is 8
//  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (5<<CS10) ;  // pre-scaling 1024

  OCR1A = 125;  // 16Khz This is preferred for 1/16 step on CNC hat
  // OCR1A = 100;  // 20Khz
  // OCR1A = 80;   // 25Khz
  // OCR1A = 60;   // 250 Hz with pre-scaling as 1024
  TCNT1 = 0;

  TIMSK1 |= (1<<OCIE1A);  // Enable Timer1 interrupt
  CLR(PORTB, ENOFFSET);   // Enable stepper drivers 
}

/***********************************/
/****** The Status blinker *********/
/***********************************/ 
// blink LED 
#define PIMESSAGEDELAY 10000000  // 10s for testing
unsigned long nowTime = micros();
unsigned long blinkInterval = NORMALBLINK; // microS
unsigned long nextTime;
bool blinkActive = true;
boolean cantMoveFwd = false;
boolean cantMoveRev = false;
unsigned long nextMessageFromPi = nowTime + PIMESSAGEDELAY;

void blink() {
  XOR(PORTD, LEDOFFSET);
}

void checkBlink() {
   if (blinkActive && nowTime >= nextTime) {
		blink();
		nextTime = nowTime + blinkInterval;
	}   
}

long r_motor_steps = 0l;
long l_motor_steps = 0l;


/***********************************/
/*********Handle control message form pi *******/
//*********************************/

// Taken from ROSAeduinoMV01.ino 

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];



// The arguments converted to integers
long arg1;
long arg2;

int moving = 0;

/* Clear the current command parameters */
void resetCommand() {
  cmd = 0;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

void resetEncoders() {
  resetEncoder(LEFT_MOTOR);
  resetEncoder(RIGHT_MOTOR);
}

void resetEncoder(int i) {
    if (i == LEFT_MOTOR){
      l_motor_steps=0L;
      return;
    } else { 
      r_motor_steps=0L;
      return;
    }
  }

long readEncoder(int n) {
  if (n == LEFT_MOTOR)
    return l_motor_steps;
  else if (n == RIGHT_MOTOR)
    return r_motor_steps;
  else 
    return 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  // char buff[50];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  // sprintf(buff, "In runCommand %c, %s, %s", cmd, argv1, argv2);
  // Serial.println(buff);
  switch(cmd) {   
    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT_MOTOR));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT_MOTOR));
      break;
    case RESET_ENCODERS:
      resetEncoders();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      setMotorSpeeds(arg1, arg2);
      if (arg1 == 0 && arg2 == 0) {
        moving = 0;
      }
      else moving = 1;
      Serial.println("OK"); 
      break;
    default:
      Serial.println("Invalid Command");
      break;
  }
}


/**************************/
/**** Motor Control *******/
/**************************/
// right motor is index 0 and left is index 1
uint16_t periodsCounter[2];      // counters for periods
uint16_t subPeriod[2][8];        // eight subperiodPaddings 
uint8_t subPeriodIndex[2];       // index for subperiodPaddings
int16_t requiredMotorSpeed[2];     // required speed of motors
int16_t actualMotorSpeed[2];     // actual speed of motors
uint8_t actualMotorDir[2];       // actual direction of steppers motors


#define ZERO_SPEED  65535
#define MAX_ACCEL   1000

// Divided into 8 sub-periods to increase the resolution at high speeds (short periods)
// subperiodPadding = ((1000 % vel)*8)/vel;
void calculateSubperiods(uint8_t motor) {
  uint8_t subperiodPadding;
  uint16_t absSpeed;
  uint8_t i;

  // char buff[100];
  // sprintf(buff, "calculateSubperiods: Motor %d, actualMotorSpeed %d",  motor, actualMotorSpeed[motor]);
  // Serial.println(buff);

  if (actualMotorSpeed[motor] == 0) {
    for (i=0; i<8; i++) {
      subPeriod[motor][i] = ZERO_SPEED;
    }  
    return;
  }
  
  #ifdef REVERSE_MOTORS_DIRECTION
    actualMotorDir[motor] = (actualMotorSpeed[motor] > 0) ? 1 : 0; 
  #else
    actualMotorDir[motor] = (actualMotorSpeed[motor] > 0) ? 0 : 1; 
  #endif  
  
  absSpeed = abs(actualMotorSpeed[motor]);

  subPeriod[motor][0] = 1000/absSpeed;
  for (i=1; i<8; i++) {
    subPeriod[motor][i] = subPeriod[motor][0];
  }  
  // Calculate the sub-period padding. 
  subperiodPadding = ((1000 % absSpeed)*8)/absSpeed;
  if (subperiodPadding > 0) {
    subPeriod[motor][1]++;
  }  
  if (subperiodPadding > 1) {
    subPeriod[motor][5]++;
  }  
  if (subperiodPadding > 2) {
    subPeriod[motor][3]++;
  }  
  if (subperiodPadding > 3) {
    subPeriod[motor][7]++;
  }  
  if (subperiodPadding > 4) {
    subPeriod[motor][0]++;
  }  
  if (subperiodPadding > 5) {
    subPeriod[motor][4]++;
  }  
  if (subperiodPadding > 6) {
    subPeriod[motor][2]++;
  }  
}

void setMotorSpeeds(int s_left, int s_right) {
  // char buff[100];
  // sprintf(buff, "setMotorSpeeds: l, r %d, %d",  s_left, s_right);
  // Serial.println(buff);
  setMotorSpeed(LEFT_MOTOR, s_left / 16);
  setMotorSpeed(RIGHT_MOTOR, s_right / 16);
}

void setMotorSpeed(uint8_t motorNum, int speed) {
  //int16_t speed = constrain(requiredMotorSpeed, -MAX_SPEED, MAX_SPEED);
  // the above can be limited by the caller 
  // LIMIT MAX ACCELERATION
  int16_t acceleration = speed - actualMotorSpeed[motorNum];
  if (acceleration > MAX_ACCEL) {
    actualMotorSpeed[motorNum] += MAX_ACCEL;
  } else if (acceleration < -MAX_ACCEL) {
    actualMotorSpeed[motorNum] -= MAX_ACCEL;
  } else {
    actualMotorSpeed[motorNum] = speed;
  } 
//   char buff[100];
//    sprintf(buff, "setMotorSpeed: %d, speed %d",  motorNum, speed);
//   Serial.println(buff);
//  sprintf(buff, "setMotorSpeed: %d, actualMotorSpeed %d",  motorNum, actualMotorSpeed[motorNum]);
//   Serial.println(buff);
  calculateSubperiods(motorNum);  // We use four subperiodPaddings to increase resolution

  // To save energy when its not running...
  if ((actualMotorSpeed[0] == 0) && (actualMotorSpeed[1] == 0)) {
	  SET(PORTB, ENOFFSET); // Disable motors
  } else {
	  CLR(PORTB, ENOFFSET); // Enable motors
  }  
}

/**************************************************/
/********** handle Motor Drives *******************/
/**************************************************/
long counter = 0;

ISR(TIMER1_COMPA_vect) 
{
//  if (counter++ > 1000) { // every 2 sc with 500 Hz
//counter = 0;
//    Serial.println("In ISR");
//  }
  // index 0 is Left Motor and 1 is Right Motor
  periodsCounter[LEFT_MOTOR]++;
  periodsCounter[RIGHT_MOTOR]++;
  
  if (periodsCounter[RIGHT_MOTOR] >= subPeriod[RIGHT_MOTOR][subPeriodIndex[RIGHT_MOTOR]]) {
    periodsCounter[RIGHT_MOTOR] = 0;
    
    if (subPeriod[RIGHT_MOTOR][0] != ZERO_SPEED) {
      if (actualMotorDir[RIGHT_MOTOR]) {
        SET(PORTD,RDIRPINOFFSET);  // DIR Motor 1
        r_motor_steps++;
      } else {
        CLR(PORTD,RDIRPINOFFSET);
        r_motor_steps--;
      }  
      // We need to wait at lest 200ns to generate the Step pulse...(?)
      subPeriodIndex[RIGHT_MOTOR] = (subPeriodIndex[RIGHT_MOTOR]+1)&0x07; // subPeriodIndex from 0 to 7
      
      SET(PORTD, RSTPPINOFFSET); // STEP Motor 1
      delayMicroseconds(1);
      CLR(PORTD, RSTPPINOFFSET);
    }
  }
  
  if (periodsCounter[LEFT_MOTOR] >= subPeriod[LEFT_MOTOR][subPeriodIndex[LEFT_MOTOR]]) {
    periodsCounter[LEFT_MOTOR] = 0;
    
    if (subPeriod[LEFT_MOTOR][0] != ZERO_SPEED) {
    
      if (actualMotorDir[LEFT_MOTOR]) {
        SET(PORTD, LDIRPINOFFSET);   // DIR Left
        l_motor_steps++;
      } else {
        CLR(PORTD, LDIRPINOFFSET);
        l_motor_steps--;
      }  
      subPeriodIndex[LEFT_MOTOR] = (subPeriodIndex[LEFT_MOTOR]+1)&0x07;
      
      SET(PORTD, LSTPPINOFFSET); // STEP Right Motor
      delayMicroseconds(1);
      CLR(PORTD, LSTPPINOFFSET);
    }
  }
}

void setup() {
  Serial.begin(57600);
  // Serial.println(VERSIONMSG);
  setupPins();
  // Serial.println("settng timer to 16kHz");
  defineTimer();
}


/************************************/
/****** The loop ********************/
/************************************/
long now = 0;

void loop() {
  nowTime = micros();
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();
    // Serial.print(chr);
    // Terminate a command with a CR
    if (chr == '\n') {
      // Serial.println("CR REceived");
 
      if (arg == 1) argv1[index] = 0;
      else if (arg == 2) {
        argv2[index] = 0;
        // Serial.print("argv1 ");
        // Serial.println(argv1);
        // Serial.print("argv2 ");
        // Serial.println(argv2);
      }
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      // Serial.print("arg after ' '");
      // Serial.println(arg);
      
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = 0;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    blinkInterval = PANICBLINK;
    setMotorSpeeds(0, 0);
    moving = 0;
  }
  else
    blinkInterval = NORMALBLINK;

  checkBlink();
}

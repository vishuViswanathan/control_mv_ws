#define VERSIONMSG "stepperControlSP03.ino"
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
#define READ_ENCODERS  'E'
#define MOTOR_SPEEDS   'S'
#define RESET_ENCODERS 'R'

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

volatile long r_motor_steps = 0l;
volatile long l_motor_steps = 0l;

// ========================================
// ====      SPI and Remote Control    ====
// ========================================

#define BUFFER_SIZE 100

char receiveBuf[BUFFER_SIZE];
char sendBuf[BUFFER_SIZE];
char tempBuf[BUFFER_SIZE];
#define CANTMOVEFWD 8
#define CANTMOVEREV 19

byte c = 0, b=0;
byte s = 0;
volatile byte receivePos;
volatile bool spiReceived;
volatile byte sendPos;
volatile bool sendDataReady;

void getSPIready(){
  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
  receivePos = 0;
  sendPos = 0;
  spiReceived = false;
  sendDataReady = false;
  SPI.attachInterrupt();
}

boolean all_params_taken = true;

ISR (SPI_STC_vect) {
  c = SPDR;
  //  Serial.print("In ISR sendpos:\t");
  // Serial.println(sendPos);
  // if (receivePos == 0) 
  //  prepareSendData();
  if (!spiReceived) {
    if (sendDataReady && sendPos < BUFFER_SIZE && (sendPos == receivePos)) {
      s = sendBuf[sendPos++];
      if (s == '\n') {
 //       SPDR = '\n';
        sendPos = 0;
        sendDataReady = false;
      }
 //     else
        SPDR = s;
    }
    if (receivePos < BUFFER_SIZE){
      if (c == '\n') {
        receiveBuf[receivePos++] = 0;
        // Serial.println(receiveBuf);
        spiReceived = true; 
        runCommand(receiveBuf);
        spiReceived = false;
      }
      else 
        receiveBuf [receivePos++] = c;
    }
  }
}

void prepareSendData() {
  if (!sendDataReady) {
    // Serial.println("Preparing Response");
    sprintf(sendBuf, "%+011ld %+011ld\n", l_motor_steps, r_motor_steps);
    Serial.println(sendBuf);
    sendDataReady = true;
    sendPos = 0;
  }
} 



/***********************************/
/*********Handle control message form pi *******/
//*********************************/

// The arguments converted to integers
long arg1;
long arg2;

int moving = 0;

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

/* Run a command.  Commands are defined in commands.h */
int runCommand(char *msg) {
  char buff[200];
  boolean retVal = false;
  Serial.println("In runCommad ");
  Serial.println(msg);
  switch(msg[0]) {   
    case RESET_ENCODERS:
      resetEncoders();
      break;
    case MOTOR_SPEEDS:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      retVal =  (sscanf(msg, "Speed l:%ld r:%ld", &arg1, &arg2) == 2);
      if (retVal) {
          Serial.println("Got Speed setting Now");
          sprintf(buff, "setMotorSpeeds: arg1 %ld, arg2 %ld",  arg1, arg2);
          Serial.println(buff);
  
          // setMotorSpeeds(arg1, arg2);
          setMotorSpeeds(arg1, arg2); 

        if (arg1 == 0 && arg2 == 0) {
          moving = 0;
        }
        else moving = 1;
      }
      break;
    case READ_ENCODERS:
      Serial.println("Got Encoder request");
      break;
    default:
      Serial.println("Invalid Command");
      break;
  }
	nextMessageFromPi = nowTime + PIMESSAGEDELAY;
 	receivePos = 0;
	prepareSendData();
	return retVal;

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
uint8_t actualMotorDir[2];       // actual direction of steppers motors true is Forwared


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
  
  actualMotorDir[motor] = (actualMotorSpeed[motor] > 0) ? 1 : 0; // true is Forwared

  
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
  setMotorSpeed(LEFT_MOTOR, s_left / 16);
  // modified to negate s_right)
  setMotorSpeed(RIGHT_MOTOR, -s_right / 16);
}

void resetSpeed() {
  setMotorSpeeds(0, 0);
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
      // for right motor the count is reversed 20231128
      if (actualMotorDir[RIGHT_MOTOR]) {
        SET(PORTD,RDIRPINOFFSET);  // DIR Motor right
        // noInterrupts();
        r_motor_steps -= 1;
        // interrupts();
      } else {
        CLR(PORTD,RDIRPINOFFSET);
        // noInterrupts();
        r_motor_steps += 1;
        // interrupts();
      }  
      // We need to wait at lest 200ns to generate the Step pulse...(?)
      subPeriodIndex[RIGHT_MOTOR] = (subPeriodIndex[RIGHT_MOTOR]+1)&0x07; // subPeriodIndex from 0 to 7
      
      SET(PORTD, RSTPPINOFFSET); // STEP Motor right
      delayMicroseconds(1);
      CLR(PORTD, RSTPPINOFFSET);
    }
  }
  
  if (periodsCounter[LEFT_MOTOR] >= subPeriod[LEFT_MOTOR][subPeriodIndex[LEFT_MOTOR]]) {
    periodsCounter[LEFT_MOTOR] = 0;
    
    if (subPeriod[LEFT_MOTOR][0] != ZERO_SPEED) {
    
      if (actualMotorDir[LEFT_MOTOR]) {
        SET(PORTD, LDIRPINOFFSET);   // DIR Motor left
        // noInterrupts();
        l_motor_steps += 1;
        // interrupts();
      } else {
        CLR(PORTD, LDIRPINOFFSET);
        // noInterrupts();
        l_motor_steps -= 1;
        // interrupts();
      }  
      subPeriodIndex[LEFT_MOTOR] = (subPeriodIndex[LEFT_MOTOR]+1)&0x07;
      
      SET(PORTD, LSTPPINOFFSET); // STEP Motor left
      delayMicroseconds(1);
      CLR(PORTD, LSTPPINOFFSET);
    }
  }
}

void setup() {
  Serial.begin(57600);
  Serial.println(VERSIONMSG);
  setupPins();
  Serial.println("settng timer to 16kHz");
  defineTimer();
  Serial.println("getSPIready");
  getSPIready();
  Serial.println("This is Arduino");
}


/************************************/
/****** The loop ********************/
/************************************/
long now = 0;

void loop() {
  while(true) {
	  nowTime = micros();
	  if (nowTime > nextMessageFromPi) {
			blinkInterval = PANICBLINK;
			// resetSpeed();
		}
		else {
		  blinkInterval = NORMALBLINK;
		}
	  checkBlink();
  }
}

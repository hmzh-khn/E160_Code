#define USE_TEENSY_HW_SERIAL
#include <vector>
#include <Encoder.h>

const int RDIR = 11;
const int RPWM = 10;
const int LDIR = 18;
const int LPWM = 20;
const int FrontDist = 15;
const int LeftDist = 16;
const int RightDist = 17;
const int ERA = 6; // Encoder pins
const int ERB = 7;
const int ELA = 8;
const int ELB = 9;

const int DEAD_ZONE_POWER = 8; //3*(256.0/100.0);

/* PID control constants */
const float Kp = 0.1;
const float Ki = 0.01;
const float Kd = 1;
const float Kpd = 0.1;

float leftTickErrorSum = 0;
float rightTickErrorSum = 0;

int motorValue = 0;
float desiredTickRatesPerSec[2] = {0, 0};

/* Timing Constants */
const int LOOP_DELAY_MS = 50;
const float UPDATE_PERIOD_SEC = 0.020;
const int SEC_TO_MS = 1000; // 10^3 ms per second
const int UPDATE_PERIOD_MS = (int) (UPDATE_PERIOD_SEC * SEC_TO_MS);
const int MS_TO_US = 1000; // 10^2 us per ms
//String currentCommand;

// character arrays for storing commands
char currentCommand[20];
int commandIndex = 0;

String commandData;
int sensorData;
int motorValues[4];
int deltaDegree = 360 / 1440; // 12 steps encoder;

long lastRAState = 0;
long lastLAState = 0;
Encoder LeftWheel(ERA, ERB);
Encoder RightWheel(ELA, ELB);

long lastRightEncPosition = RightWheel.read();
long lastLeftEncPosition = LeftWheel.read();
long rightEncPosition = RightWheel.read();
long leftEncPosition = LeftWheel.read();

IntervalTimer encoderTimer;

float controlRight = 0.0;
float controlLeft = 0.0;

void setup() {
    // Begin serial monitor port
    Serial.begin(9600);
    // Begin HW serial
    Serial1.begin(9600);
    // Pin 11: - right DIR
    // Pin 10: + right PWM
    // Pin 18: + left  DIR
    // Pin 20: - left  PWM/
    pinMode(RDIR, OUTPUT);
    pinMode(RPWM, OUTPUT);
    pinMode(LDIR, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(FrontDist, INPUT);
    pinMode(LeftDist, INPUT);
    pinMode(RightDist, INPUT);
//    char c[100] = "100 1 100 1";
    // Read the encoder state every 20 ms
    encoderTimer.begin(senseAndControl, UPDATE_PERIOD_MS * MS_TO_US);
}

void loop() {
  // If there is an incoming reading...
  if (Serial1.available() > 0) {
    noInterrupts();
    while(Serial1.available() > 0) {
      char currentChar = Serial1.read();
      if (currentChar == '$') {
        // if new command, empty the command buffer and set commandIndex to 0
        commandIndex = 0;
        memset(currentCommand, 0, 20);
        currentCommand[commandIndex] = currentChar;
      } 
      else if (currentChar == '@') {
         // Execute the command when @ is recieved 
         Serial.println(currentCommand);
         readCommand(); // read command either populates the motorValues or send sensor datas to the coordinator
      }
      else {
        // populate the command until '$'
         currentCommand[commandIndex] = currentChar;
         commandIndex++;
      }
    }
    interrupts();
  }
  digitalWrite(RDIR, motorValues[0]);
  digitalWrite(LDIR, motorValues[2]);
  analogWrite(RPWM, motorValues[1]);
  analogWrite(LPWM, motorValues[3]);
  //  sendSensorData();
  delay(LOOP_DELAY_MS);
}

void readCommand() {
  // Depending on command, send data or populate motor values
  if (currentCommand[0] == 'M') 
  {
    readMotorCommand(motorValues, currentCommand + 1);
  } 
  else if (currentCommand[0] == 'T')
  {
    readTickCommand(motorValues, currentCommand + 1);
  }
  else if (currentCommand[0] = 'S')
  {
    sendSensorData();
  }
}

void readMotorCommand(int cmd[], char *input) {
  const char s[2] = " ";
  char *token;
  /* get the first token */
  //   /Serial.printf( " %s\n", input );
  token = strtok(input, s);
  cmd[0] = atoi(token);
  int index = 1;
  
  //   /* walk through other tokens */
  while( token != NULL ) {
    //Serial.printf( " %s\n", token ); 
    token = strtok(NULL, s);
    cmd[index] = atoi(token);
    //      if (*token == 'S'){
    //        sendSensorData();
    //      }
    ++index;
  }
}

void readTickCommand(int motorCmds[], char *input) {
  const char s[2] = " ";
  char *token;
  /* get the first token */
  //   /Serial.printf( " %s\n", input );
  token = strtok(input, s);
  desiredTickRatesPerSec[0] = atoi(token);
  int index = 1;
  
  //   /* walk through other tokens */
  while( token != NULL ) {
    // Serial.printf( " %s\n", token ); 
    token = strtok(NULL, s);
    desiredTickRatesPerSec[index] = atoi(token);
    ++index;
  }
}

void senseAndControl() {
  updateEncoderState();
  controlTicks(motorValues);
}

void updateEncoderState() {
  lastLeftEncPosition = leftEncPosition;
  lastRightEncPosition = rightEncPosition;

  rightEncPosition = RightWheel.read();
  leftEncPosition = LeftWheel.read();
}

void controlTicks(int motorCmds[]) {
  // Get errors
//  int powerLeft = motorCmds[3];
//  int powerRight = motorCmds[1];
  float desiredRightTickRate = desiredTickRatesPerSec[0] * UPDATE_PERIOD_SEC;
  float desiredLeftTickRate = desiredTickRatesPerSec[1] * UPDATE_PERIOD_SEC;
  float actualRightTickRate = -(rightEncPosition - lastRightEncPosition);
  float actualLeftTickRate = -(leftEncPosition - lastLeftEncPosition);
  float errorRightTickRate = desiredRightTickRate - actualRightTickRate;
  float errorLeftTickRate = desiredLeftTickRate - actualLeftTickRate;

  // maintain consistent proportions
  float rightActualToDesiredProportion = 0;
  float leftActualToDesiredProportion  = 0;
  if (desiredRightTickRate*desiredLeftTickRate != 0) {
    float rightActualToDesiredProportion = actualRightTickRate/desiredRightTickRate;
    float leftActualToDesiredProportion  = actualLeftTickRate/desiredLeftTickRate;
  }
  float rightLeftProportionError = rightActualToDesiredProportion - leftActualToDesiredProportion;

  // PI controllers for each wheel
  leftTickErrorSum += errorLeftTickRate * UPDATE_PERIOD_SEC;
  controlLeft += Kp*errorLeftTickRate + Ki*leftTickErrorSum + Kpd*desiredLeftTickRate*rightLeftProportionError;

  rightTickErrorSum += errorRightTickRate * UPDATE_PERIOD_SEC;
  controlRight += Kp*errorRightTickRate + Ki*rightTickErrorSum - Kpd*desiredRightTickRate*rightLeftProportionError;


  

//   Serial.print(actualRightTickRate);
//   Serial.print(" R - L ");
//   Serial.println(actualLeftTickRate);
// //
//   Serial.print(controlRight);
//   Serial.print(" PR - PL ");
//   Serial.println(controlLeft);

//  Serial.print(errorRightTickRate);
//  Serial.print(" ER - EL ");
//  Serial.println(errorLeftTickRate);
//  Serial.println();

  // limit range to [-255, 255]
  controlRight = max(-255, min(controlRight, 255));
  controlLeft = max(-255, min(controlLeft, 255));

  controlRight = (desiredRightTickRate == 0)? 0:controlRight;
  controlLeft = (desiredLeftTickRate == 0)? 0:controlLeft;

  motorCmds[0] = controlLeft >= 0;
  motorCmds[1] = (int) fabs(controlLeft);
  // account for dead zone
  //motorCmds[1] = (motorCmds[1] <= DEAD_ZONE_POWER)? 0:motorCmds[1];
  if(motorCmds[1] <= DEAD_ZONE_POWER && motorCmds[1] > (rand()%DEAD_ZONE_POWER)) {
    motorCmds[1] = DEAD_ZONE_POWER;
  }
  motorCmds[2] = controlRight >= 0;
  motorCmds[3] = (int) fabs(controlRight);
  // motorCmds[3] = (motorCmds[3] <= DEAD_ZONE_POWER)? 0:motorCmds[3];
  if(motorCmds[3] <= DEAD_ZONE_POWER && motorCmds[3] > (rand()%DEAD_ZONE_POWER)) {
    motorCmds[3] = DEAD_ZONE_POWER;
  }
}

void sendSensorData() {
  int front = analogRead(FrontDist);
  int left = analogRead(LeftDist);
  int right = analogRead(RightDist);
  long leftWheel = LeftWheel.read();
  long rightWheel = RightWheel.read();
  //  LeftWheel.write(0);
  //  RightWheel.write(0);
  char message[50] = "";
  char temp[10];
  itoa(front, temp, 10);
  strcat(message, temp);
  strcat(message, " ");

  itoa(left, temp, 10);
  strcat(message, temp);
  strcat(message, " ");

  itoa(right, temp, 10);
  strcat(message, temp);
  strcat(message, " ");

  itoa(leftEncPosition, temp, 10);
  strcat(message, temp);
  strcat(message, " ");

  itoa(rightEncPosition, temp, 10);
  strcat(message, temp);
  strcat(message, " ");

  itoa(leftWheel, temp, 10);
  strcat(message, temp);
  strcat(message, " ");

  itoa(rightWheel, temp, 10);
  strcat(message, temp);
  strcat(message, " ");
  Serial1.println(message);
}

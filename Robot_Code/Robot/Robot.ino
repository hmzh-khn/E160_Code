#define USE_TEENSY_HW_SERIAL
#include <vector>
#include<Encoder.h>

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

/* PID control constants */
const float Kp = 0.5;
const float Ki = 0;
//const float Kd = 1;

float leftTickErrorSum = 0;
float rightTickErrorSum = 0;

int motorValue = 0;
int desiredTickRates[2] = {0, 0};

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

int signum(float num) {
  /*
   * Signum returns 1 for positive numbers, -1 for negative numbers. Else 0.
   */
  if     (num > 0) return  1;
  else if(num < 0) return -1;
  else             return  0;
}

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
  desiredTickRates[0] = atoi(token);
  int index = 1;
  
  //   /* walk through other tokens */
  while( token != NULL ) {
    // Serial.printf( " %s\n", token ); 
    token = strtok(NULL, s);
    desiredTickRates[index] = atoi(token);
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
  int powerLeft = motorCmds[3]; //why is this here?
  int powerRight = motorCmds[1];
  int desiredRightTickRate = 0; //desiredTickRates[0];
  int desiredLeftTickRate = 0; //desiredTickRates[1];
  int actualRightTickRate = -(rightEncPosition - lastRightEncPosition);
  int actualLeftTickRate = -(leftEncPosition - lastLeftEncPosition);
  int errorRightTickRate = desiredRightTickRate - actualRightTickRate;
  int errorLeftTickRate = desiredLeftTickRate - actualLeftTickRate;

  // PI controllers for each wheel
  leftTickErrorSum += errorLeftTickRate * UPDATE_PERIOD_SEC; //why multiply by the time...?
  powerLeft += Kp*errorLeftTickRate + Ki*leftTickErrorSum;

  rightTickErrorSum += errorRightTickRate * UPDATE_PERIOD_SEC;
  powerRight += Kp*errorRightTickRate + Ki*rightTickErrorSum;

  Serial.print(actualRightTickRate);
  Serial.print(" R - L ");
  Serial.println(actualLeftTickRate);

  Serial.print(powerRight);
  Serial.print(" PR - PL ");
  Serial.println(powerLeft);

  Serial.print(errorRightTickRate);
  Serial.print(" ER - EL ");
  Serial.println(errorLeftTickRate);

  Serial.println();

  motorCmds[0] = signum(powerRight);
  motorCmds[1] = abs(powerRight);
  motorCmds[2] = signum(powerLeft);
  motorCmds[3] = abs(powerLeft);
}

void sendSensorData() {
  int front = analogRead(FrontDist);
  int left = analogRead(LeftDist);
  int right = analogRead(RightDist);
  long leftWheel = LeftWheel.read();
  long rightWheel = RightWheel.read();
  //  LeftWheel.write(0);
  //  RightWheel.write(0);
  char message[30] = "";
  char temp[10];
  itoa(front, temp, 10);
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

#include <LinkedList.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

// motor analogue pins
#define MOT_A1_PIN 5
#define MOT_A2_PIN 6
#define MOT_B1_PIN 9
#define MOT_B2_PIN 10

int colour; 

// IR digital pins
int left = A3;
int centre = A2;
int right = A1;

// integers used to control speed of individual motors
int leftMotorSpeed;
int leftReverseSpeed;
int rightMotorSpeed;
int rightReverseSpeed;

// thresholds of the IR sensors which were decided based on its characterisation 
int lowThreshold = 300;
int highThreshold = 700;
// maximum speed of buggy
const int maxSpeed = 175;

// the values of the IR sensors
int sensorValueLeft, sensorValueCentre, sensorValueRight;

boolean initial = true;

// path list holds all the completed paths
LinkedList<int> path = LinkedList<int>();
// dfs holds uncompleted paths while buggy is mapping
LinkedList<int> dfs = LinkedList<int>();
// holds the current path that the buggy is traversing
int curPathArr[8];
// keeps count of how many paths the buggy has currently completed
int pathNumber = 0;
// counter keeps track of which fork of the path it is currently traversing (increases at every fork)
int counter = 0;
// an integer (could probably be a boolean) - keeps track whether the buggy is currently turning or not from a fork or end of line
int rightT = 0;
int forward = 0;
int leftT = 0;
// boolean that keeps track whether the buggy has reached the end of a path so it knows when to turn around
boolean endPath = false;
// turns true when buggy has traversed every path in the path list
boolean completed = false;
// keeps track of whether the buggy is mapping or not - done at the beginning of the program
boolean mapping = true;

// a linked list that keeps track of current and past positions of the buggy
// this is to be used with the dfs list to help map the paths
LinkedList<int> pos = LinkedList<int>();
// represents the position the buggy is facing (1 = north, 2 = west, 3 = south, 4 = east)
int curPos = 1;

void setup() {
  // put your setup code here, to run once:
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);
  pinMode(MOT_B1_PIN, OUTPUT);
  pinMode(MOT_B2_PIN, OUTPUT);
  // Start with drivers off, motors coasting.
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);

  Serial.begin(9600);

  // ignore its old code

  // adding paths to linkedlist
//  path.add(17);
//  path.add(126);
//  path.add(229);
//  path.add(326);
//  path.add(38);

  //convToArray(pathNumber);

 if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  
}

// retrieves IR values
void scan() {
  sensorValueLeft = analogRead(left) * 0.96;
  sensorValueCentre = analogRead(centre);
  sensorValueRight = analogRead(right) * 1.03;
  //movement(sensorValueLeft, sensorValueCentre, sensorValueRight);
}

void movement() {
  // retrieves IR values
  scan();

  //if the buggy is currently not turning
  if (rightT == 0 && forward == 0 && leftT == 0) {
    if (sensorValueCentre > lowThreshold) { //if buggy is centred
      leftMotorSpeed = maxSpeed - 4000/sensorValueRight; 
      rightMotorSpeed = maxSpeed - 4000/sensorValueLeft;
      leftReverseSpeed = 0;
      rightReverseSpeed = 0;

      // if the buggy has reached a fork in its path
      if ((sensorValueRight > lowThreshold && sensorValueLeft > lowThreshold) && sensorValueCentre > highThreshold) {
        if (mapping) {
          // calls this function if its trying to map routes
          mappingRoutes();
        }
        else {
          // calls this function if it is trying to locate an object
          choosePath(); 
        }
      }
    }
    else{ // if buggy has come away from the line (eg buggy needs additional correcting, buggy has come across an object (this will change), buggy is at base (will also change))
      if (sensorValueRight > lowThreshold) { //if buggy needs additional correction to the right - should hardly happen
        rightMotorSpeed = 0;
        rightReverseSpeed = maxSpeed - 10000/(sensorValueRight-200);
        leftMotorSpeed = maxSpeed - 4000/sensorValueRight; 
        leftReverseSpeed = 0;
      }
      else if (sensorValueLeft > lowThreshold) { // buggy needs additional correcting to the left - should hardly happen
        leftMotorSpeed = 0;
        leftReverseSpeed = maxSpeed - 10000/(sensorValueLeft-200);
        rightMotorSpeed = maxSpeed - 4000/sensorValueLeft;
        rightReverseSpeed = 0;
      }
      else { //if buggy reaches end of path (so left, centre and right sensors are all picking up white
        // uses this condition if the buggy is currently mapping the paths
        if (mapping) {
          
          // check colour (eg red = 5.....)
          // this is where the colour sensor needs to detect the colour at the end of the path
          // I would recommend calling a function here to keep code neat and tidy

          // if the buggy has reached the end of a path it will add the path integer to the list of paths (this path list can then be later manipulated)
          path.add(dfs.get(0)*10+4);
          Serial.println(dfs.get(0)*10+4);
          dfs.shift();

          // turns left 180 degrees to get back onto line
          rightMotorSpeed = 100;
          rightReverseSpeed = 0;
          leftMotorSpeed = 0; 
          leftReverseSpeed = 150;
          steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
          leftT = 1;

          // adds old position to position list
          pos.add(curPos);

          // changes the current position integer of the buggy
          if (curPos == 3) {
            curPos = 1;
          }
          else if (curPos == 1) {
            curPos = 3;
          }
          else if (curPos == 2) {
            curPos = 4;
          }
          else if (curPos == 4) {
            curPos = 2;
          }

          // the dfs list will be empty when it returns back to base
          if (dfs.size() == 0) {
            mapping = false;
            initial = true;
          }
        }
        else {
          if (initial) {
            // this means it has reached base after finishing mapping the routes
            leftT = 1;
            rightMotorSpeed = 100;
            rightReverseSpeed = 0;
            leftMotorSpeed = 0; 
            leftReverseSpeed = 150;
          
            initial = false;
            counter = 0;
            pathNumber = 0;

            // converts the first path integer into an array
            convToArray(pathNumber);
            //endPath = false;
          }
          else if (curPathArr[counter] == 0) {
            endPath = true;
            rightMotorSpeed = 0;
            rightReverseSpeed = 0;
            leftMotorSpeed = 0; 
            leftReverseSpeed = 0;
            steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
            delay(1500);
            rightMotorSpeed = 90;
            rightReverseSpeed = 0;
            leftMotorSpeed = 0; 
            leftReverseSpeed = 90;
            steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
            leftT = 1;
            counter--;
            changeDirection();
          }
          else if (curPathArr[counter] > 3) {
            endPath = false;
            rightMotorSpeed = 90;
            rightReverseSpeed = 0;
            leftMotorSpeed = 0; 
            leftReverseSpeed = 90;
            leftT = 1;
            if (pathNumber < 4) {
              pathNumber++;
            }
            else {
              completed = true;
              steer(200, 0, 0, 200);
              delay(2000);
            }
            convToArray(pathNumber);
            counter = 0;
          }
        }
      } 
    }
  }
  else { // dont correct until finished turning
    if (sensorValueLeft > highThreshold && sensorValueCentre < lowThreshold && sensorValueRight < lowThreshold) {
      leftT = 0;
      forward = 0;
    }
    else if (sensorValueLeft < lowThreshold && sensorValueCentre > highThreshold && sensorValueRight < lowThreshold) {
      forward = 0;
    }
    else if (sensorValueLeft < lowThreshold && sensorValueCentre < lowThreshold && sensorValueRight > highThreshold) {
      rightT = 0;
      forward = 0;
    }
  }
  
  steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
  
}

void mappingRoutes() {
  
  pos.add(curPos);
//
//  for (int i = 0; i < pos.size(); i++) {
//    Serial.println(pos.get(i));
//  }
//  Serial.println();


  //int curPos = pos.get(pos.size()-1); // receieve last position
  //Serial.println(curPos);
  int prevPos;
  
  if (pos.size() > 1) {
    prevPos = pos.get(pos.size()-2);
  }
  else {
    prevPos = curPos;
  }

  if (!(abs(prevPos - curPos) == 2)) {
    if (dfs.size() == 0) {
      dfs.add(1);
      dfs.add(2);
      dfs.add(3);
    }
    else {
      int p = dfs.shift();
      dfs.add(0, p*10+3);
      dfs.add(0, p*10+2);
      dfs.add(0, p*10+1);
    }
  }
  else {
    pos.pop();
    pos.pop();
  }

//  for (int i = 0; i < dfs.size(); i++) {
//    Serial.println(dfs.get(i));
//  }

  leftT = 1;

  if (curPos == 4) {
    curPos = 1;
  }
  else {
    curPos += 1;
  }

  choosePath();
}

void choosePath() {
  //Serial.println(curPathArr[counter]);
  //Serial.println(counter);
  if (curPathArr[counter] == 1 || leftT == 1) {
    rightMotorSpeed = 150;
    rightReverseSpeed = 0;
    leftMotorSpeed = 0; 
    leftReverseSpeed = 0;
    leftT = 1;
  }
  else if (curPathArr[counter] == 2 || forward == 1) {
    rightMotorSpeed = 130;
    rightReverseSpeed = 0;
    leftMotorSpeed = 135; 
    leftReverseSpeed = 0;
    forward = 1;
  }
  else if (curPathArr[counter] == 3 || rightT == 1) {
    rightMotorSpeed = 0;
    rightReverseSpeed = 0;
    leftMotorSpeed = 150; 
    leftReverseSpeed = 0;
    rightT = 1;
  }
  steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
  if (!endPath) {
    counter++;
  }
  else {
    counter--;
  }
}

void convToArray(int pathNo) {
  //int curPath = paths[pathNo];
  int curPath = path.get(pathNo);
  //Serial.print("Getting path");
  //Serial.println(curPath);
  int reverseArr[8];
  for (int j=0; j<8; j++) {
    curPathArr[j] = 0;
    reverseArr[j] = 0;
  }
  int i=0;
  int y;
  while ( curPath > 0 ) {
      y=curPath/10;
      curPathArr[i] = curPath-(10*y);
      reverseArr[i] = curPath-(10*y);
      curPath=y;
      i++;
  }

  i = 0;
  while (curPathArr[i] != 0) {
    i++;
  }
  
  int endOfPathIndex = i-1;

  for (int j=0; j<endOfPathIndex; j++) {
    curPathArr[j+1] = reverseArr[endOfPathIndex-j];
  }

  for (int i=0; i<8; i++) {
    Serial.println(curPathArr[i]);
  }
  Serial.println();
  
  counter++;

}

void changeDirection() {
  int i = 0;
  while (!curPathArr[i] == 0) {
    if (curPathArr[i] == 1) {
      curPathArr[i] = 3;
    }
    else if (curPathArr[i] == 3) {
      curPathArr[i] = 1;
    }
    i++;
  }
}

void steer(int m1p1, int m1p2, int m2p1, int m2p2) {
  analogWrite(MOT_A1_PIN, m1p1);
  analogWrite(MOT_A2_PIN, m1p2);
  analogWrite(MOT_B1_PIN, m2p1);
  analogWrite(MOT_B2_PIN, m2p2);
}

int detectColour( int r ,int g , int b, int colorTemp, int c){
 
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);

  /*
  if(r >= 120 & r <= 160 & g >= 20 & g <= 60 & b >= 20 & b <= 60) {
   colour = 4;
    }

  if(r >= 50 & r <= 80 & g  >= 70 & g <= 100 & b >= 40 & b <= 70){
   colour = 5;
   }
 
  if(r >= 200 & r <= 245 & g>= 155 & g <=200 & b >= 60 & b <= 105){
   colour = 6;
    }
    */

 return colour;
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t r, g, b, c, colorTemp;
  tcs.getRawData(&r, &g, &b, &c);

  colour = detectColour(r, g, b, colorTemp, c);
  
  if (!completed) {
    movement();
    Serial.println("Line");
  }
//  if (colour == 4) {
//    steer(200, 0, 0, 200);
//    completed = true;
//    Serial.println("RED");
//  }
  
  delay(2);

}

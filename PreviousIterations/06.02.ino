#include <LinkedList.h>

#define MOT_A1_PIN 5
#define MOT_A2_PIN 6
#define MOT_B1_PIN 9
#define MOT_B2_PIN 10

int left = A3;
int centre = A2;
int right = A1;

int leftMotorSpeed;
int leftReverseSpeed;
int rightMotorSpeed;
int rightReverseSpeed;

int lowThreshold = 300;
int highThreshold = 700;
const int maxSpeed = 175;

int sensorValueLeft, sensorValueCentre, sensorValueRight;

boolean initial = true;

//int paths[] = {17, 126, 229, 326, 38};
LinkedList<int> path = LinkedList<int>();
LinkedList<int> dfs = LinkedList<int>();
int curPathArr[8];
int pathNumber = 0;
int counter = 0;
int rightT = 0;
int forward = 0;
int leftT = 0;
boolean endPath = false;
boolean completed = false;
boolean mapping = true;

LinkedList<int> pos = LinkedList<int>();
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

  // adding paths to linkedlist
//  path.add(17);
//  path.add(126);
//  path.add(229);
//  path.add(326);
//  path.add(38);

  //convToArray(pathNumber);
}

void scan() {
  sensorValueLeft = analogRead(left) * 0.96;
  sensorValueCentre = analogRead(centre);
  sensorValueRight = analogRead(right) * 1.03;
  //movement(sensorValueLeft, sensorValueCentre, sensorValueRight);
}

void movement() {
  //int selectSide = 0; //0 is nothing, 1 is left, 2 is right
  scan();
  
  if (rightT == 0 && forward == 0 && leftT == 0) {
    if (sensorValueCentre > lowThreshold) { //if buggy is centred
      leftMotorSpeed = maxSpeed - 4000/sensorValueRight; 
      rightMotorSpeed = maxSpeed - 4000/sensorValueLeft;
      leftReverseSpeed = 0;
      rightReverseSpeed = 0;
  
      if ((sensorValueRight > lowThreshold && sensorValueLeft > lowThreshold) && sensorValueCentre > highThreshold) { //if buggy reaches a fork
        if (mapping) {
          mappingRoutes();
        }
        else {
          choosePath(); 
        }
      }
    }
    else{
      if (sensorValueRight > lowThreshold) { //if buggy needs additional correction
        rightMotorSpeed = 0;
        rightReverseSpeed = maxSpeed - 10000/(sensorValueRight-200);
        leftMotorSpeed = maxSpeed - 4000/sensorValueRight; 
        leftReverseSpeed = 0;
      }
      else if (sensorValueLeft > lowThreshold) {
        leftMotorSpeed = 0;
        leftReverseSpeed = maxSpeed - 10000/(sensorValueLeft-200);
        rightMotorSpeed = maxSpeed - 4000/sensorValueLeft;
        rightReverseSpeed = 0;
      }
      else { //if buggy reaches end of path
        if (mapping) {
          // check colour (eg red = 5.....)
          path.add(dfs.get(0)*10+4);
          //Serial.println(dfs.get(0)*10);
          //Serial.println(path.get(0));
          dfs.shift();
          rightMotorSpeed = 100;
          rightReverseSpeed = 0;
          leftMotorSpeed = 0; 
          leftReverseSpeed = 150;
          steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
          leftT = 1;
          pos.add(curPos);
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

          if (dfs.size() == 0) {
            mapping = false;
            initial = true;
          }
        }
        else {
          if (initial) {
            leftT = 1;
            rightMotorSpeed = 100;
            rightReverseSpeed = 0;
            leftMotorSpeed = 0; 
            leftReverseSpeed = 150;
          
            initial = false;
            counter = 0;
            pathNumber = 0;
            convToArray(pathNumber);
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
              rightMotorSpeed = 200;
              rightReverseSpeed = 0;
              leftMotorSpeed = 0; 
              leftReverseSpeed = 200;
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
  Serial.println(curPathArr[counter]);
  Serial.println(counter);
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
  Serial.println(curPath);
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

void loop() {
  // put your main code here, to run repeatedly:
  if (!completed) {
    movement();
  }
  delay(2);

}

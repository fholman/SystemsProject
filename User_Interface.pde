import processing.serial.*;
Serial myPort;

String[] messages = new String[25]; // holds messages used in buggy status tab
int yPosMes = 55; // y coordinate of first message
int numOfMes = 0; // keeps track of number of messages in the status tab

int press = 0; // keeps track if button has been pressed

// keeps track of opacity of the button
int green = 100;
int red = 100;
int yellow = 100;
int all = 100;

String[] pathDis = new String[15]; // holds possible paths buggy can traverse
int numOfPaths = 0; // keeps track of number of paths
int yPosPaths = 195; // y coordinate of the first path

boolean manual = false; // keeps track if buggy is in manual mode or not

void setup(){
  size(1000, 500); // screen size
  myPort = new Serial(this, "COM18", 9600); // Starts the serial communication
  myPort.bufferUntil('\n'); // Defines up to which character the data from the serial port will be read. The character '\n' or 'New Line'
}

void layout() {
  background(207, 200, 221); // background colour
  stroke(255);
  strokeWeight(4);
  
  // manual control button layout
  fill(0); // black
  rect(20, 380, 100, 100); // left button
  rect(130, 380, 100, 100); // backwards button
  rect(240, 380, 100, 100); // right button
  rect(130, 270, 100, 100); // forwards button
  rect(20, 270, 100, 45); // pickup button
  rect(20, 325, 100, 45); // place down button
  rect(240, 270, 100, 100); // alert button
  fill(255); // white
  triangle(180, 280, 140, 320, 220, 320); // forwards arrow
  rect(160, 320, 40, 40);
  triangle(180, 470, 140, 430, 220, 430); // backwards arrow
  rect(160, 390, 40, 40);
  triangle(30, 430, 70, 390, 70, 470); // left arrow
  rect(70, 410, 40, 40);
  triangle(330, 430, 290, 390, 290, 470); // right arrow
  rect(250, 410, 40, 40);
  
  fill(0); // black
  rect(750, 0, 250, 500); // buggy status
  rect(360, 140, 389, 360); // paths found
  
  fill(255, 0, 0, red); // red
  rect(20, 20, 100, 100); // red button
  fill(0, 255, 0, green); // green
  rect(130, 20, 100, 100); // green button
  fill(255, 255, 0, yellow); // yellow
  rect(240, 20, 100, 100); // yellow button
  fill(0, 0, 0, all); // black
  rect(350, 20, 100, 100); // black button
  fill(0, 0, 0); // black
  rect(20, 130, 100, 75); // map button
  rect(130, 130, 100, 75); // back to base button
  rect(240, 130, 100, 75); // force stop button
  
  fill(255); // white
  
  // text
  textSize(50);
  text("ALL", 365, 85);
  textSize(16);
  text("MAP", 57, 173);
  text("BACK", 165, 163);
  text("TO BASE", 153, 183);
  text("FORCE", 268, 163);
  text("STOP", 273, 183);
  text("PICK UP", 43, 298);
  text("PLACE DOWN", 27, 352);
  text("SPEAKER", 260, 325);
  textSize(16);
  text("Buggy Status:", 775, 25);
  text("Found Paths:", 385, 165);
  strokeWeight(1);
  rect(750, 35, 250, 3);
  rect(360, 175, 389, 3);
}

void draw(){
  
  while (myPort.available() > 0) { // checks for bluetooth connection
    int inByte = 0;
    inByte = myPort.read(); // reads from serial

    if (inByte == 4) {
      messages[numOfMes] = "Red Detected";
      red = 1000;
      all = 1000;
      numOfMes++;
    }
    else if (inByte == 5) {
      messages[numOfMes] = "Green Detected";
      green = 1000;
      all = 1000;
      numOfMes++;
    }
    else if (inByte == 6) {
      messages[numOfMes] = "Yellow Detected";
      yellow = 1000;
      all = 1000;
      numOfMes++;
    }
    else if (inByte == 8) {
      messages[numOfMes] = "No Colour Detected";
      numOfMes++;
    }
    else if (inByte == 11) {
      messages[numOfMes] = "Going Forwards";
      numOfMes++;
    }
    else if (inByte == 12) {
      messages[numOfMes] = "Turning Right";
      numOfMes++;
    }
    else if (inByte == 13) {
      messages[numOfMes] = "Turning Around";
      numOfMes++;
    }
    else if (inByte == 14) {
      messages[numOfMes] = "Turning Left";
      numOfMes++;
    }
    else if (inByte == 53) {
      messages[numOfMes] = "Not at Base";
      numOfMes++;
    }
    else if (inByte == 54) {
      messages[numOfMes] = "Not on Line";
      numOfMes++;
    }
    else if (inByte == 55) {
      messages[numOfMes] = "Rerouting to Base";
      numOfMes++;
    }
    else if (inByte == 56) {
      messages[numOfMes] = "Stored Mapping May be Incorrect";
      numOfMes++;
    }
    else if (inByte == 58) {
      messages[numOfMes] = "Mapping";
      numOfMes++;
      for (int i=0; i<15; i++) { // clears path list displayed on UI if buggy will remap
        pathDis[i] = "";
      }
      numOfPaths = 0;
      yPosPaths = 195;
    }
    else if (inByte == 59) {
      messages[numOfMes] = "Collecting Red";
      numOfMes++;
    }
    else if (inByte == 60) {
      messages[numOfMes] = "Collecting Green";
      numOfMes++;
    }
    else if (inByte == 61) {
      messages[numOfMes] = "Collecting Yellow";
      numOfMes++;
    }
    else if (inByte == 62) {
      messages[numOfMes] = "Collecting All";
      numOfMes++;
    }
    else if (inByte == 63) {
      messages[numOfMes] = "Picking up Red Object";
      numOfMes++;
    }
    else if (inByte == 64) {
      messages[numOfMes] = "Picking up Green Object";
      numOfMes++;
    }
    else if (inByte == 65) {
      messages[numOfMes] = "Picking up Yellow Object";
      numOfMes++;
    }
    else if (inByte == 66) {
      messages[numOfMes] = "Returned to Base";
      numOfMes++;
    }
    else if (inByte == 67) {
      messages[numOfMes] = "Colour not Present - Please Map First";
      numOfMes++;
    }
    else if (inByte == 68) {
      messages[numOfMes] = "Returning to Base";
      numOfMes++;
    }
    else if (inByte == 70) {
      messages[numOfMes] = "Placing Down Object";
      numOfMes++;
    }
    else if (inByte == 71) {
      messages[numOfMes] = "Object Detected";
      numOfMes++;
    }
    else if (inByte == 72) {
      messages[numOfMes] = "Starting Up ....";
      numOfMes++;
    }
    else if (inByte == 1) { // if received 1 this means a flow of bits are being sent over representing the path
      String path = "";
      while (inByte != 0) { // a sent 0 bit represents a finished path
        inByte = 1000;
        inByte = myPort.read(); // read from port again
        delay(10);
        if (inByte == 1) {
          path = "Left --- " + path;
        }
        else if (inByte == 2) {
          path = "Forwards --- " + path;
        }
        else if (inByte == 3) {
          path = "Right --- " + path;
        }
        else if (inByte == 4) {
          path = "Red" + path;
        }
        else if (inByte == 5) {
          path = "Green" + path;
        }
        else if (inByte == 6) {
          path = "Yellow" + path;
        }
        else if (inByte == 8) {
          path = "No Colour" + path;
        }
        
      }
      
      pathDis[numOfPaths] = path; // path string is added to path list
      numOfPaths++; // increase number of paths
    }
     
  }
  
  layout(); // draws the layout
  
  yPosMes = 55;
  yPosPaths = 195;
  
  while (numOfMes > 22) { // will remove top messages if list is about to fill up
    for (int i=0; i < 25; i++) {
      if (i != 24) {
        messages[i] = messages[i+1];
      }
      else {
        messages[i] = null;
      }
    }
    numOfMes--;
  }
  
  for (String str : messages) { // displays buggy status
    if (str != null) {
      textSize(12);
      text(str, 775, yPosMes);
      yPosMes += 20;
    }
  }
  
  for (String str : pathDis) { // displays paths
    if (str != null) {
      textSize(12);
      text(str, 385, yPosPaths);
      yPosPaths += 20;
    }
  }

  if(mousePressed && mouseX>20 && mouseX<120 && mouseY>380 && mouseY<480){ // if manual left button is pressed
    myPort.write('0');
    manual = true;
    // Highlights the buttons in red color when pressed
    stroke(255,0,0);
    strokeWeight(2);
    noFill();
    rect(20, 380, 100, 100);
    if (press != 1) {
      messages[numOfMes] = "Manual Left";
      numOfMes++;
    }
    press = 1;
  }

  if(mousePressed && mouseX>130 && mouseX<230 && mouseY>380 && mouseY<480){ // if manual backwards button is pressed
    myPort.write('1');
    manual = true;
    // Highlights the buttons in red color when pressed
    stroke(255,0,0);
    strokeWeight(2);
    noFill();
    rect(130, 380, 100, 100);
    if (press != 1) {
      messages[numOfMes] = "Manual Backwards";
      numOfMes++;
    }
    press = 1;
  }
  
  if(mousePressed && mouseX>240 && mouseX<340 && mouseY>380 && mouseY<480){ // if manual right button is pressed
    myPort.write('2');
    manual = true;
    // Highlights the buttons in red color when pressed
    stroke(255,0,0);
    strokeWeight(2);
    noFill();
    rect(240, 380, 100, 100);
    if (press != 1) {
      messages[numOfMes] = "Manual Right";
      numOfMes++;
    }
    press = 1;
  }
  
  if(mousePressed && mouseX>130 && mouseX<230 && mouseY>270 && mouseY<370){ // if manual forwards button is pressed
    myPort.write('3');
    manual = true;
    // Highlights the buttons in red color when pressed
    stroke(255,0,0);
    strokeWeight(2);
    noFill();
    rect(130, 270, 100, 100);
    if (press != 1) {
      messages[numOfMes] = "Manual Forward";
      numOfMes++;
    }
    press = 1;
  }
  
  if(mousePressed && mouseX>20 && mouseX<120 && mouseY>270 && mouseY<315){ // picking up
    // Highlights the buttons in red color when pressed
    stroke(255,0,0);
    strokeWeight(2);
    noFill();
    rect(20, 270, 100, 45);
    if (press != 1) {
      myPort.write('6');
    }
    press = 1;
  }
  
  if(mousePressed && mouseX>20 && mouseX<120 && mouseY>325 && mouseY<370){ // placing down
    // Highlights the buttons in red color when pressed
    stroke(255,0,0);
    strokeWeight(2);
    noFill();
    rect(20, 325, 100, 45);
    if (press != 1) {
      myPort.write('7');
    }
    press = 1;
  }
  
  if(mousePressed && mouseX>240 && mouseX<340 && mouseY>270 && mouseY<370){ // manually alerts surroundings
    // Highlights the buttons in red color when pressed
    stroke(255,0,0);
    strokeWeight(2);
    noFill();
    rect(240, 270, 100, 100);
    if (press != 1) {
      myPort.write('8');
      messages[numOfMes] = "Alert On";
      numOfMes++;
    }
    press = 1;
  }
  
  if(mousePressed && mouseX>20 && mouseX<120 && mouseY>20 && mouseY<120){ // collects red objects
    // Highlights the buttons in red color when pressed
    stroke(255,0,0);
    strokeWeight(2);
    noFill();
    rect(20, 20, 100, 100);
    if (press != 1) {
      myPort.write(4);
    }
    press = 1;
    myPort.write(0);
  }
  
  if(mousePressed && mouseX>130 && mouseX<230 && mouseY>20 && mouseY<120){ // collects green objects
    // Highlights the buttons in red color when pressed
    stroke(255,0,0);
    strokeWeight(2);
    noFill();
    rect(130, 20, 100, 100);
    if (press != 1) {
      myPort.write(5);
    }
    press = 1;
    myPort.write(0);
  }
  
  if(mousePressed && mouseX>240 && mouseX<340 && mouseY>20 && mouseY<120){ // collects yellow objects
    // Highlights the buttons in red color when pressed
    stroke(255,0,0);
    strokeWeight(2);
    noFill();
    rect(240, 20, 100, 100);
    if (press != 1) {
      myPort.write(6);
    }
    press = 1;
    myPort.write(0);
  }
  
  if(mousePressed && mouseX>350 && mouseX<450 && mouseY>20 && mouseY<120){ // collects all objects
    // Highlights the buttons in red color when pressed
    stroke(255,0,0);
    strokeWeight(2);
    noFill();
    rect(350, 20, 100, 100);
    if (press != 1) {
      myPort.write(8);
    }
    press = 1;
    myPort.write(0);
  }
  
  if(mousePressed && mouseX>20 && mouseX<120 && mouseY>130 && mouseY<205){ // maps routes
    // Highlights the buttons in red color when pressed
    stroke(255,0,0);
    strokeWeight(2);
    noFill();
    rect(20, 130, 100, 75);
    green = 100;
    red = 100;
    yellow = 100;
    all = 100;
    if (press != 1) {
      myPort.write('5');
    }
    press = 1;
    myPort.write(0);
  }
  
  if(mousePressed && mouseX>130 && mouseX<230 && mouseY>130 && mouseY<205){ // back to base
    // Highlights the buttons in red color when pressed
    stroke(255,0,0);
    strokeWeight(2);
    noFill();
    rect(130, 130, 100, 75);
    if (press != 1) {
      myPort.write(9);
    }
    press = 1;
  }
  
  if(mousePressed && mouseX>240 && mouseX<340 && mouseY>130 && mouseY<205){ // stops buggy
    // Highlights the buttons in red color when pressed
    myPort.write('4');
    stroke(255,0,0);
    strokeWeight(2);
    noFill();
    rect(240, 130, 100, 75);
    if (press != 1) {
      messages[numOfMes] = "Force Stop";
      numOfMes++;
    }
    press = 1;
  }
  
  if (!mousePressed) {
    press = 0;
    if (manual) {
      myPort.write('4');
      manual = false;
    }
  }
  
}

#include <AccelStepper.h>
#include <Bounce.h>
#include <Servo.h>

int led = 13;
//int home_led = 8;
int speed = 0;
int demo = 0;

// Define R-Axis Pins
int raxis_step = 7;    //Step pin, r-axis stepper motor drive
int raxis_dir = 8;     //Direction pin, r-axis stepper motor drive
AccelStepper raxismotor(1,raxis_step,raxis_dir);   //Define r-axis stepper motor instance
int rpos = 0;
int rvel = 0;
int racc = 0;

int drpos=7000;

// Define X-Axis Pins
int xaxis_step = 10;    //Step pin, x-axis stepper motor drive
int xaxis_dir = 11;     //Direction pin, x-axis stepper motor drive
AccelStepper xaxismotor(1,xaxis_step,xaxis_dir);   //Define x-axis stepper motor instance
int xpos = 0;
int xvel = 0;
int xacc = 0;

int dxpos = 5000;

// Define Whipped cream pins
int wc_step = 2;    //Step pin, whipped cream stepper motor drive
int wc_dir = 1;     //Direction pin, whipped cream stepper motor drive
int hed = 0;        //HED output, whipped cream stepper motor home position sensor

AccelStepper wcmotor(1,wc_step,wc_dir);   //Define whipped cream stepper motor instance
#define MAX_SERIAL_BUFFER_SIZE 64
uint8_t sindex = 0; // Define index as a global variable
static char buffer[MAX_SERIAL_BUFFER_SIZE];
int homing_active = 0;
long wcpos = 0;
int home_state = 0;
int disp_state = 0;
int hed_state = 0;
int hp = 0;
int hn = 0;
int h0 = 0;
int gap = 0;
long dt0 = 0;    //whipped cream dispense time 0
int wc_disp_pos = 0;    //whipped cream dispense position
long disp_time = 0;
int wc_dispensing = 0;

//Sprinkles
int sprinklePin = 4;
long sprinkle_time = 0;
int sprinkle_state = 0;
int sprinkle_dc = 0;
int sprinkle_dispensing = 0;
long sp_t0 = 0;


const int wcbuttonPin = 3;
Bounce wcButton = Bounce(wcbuttonPin, 100);  // 10 ms debounce

const int spButtonPin = 5;
Bounce spButton = Bounce(spButtonPin, 100);  // 10 ms debounce

Servo cs_servo;
int cpos = 0;



void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(led,OUTPUT);
  pinMode(hed,INPUT);
  pinMode(sprinklePin,OUTPUT);
  //pinMode(home_led,OUTPUT);
  pinMode(wcbuttonPin,INPUT_PULLUP);
  pinMode(spButtonPin,INPUT_PULLUP);

  

  digitalWrite(led,HIGH);
  analogWrite(sprinklePin,LOW);
  
  wcmotor.setMaxSpeed(2000);
  wcmotor.setAcceleration(4000);
  wcmotor.setSpeed(0);

  pinMode(raxis_step,OUTPUT);
  pinMode(raxis_dir,OUTPUT);
  raxismotor.setMaxSpeed(2500);
  raxismotor.setAcceleration(7000);
  raxismotor.setSpeed(0);
  raxismotor.moveTo(0);
  

  pinMode(xaxis_step,OUTPUT);
  pinMode(xaxis_dir,OUTPUT);
  xaxismotor.setMaxSpeed(4000);
  xaxismotor.setAcceleration(5000);
  xaxismotor.setSpeed(0);
  xaxismotor.moveTo(0);
  xaxismotor.setMinPulseWidth(25);

  Serial.println("Starting");
  home_state = 0;
  //digitalWrite(home_led,HIGH);

  cs_servo.attach(6,500,2500);
  cpos=90;
  cs_servo.write(cpos);

  

}

void loop() {

  //wcmotor.runSpeed();
  // if(!digitalRead(hed)){
  //   digitalWrite(home_led,HIGH);
  // } else{
  //   digitalWrite(home_led,LOW);
  // }

  if(homing_active){
    wc_homing();
  }

  if(raxismotor.distanceToGo() !=0){
    raxismotor.run();
  } else {
    if(demo){
      drpos = -drpos;
      raxismotor.moveTo(drpos);
    }
  }

  if(xaxismotor.distanceToGo() !=0){
    xaxismotor.run();
  } else {
    if(demo){
      dxpos = -dxpos;
      xaxismotor.moveTo(dxpos);
    }
  }



  

  //If wc button is pressed
  if(wcButton.update()){
    if(wcButton.fallingEdge()){
      wc_dispensing = 1;
      wc_disp_pos = 465;
      Serial.print("Dispense position: ");
      Serial.println(wc_disp_pos);
      
      disp_time = int(analogRead(A0)/1023.0*1900.0 + 100.0);
      Serial.print("Dispense time: ");
      Serial.println(disp_time);

      Serial.print("Start whipped cream");
    } else {
      //do nothing
    }
  }

  if(wc_dispensing){
    wc_dispense();
  }

  //If wc button is pressed
  if(spButton.update()){
    if(spButton.fallingEdge()){
      sprinkle_dispensing = 1;
      sprinkle_dc = 85;
            
      sprinkle_time = int(analogRead(A1)/1023.0*1900.0 + 100.0);
      Serial.print("Dispense time: ");
      Serial.println(sprinkle_time);

      Serial.print("Start sprinkling");
    } else {
      //do nothing
    }
  }

  if(sprinkle_dispensing){
    sprinkle_dispense();
  }

  if (Serial.available() > 0) { // Check if data is available to read
    char incomingChar = Serial.read();    
    
    if (incomingChar == '\r') { // Check for termination character
      buffer[sindex] = '\0'; // Null-terminate the string
      // Process the received string here
      Serial.print("Received: ");
      Serial.println(buffer);
      processCommand(buffer);      
      sindex = 0; // Reset index for next message      
    } else {
      if (sindex < MAX_SERIAL_BUFFER_SIZE - 1) { // Ensure buffer overflow doesn't occur
        buffer[sindex++] = incomingChar;
      }
    }
  }


    

}

void processCommand(char* message) {

  
  if (strncmp(message, "on", 2) == 0){
    Serial.println("Turning LED ON");
    analogWrite(led,255);
  }

  if (strncmp(message, "off", 3) == 0){
    Serial.println("Turning LED OFF");
    analogWrite(led,0);
  }

  if (strncmp(message, "home", 4) == 0) {
    homing_active = 1;
    home_state = 0;
  }

  if (strncmp(message, "speed=", 6) == 0) {
    int speedValue = atoi(message + 6);
    wcmotor.setSpeed(speedValue);
    Serial.print("Speed set to: ");
    Serial.println(speedValue);
  }

  if (strncmp(message, "pos?", 4) == 0) {
    Serial.print("Current position: ");
    Serial.println(wcmotor.currentPosition());
  }

  if (strncmp(message, "dpos=", 5) == 0) {
    wc_disp_pos = atoi(message + 5);
    Serial.print("Dispense position: ");
    Serial.println(wc_disp_pos);
  }

  if (strncmp(message, "dtime=", 6) == 0) {
    disp_time = atoi(message + 6);
    Serial.print("Dispense time: ");
    Serial.println(disp_time);
  }

  if (strncmp(message, "disp", 4) == 0) {
    wc_dispensing = 1;
    Serial.print("Start whipped cream");
  }

  if (strncmp(message, "stime=", 6) == 0) {
    sprinkle_time = atoi(message + 6);
    Serial.print("Sprinkle time: ");
    Serial.println(sprinkle_time);
  }

  if (strncmp(message, "sdc=", 4) == 0) {
    sprinkle_dc = atoi(message + 4);
    Serial.print("Sprinkle duty cycle: ");
    Serial.println(sprinkle_dc);
  }

  if (strncmp(message, "sprinkle", 8) == 0) {
    sprinkle_dispensing = 1;
    Serial.print("Start sprinkles");
  }

  if (strncmp(message, "cpos=", 5) == 0) {
    cpos = atoi(message + 5);
    Serial.print("Moving servo to: ");
    Serial.println(cpos);
    cs_servo.write(cpos);
  }

  if (strncmp(message, "rpos=", 5) == 0) {
    rpos = atoi(message + 5);
    Serial.print("Moving r-axis to: ");
    Serial.println(rpos);
    raxismotor.moveTo(rpos);
  }

  if (strncmp(message, "rvel=", 5) == 0) {
    rvel = atoi(message + 5);
    Serial.print("R-Axis Max Speed: ");
    Serial.println(rvel);
    raxismotor.setMaxSpeed(rvel);
  }

  if (strncmp(message, "racc=", 5) == 0) {
    racc = atoi(message + 5);
    Serial.print("R-Axis Max Acceleration: ");
    Serial.println(racc);
    raxismotor.setAcceleration(racc);
  }

  if (strncmp(message, "xpos=", 5) == 0) {
    xpos = atoi(message + 5);
    Serial.print("Moving X-axis to: ");
    Serial.println(xpos);
    xaxismotor.moveTo(xpos);
  }

  if (strncmp(message, "xvel=", 5) == 0) {
    xvel = atoi(message + 5);
    Serial.print("X-Axis Max Speed: ");
    Serial.println(xvel);
    xaxismotor.setMaxSpeed(xvel);
  }

  if (strncmp(message, "xacc=", 5) == 0) {
    xacc = atoi(message + 5);
    Serial.print("X-Axis Max Acceleration: ");
    Serial.println(xacc);
    xaxismotor.setAcceleration(xacc);
  }

  if (strncmp(message, "demo=", 5) == 0) {
    demo = atoi(message + 5);
    if(demo){
      Serial.print("Demo Mode");
      raxismotor.moveTo(drpos);
      xaxismotor.moveTo(dxpos);
    }else{
      raxismotor.moveTo(0);
      xaxismotor.moveTo(0);
      drpos = 7000;
      dxpos= 5000;
    }
  }

}

void wc_homing() {
  // Perform homing routine here
  switch (home_state){
    case 0:
    //Initial homing state
    Serial.println("Homing routine initiated...");
    hed_state = digitalRead(hed);     //Capture initial HED state
    wcmotor.move(500);                //Command motor to move 500 steps
    home_state = 1;                   //Proceed to next state
    wcmotor.setMaxSpeed(500);         //Set maximum speed for homing
    hn = 0;
    hp = 0;
    h0 = 0;
    break;

    case 1:
    //Positive rotation, check for change in HED state before rotating
    if(digitalRead(hed) != hed_state){
      //HED has changed state, update hed_state
      hed_state = digitalRead(hed);
      if(hed_state){
        //Magnet has moved off of HED, change direction
        wcmotor.stop();
        wcmotor.move(-500);
        home_state = 2;
      } else {
        //Magnet has move over HED
        wcmotor.stop();
        hn = wcmotor.currentPosition();
        Serial.print("Negative edge: ");
        Serial.println(hn);
        if(hp == 0){
          wcmotor.move(500);
          home_state = 1;
        } else {
          //Both positive and negative edges have been found
          h0 = hn + (hp-hn)/2;
          wcmotor.moveTo(h0);
          home_state = 3;
        }        
      }
    }
    wcmotor.run();
    break;

    case 2:
    //Negative rotation, check for change in HED state before rotating
    if(digitalRead(hed) != hed_state){
      hed_state = digitalRead(hed);
      if(!hed_state){
        //Magnet has moved over HED, change direction
        wcmotor.stop();
        hp = wcmotor.currentPosition();
        Serial.print("Positive edge: ");
        Serial.println(hp);
        if(hn == 0){
          wcmotor.move(-500);
          home_state = 2;
        } else {
          //Both positive and negative edges have been found
          h0 = hn + (hp-hn)/2;
          wcmotor.moveTo(h0);
          home_state = 3;
        }        
      } else {
        //Magnet has moved off HED, change direction
        wcmotor.stop();
        wcmotor.move(500);
        home_state = 1;
      }
    }
    wcmotor.run();
    break;

    case 3:
    if(wcmotor.distanceToGo() == 0){
      //Set the current position to home
      wcmotor.setCurrentPosition(0);
      gap = hp-hn;
      Serial.print("Gap: ");
      Serial.println(gap);
      Serial.println("Homing complete");
      homing_active = 0;
      home_state = 4;
    } else{
      wcmotor.run();
    }
    break;
  }
}

void wc_dispense() {

  switch(disp_state){
    case 0:
    //Initialize dispense
    wcmotor.setMaxSpeed(2000);
    wcmotor.moveTo(wc_disp_pos);
    disp_state = 1;
    wcmotor.run();
    Serial.println("Moving to dispense position");
    break;

    case 1:
    //Move to dispense location
    if(wcmotor.distanceToGo() == 0){
      dt0 = millis();
      disp_state = 2;
      Serial.println("At dispense position");
    } else {
      wcmotor.run();
    }
    break;

    case 2:
    //Wait for dispense to complete
    if(millis() - dt0 < disp_time){
      //Hold position and wait for dispense to complete
      //Serial.println("Dispensing");
    } else {
      wcmotor.moveTo(0);
      disp_state = 3;
      wcmotor.run();
      Serial.println("Moving to home position");
    }
    break;

    case 3:
    if(wcmotor.distanceToGo() == 0){
      Serial.println("Dispense complete");
      wc_dispensing = 0;
      disp_state = 0;
    } else {
      wcmotor.run();
    }
    break;



  }

}

void sprinkle_dispense() {
  
  switch(sprinkle_state){

    case 0:
    Serial.println("Start sprinkles");
    //sprinkle_dc = 85;
    sp_t0 = millis();
    sprinkle_state = 1;
    analogWrite(sprinklePin,sprinkle_dc);
    break;

    case 1:
    if(millis() - sp_t0 < sprinkle_time){
      //do nothing
      Serial.println("Sprinkling!");
    } else {
      analogWrite(sprinklePin,0);
      Serial.println("Done sprinkling");
      sprinkle_state = 0;
      sprinkle_dispensing = 0;
    }    
    break;


  }


}

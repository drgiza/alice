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

int drpos=1600;

// Define X-Axis Pins
int xaxis_step = 10;    //Step pin, x-axis stepper motor drive
int xaxis_dir = 11;     //Direction pin, x-axis stepper motor drive
AccelStepper xaxismotor(1,xaxis_step,xaxis_dir);   //Define x-axis stepper motor instance
int xpos = 0;
int xvel = 0;
int xacc = 0;
int m0 = 16;
int m1 = 17;
int m2 = 18;

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
int wc_disp_pos = 565;    //whipped cream dispense position
long disp_time = 0;
int wc_dispensing = 0;
int wc0 = 4663;         //Whipped cream 0 position
int wc_max = 3964;      //Position for maximum whipped cream
int wc_start = 3964;    //Starting position for maximum whipped cream
int wc_qty = 0;
int wc_xpos = 0;
int wc_xvel = 0;
int wc_rpos = 0;
int wc_rvel = 0;
float wc_time = 0;

//Sprinkles
int sprinklePin = 4;
long sprinkle_time = 0;
int sprinkle_state = 0;
int sprinkle_dc = 0;
int sprinkle_dispensing = 0;
long sp_t0 = 0;

int dorun = 0;    //State machine state for main controller
int mc_active = 0; //Main controller active status



const int wcbuttonPin = 3;
Bounce wcButton = Bounce(wcbuttonPin, 100);  // 10 ms debounce

const int spButtonPin = 5;
Bounce spButton = Bounce(spButtonPin, 100);  // 10 ms debounce

Servo cs_servo;
int cpos = 0;
int cs_state = 0;
int cs_dispensing = 0;

//int pot_cur = 0;
int pot_prev = 0;



void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(led,OUTPUT);
  pinMode(hed,INPUT);
  pinMode(sprinklePin,OUTPUT);
  //pinMode(home_led,OUTPUT);
  pinMode(wcbuttonPin,INPUT_PULLUP);
  pinMode(spButtonPin,INPUT_PULLUP);

  pinMode(m0,OUTPUT);
  pinMode(m1,OUTPUT);
  pinMode(m2,OUTPUT);
  digitalWrite(m0,LOW);
  digitalWrite(m1,LOW);
  digitalWrite(m2,LOW);

  

  digitalWrite(led,HIGH);
  analogWrite(sprinklePin,LOW);
  
  wcmotor.setMaxSpeed(2000);
  wcmotor.setAcceleration(4000);
  wcmotor.setSpeed(0);
  wcmotor.setMinPulseWidth(25);

  pinMode(raxis_step,OUTPUT);
  pinMode(raxis_dir,OUTPUT);
  raxismotor.setMaxSpeed(800);
  raxismotor.setAcceleration(2500);
  raxismotor.setSpeed(0);
  raxismotor.moveTo(0);
  

  pinMode(xaxis_step,OUTPUT);
  pinMode(xaxis_dir,OUTPUT);
  xaxismotor.setMaxSpeed(3000);
  xaxismotor.setAcceleration(10000);
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

  if(millis()-pot_prev > 100){
    Serial.print(analogRead(A0));
    Serial.print(',');
    Serial.print(analogRead(A1));
    Serial.print(',');
    Serial.print(analogRead(A5));
    Serial.print(',');
    Serial.println(analogRead(A6));
    pot_prev = millis();

  }
  if(mc_active){
    main_controller();
  }

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
      // wc_dispensing = 1;
      // wc_disp_pos = 465;
      // Serial.print("Dispense position: ");
      // Serial.println(wc_disp_pos);
      
      // disp_time = int(analogRead(A0)/1023.0*1900.0 + 100.0);
      // Serial.print("Dispense time: ");
      // Serial.println(disp_time);

      // Serial.print("Start whipped cream");
      mc_active=1;
    } else {
      //do nothing
    }
  }

  if(wc_dispensing){
    wc_dispense();
  }

  if(cs_dispensing){
    cs_dispense();
  }

  

  //If sprinkle button is pressed
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
      drpos = 1600;
      dxpos= 5000;
    }
  }

  if (strncmp(message, "mc=", 3) == 0) {
    mc_active = atoi(message + 3);
    Serial.print("Main Controller Status");
    Serial.println(mc_active);
  }

}

void wc_homing() {
  // Perform homing routine here
  switch (home_state){
    case 0:
    //Initial homing state
    Serial.println("Homing routine initiated...");
    hed_state = digitalRead(hed);     //Capture initial HED state
    wcmotor.move(500);                //Command motor to move 500 steps for 1/16th steps
    //wcmotor.move(1000);                //Command motor to move 1000 steps for 1/32 steps
    home_state = 1;                   //Proceed to next state
    //wcmotor.setMaxSpeed(500);         //Set maximum speed for homing for 1/16th steps
    wcmotor.setMaxSpeed(4000);         //Set maximum speed for homing for 1/32 steps
    //wcmotor.setSpeed(4000);
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
        //wcmotor.move(-1000);
        home_state = 2;
      } else {
        //Magnet has move over HED
        wcmotor.stop();
        hn = wcmotor.currentPosition();
        Serial.print("Negative edge: ");
        Serial.println(hn);
        if(hp == 0){
          wcmotor.move(500);
          //wcmotor.move(1000);
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
    //wcmotor.runSpeedToPosition();
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
    Serial.println(wc_disp_pos);
    break;

    case 1:
    //Move to dispense location
    if(wcmotor.distanceToGo() == 0){
      dt0 = millis();
      disp_state = 2;
      Serial.println("At dispense position");

      //Change to 1/8 step mode
      digitalWrite(m0,HIGH);
      digitalWrite(m1,HIGH);
      digitalWrite(m2,LOW);
      //Begin moving x and r axes
      //xaxismotor.setMaxSpeed(238);
      wc_rvel = 900;
      raxismotor.setMaxSpeed(wc_rvel);
      wc_rpos = int(2400.0*float(wc_qty)/1023.0);
      raxismotor.move(wc_rpos);

      wc_time = float(wc_rpos)/float(wc_rvel);

      wc_xpos = (wc0-wc_start)*8;
      xaxismotor.move(wc_xpos);
      wc_xvel = float(wc_xpos)/wc_time;
      xaxismotor.setMaxSpeed(wc_xvel);
      //xaxismotor.moveTo(4663);     

      xaxismotor.run();
      raxismotor.run();
    } else {
      wcmotor.run();
    }
    break;

    case 2:
    //Wait for dispense to complete
    if(xaxismotor.distanceToGo() == 0 && raxismotor.distanceToGo()){
        wcmotor.moveTo(0);
        disp_state = 3;
        wcmotor.run();
        Serial.println("Moving to home position");
    } else {
      xaxismotor.run();
      raxismotor.run();
    }
    break;

    case 3:
    if(wcmotor.distanceToGo() == 0){
      Serial.println("Dispense complete");
      wc_dispensing = 0;
      disp_state = 0;

      //Go back to full step mode
      digitalWrite(m0,LOW);
      digitalWrite(m1,LOW);
      digitalWrite(m2,LOW);
      //Reset current position for full step mode
      xaxismotor.setCurrentPosition(4663);

      xaxismotor.setMaxSpeed(3000);
      raxismotor.setMaxSpeed(800);
    } else {
      wcmotor.run();
    }
    break;



  }

}

void cs_dispense() {

  switch(cs_state){
    case 0:
    //Initialize dispense
    cpos = 132;
    cs_servo.write(cpos);
    cs_state = 1;
    dt0 = millis();
    Serial.println("Chocolate Syrup Dispensing");
    break;

    case 1:
    //Move to dispense location
    if(millis() - dt0 > 3000){
      cs_state = 2;
      
      //Begin moving x and r axes
      //xaxismotor.setMaxSpeed(238);
      //xaxismotor.moveTo(9325);
      //raxismotor.setMaxSpeed(600);
      //raxismotor.move(2400);

      //Change to 1/8 step mode
      digitalWrite(m0,HIGH);
      digitalWrite(m1,HIGH);
      digitalWrite(m2,LOW);
      //Begin moving x and r axes
      //xaxismotor.setMaxSpeed(238);
      xaxismotor.setMaxSpeed(1904);
      //xaxismotor.moveTo(4663);
      xaxismotor.move(7624);
      raxismotor.setMaxSpeed(600);
      raxismotor.move(2400);


      xaxismotor.run();
      raxismotor.run();
    } else {
      //wait for chocolate syrup to start dipping
    }
    break;

    case 2:
    //Wait for dispense to complete
    if(xaxismotor.distanceToGo() == 0 && raxismotor.distanceToGo()){
        cpos=90;
        cs_servo.write(cpos);
        cs_state = 3;
        Serial.println("Waiting for chocolate to stop dripping");
        dt0 = millis();
    } else {
      xaxismotor.run();
      raxismotor.run();
    }
    break;

    case 3:
    if(millis() - dt0 > 5000){
      Serial.println("Chocolate syrup complete");
      cs_dispensing = 0;
      cs_state = 0;

      //Go back to full step mode
      digitalWrite(m0,LOW);
      digitalWrite(m1,LOW);
      digitalWrite(m2,LOW);
      //Reset current position for full step mode
      xaxismotor.setCurrentPosition(9325);

      xaxismotor.setMaxSpeed(3000);
      raxismotor.setMaxSpeed(800);
    } else {
      //wait
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
    sprinkle_dc = 103;
    sprinkle_time = int(analogRead(A5)/1023.0 * 3000);
    analogWrite(sprinklePin,sprinkle_dc);
    //raxismotor.setSpeed(400.0);
    raxismotor.setMaxSpeed(400.0);
    raxismotor.move(float(sprinkle_time)/1000.0*400.0);
    break;

    case 1:
    if(millis() - sp_t0 < sprinkle_time){
      raxismotor.run();
      //Serial.println("Sprinkling!");
    } else {
      analogWrite(sprinklePin,0);
      if(millis() - sp_t0 > sprinkle_time + 1500){        
        Serial.println("Done sprinkling");
        sprinkle_state = 0;
        sprinkle_dispensing = 0;
        xaxismotor.setMaxSpeed(3000);
        raxismotor.setMaxSpeed(800);
      }
    }    
    break;


  }


}

void main_controller() {
  
  switch(dorun){

    case 0:
    Serial.println("Start sundae!");
    //Read all potentiometers and determine how much of each topping to dispense

    //Move to whipped cream center + additional distance depending on amount of whipped cream
    //If No whipped cream is selected, skip and go to chocolate syrup
    wc_qty = analogRead(A0);
      if(wc_qty < 20){
        xaxismotor.moveTo(8373);
        dorun = 3;
      } else{
        //Determine whipped cream start position
        wc_start = wc_max + ((wc0-wc_max) - float(wc_qty)/1023.0*float(wc0-wc_max));
        xaxismotor.moveTo(wc_start);
        Serial.println(wc_start);
      }
    

    //Move on to next state
    dorun = 1;
    break;

    case 1:
    if(xaxismotor.distanceToGo() == 0){
      //Begin dispensing whipped cream
      wc_dispensing = 1;

      //Move on to the next state
      dorun = 2;

    } else {
      xaxismotor.run();
      //If No whipped cream is selected, skip and go to chocolate syrup
      if(wc_qty < 20){
        xaxismotor.moveTo(8373);
        xaxismotor.setMaxSpeed(3000);
        //Go back to full step mode
        digitalWrite(m0,LOW);
        digitalWrite(m1,LOW);
        digitalWrite(m2,LOW);
        dorun = 3;
      }
    }
    break;

    case 2:
    if(wc_dispensing == 1){
      //Do nothing, wait for whipped cream to finish dispensing
      
    } else {
      //Move to chocolate syrup position
      xaxismotor.moveTo(8373);
      //Move on to the next state
      dorun = 3;
    }
    break;

    case 3:
    if(xaxismotor.distanceToGo() == 0){
      cs_dispensing = 1; //Make controller inactive
      dorun = 4;    //Reset main controller state machine
      
    } else {
      xaxismotor.run();
      //If No chocolate syrup is selected, skip and sprinkles
      if(analogRead(A1) < 20){
        xaxismotor.moveTo(13988);
        xaxismotor.setMaxSpeed(3000);
        //Go back to full step mode
        digitalWrite(m0,LOW);
        digitalWrite(m1,LOW);
        digitalWrite(m2,LOW);
        dorun = 5;
      }
    }
    break;

    case 4:
    if(cs_dispensing == 1){
      //Do nothing, wait for chocolate syrup to finish dispensing
      
    } else {
      //Move to sprinkle position
      xaxismotor.moveTo(13988);
      //Move on to the next state
      dorun = 5;
    }
    break;

    case 5:
    if(xaxismotor.distanceToGo() == 0){
      sprinkle_dispensing = 1;
      dorun = 6;
      
    } else {
      xaxismotor.run();
      //If no sprinkles is selected, skip and go home
      if(analogRead(A5) < 20){
        xaxismotor.moveTo(0);
        xaxismotor.setMaxSpeed(3000);
        digitalWrite(m0,LOW);
        digitalWrite(m1,LOW);
        digitalWrite(m2,LOW);
        dorun = 7;
      }
    }
    break;

    case 6:
    if(sprinkle_dispensing == 1){
      //Do nothing, wait for sprinkles to finish dispensing
      
    } else {
      //Move to home position
      xaxismotor.moveTo(0);
      //Move on to the next state
      dorun = 7;
    }
    break;

    case 7:
    if(xaxismotor.distanceToGo() == 0){
      mc_active = 0; //Make controller inactive
      dorun = 0;    //Reset main controller state machine
      
    } else {
      xaxismotor.run();
    }
    break;


  }
}

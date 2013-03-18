/*

 seaGlider with stall detection
 
 Michael Britt-Crane NSWCCD code 5600
 2013.03.17
 
 put description here etc.
 
 features to add:
 0. Get rid of Servo.h drive servo with my own method(s). More flexability, cleaner.
 5. Clean some stuff up. Clean up check stall, strip out commented code etc. Could eliminate some global variables: stallValue, pot controlled var.
 6. Further generalize and consolodate methods: dive & rise, readPot.
 7. Fully impliment IR reciever so that you can control the glider with an IR remote control.
 8. EVEN MORE COMMENTS!
 
 DONE, actually replaced by an RGB LED  1. blinking during "pause"
 once a sec until last ~sec then every 300 til 300 left then solid for 200 off for 100
 REPLACED BY RGB LED 4. use "millis()" for blinking on dive instead of "delay()"
 DONE, this actually replaced limmit switches 2. add crash / stall protection:
 *revise*    - check limit switch state at startup, drive away, re-check, drive away, re-check. if still on error loop
 - current monitor: read voltage across v-small resistor in series with motor to detect stall
 use in conjunction with limit spring(s) instead of limit switches
 CAN'T BE DONE, NO POT IN SERVO 3. Feedback from servo pot, use it as an encoder
 
*/

// libraries:
  #include <Servo.h> 
  //#include <IRremote.h>           // not in use. used to read codes from IR reciever which recieves signals from an IR TV remote

// object declarations
  Servo myservo;                    // create servo object to control the continuous rotation servo that drives the lead-screw 

// Constants:
  // Pins
    const byte ledPin = 13;           // this is attached to an LED on the arduino board, used for debug
    const byte servoPin = 17;         // A3 (d17)
    const byte potPin = 1;            // A1 (d15) center pin on pot, acting as variable volgage devider 
    const byte motorLoadPin = 2;      // A2 (d16) 
    //const byte diveStopPin = 11;    // not in use. Could be used for dive-end end stop
    //const byte riseStopPin = 12;    // not in use. Could be used for rise-end end stop
    //const byte STAY_ON_PIN =8;      // not in use. Could be used for a latching remote shutoff circuit
  // RGB LED pins 
    const byte RED_LED = 9;
    const byte GREEN_LED = 6;
    const byte BLUE_LED = 5;
    const byte LED_BASE = 7;          // this is the comon pin to the 3 LEDs for this LED it is a common anode
  // Other Constants
    int stallThreshold = 22;  // <<<<<---------------- set the stall sensitivity       
    int motorStartTime = 350;         // runs motor for short time before stall detection starts. Avoids detecting startup current surge as stall. Bypasses stickton
    int pauseTime = 1200;             // 1.2 sec
    //int minPause =  1000;           // not in use. This is for if the pause time were controlled by the pot
    //int maxPause = 15000;           // not in use. ""
    int minDiveTime =  700;           // 700 is a good min ~20cc on 1/4-10 lag bolt 6v
                                      // 800 is a good min ~20cc on 5/16-18 7v
    int maxDiveTime = 15000;          // 1500 is a good max ~40cc on 1/4-10 lag bolt 6v 
                                      // 1600 is a good max ~42cc on 5/16-18 7v
                              // 11 sec = ~ 30ML on 4AAA W/ 1/4-10 Lag bolt
                              // 13 sec = ~ 35ML at 7.5v W/ 5/16-18 machine screw
                              // 11 sec = ~ 30ML at 6v W/ 300 rpm servo driving 1/4-20 screw
    
    byte servoDiveCommand = 0;        // this is the angle value that the dive method sends to the servo
    byte servoRiseCommand = 180;      // this is the angle value that the rise method sends to the servo

// Global Variables
  //long diveDriveTime  = 13350;    //  <<<<<----------------------- set dive drive time. NOT IN USE V-see below-V
  long diveDriveTime;               // This value is determined by the position of the pot and the min & maxDiveTime vars. it is assigned in the readPot mthod
  int potValue = 0;                 // 10-bit analog reading of voltage on pot pin (0-1023)
  int stallValue = 0;               // the variable that stores the read stall value
  long previousMillis = 0;          // this variable is used for the diveDriveTime timer


// -------------------------- setup ---------------------------
void setup() {
  // debug settup:  
  Serial.begin(9600);                 // start the serial port
  pinMode(ledPin, OUTPUT);            // the LED on the arduino board used for debuting

  pinMode(potPin, INPUT);             // set pot pin used to adjust diveDriveTime 
  pinMode(motorLoadPin, INPUT);       // set the stall detection pin as analog input

  // RGB LED setup      
  pinMode(RED_LED, OUTPUT);     
  pinMode(GREEN_LED, OUTPUT);     
  pinMode(BLUE_LED, OUTPUT);     
  pinMode(LED_BASE, OUTPUT);     
  digitalWrite(LED_BASE, HIGH);       // this is the comon pin to the 3 LEDs for this LED it is a common anode, providing +5v to all 3 LEDs

  displayPot();                       // Reads the pot and blinks the RGB LED to indicate diveDriveTime setting. Blinks secconds followed by 10ths
  delay(400);                         //
}

// --------------------------- main ---------------------------
void loop(){
  ledRGB_Write(255, 0, 0);     // set LED to RED to indicate that the glider is diving
  dive();                      // DIVE! DIVE! DIIVE!!  (run the "dive" method)
  readPot();
  //Serial.println("120");     // NOT IN USE, a debugg marker for excell
  ledRGB_Write(255, 80, 0);    // set LED to ORANGE to indicate that the glider is coasting in a dive
  delay(pauseTime);            // pause for pauseTime, coast

  ledRGB_Write(0, 200, 0);     // set LED to GREEN to indicate that the glider is rising
  rise();                      // Rise  (Run the "rise" method)
  readPot();
  ledRGB_Write(0, 0, 255);     // set LED to BLUE to indicate that the glider is coasting in a rise
  //Serial.println("120");     // NOT IN USE, a debugg marker for excell
  delay(pauseTime);            // pause for pauseTime, coast

  //delay(3000);               // NOT IN USE, used for remote shut down testing
  //digitalWrite(STAY_ON_PIN, LOW); // NOT IN USE, used for remote shut down testing
}


// -------------------------- methods -------------------------

void ledRGB_Write(int R, int G, int B){
  analogWrite(RED_LED, 255-R);                  // These are backwards because you write low values to turn these LEDs on
  analogWrite(GREEN_LED, 255-G);                // This method reverses the counterintuitive nature of the LEDs
  analogWrite(BLUE_LED, 255-B);                 // If using common cathode rather than common anode LEDs remove the "255-"es
}

int readPot(){                                  // this method reads a potentiometer to determine the pause time 
  byte samples = 10;
  int potValues = 0;
  for(int i=0; i<=samples; i++){                // sum a number of readings with 10ms inbetween them
    potValues += analogRead(potPin);            // reads the value of the potentiometer (value between 0 and 1023) 
    delay(10);
  }
  potValue = potValues / samples;               // take the average value
  return potValue;
  //pauseTime = map(potValue, 0, 1023, minPause, maxPause);     // scale the value to the pause range defined by minPause & maxPause 
  diveDriveTime = map(potValue, 0, 1023, minDiveTime, maxDiveTime); 
  // scale the value to the diveDriveTime range defined by minDriveTime & maxDriveTime
}

boolean checkForPotChange(){                    // This method can be used check whether the pot has moved position.
  int oldPotValue = potValue;                   //   Can be used to display the pot value whenever it is changed or perform whatever other action you like.
  readPot();                                    // read and set the pot value using readPot
  if (potValue/20 != oldPotValue/20){           // check whether new pause time is within 20 of old pause time
    displayPot();                               
  }
}

void displayPot(){
  readPot();                                    // read the potentiometer
  printPot();                                   // print the read value to the serial port
  for (int i=0; i < (diveDriveTime/1000); i++){ // flash 1 long pulse for each seccond of pauseTime
    digitalWrite(ledPin, HIGH);
    ledRGB_Write(250, 80, 0);
    Serial.print(i+1);                          // print flash number to console
    Serial.print(" ");
    delay(90);
    digitalWrite(ledPin, LOW);
    ledRGB_Write(0, 0, 0);
    delay(450);
  }
  Serial.print(" -  ");
  delay(700);
  for (int i=40; i < (diveDriveTime%1000); i=i+100){  // flash 1 short pulse for each 1/10 seccond of pauseTime
    digitalWrite(ledPin, HIGH);
    ledRGB_Write(240, 0, 0);
    Serial.print(i/100+1);                      // print flash number to console
    Serial.print(" ");
    delay(50);
    digitalWrite(ledPin, LOW);
    ledRGB_Write(0, 0, 0);
    delay(100);
  }
  Serial.print("... ");
  Serial.println("and GO!");
}

void printPot(){
  Serial.println();
  Serial.print("analog reading is: ");
  Serial.println(potValue, DEC);                // print the potValue to the console
  Serial.print("the pause time is: ");
  Serial.print(diveDriveTime/1000, DEC);        // print the pauseTime to the console (secconds)
  Serial.print(".");                            
  Serial.print(diveDriveTime%1000/100, DEC);    // print the pauseTime to the console (tenths)
  Serial.print(diveDriveTime%1000%100/10, DEC); // print the pauseTime to the console (hundredths)
  Serial.print(" secconds (");
  Serial.print(diveDriveTime, DEC);             // print the pauseTime to the console (in milli-sec.)
  Serial.println(")");
}

void dive(){ 
  checkStall();
  myservo.attach(servoPin);                     // attaches the servo on servoPin to the servo object 
  myservo.write(servoDiveCommand);              // drive servo clockwise, pull weight forward (pull counterweight & plunger towards servo)
  delay(motorStartTime);
  while (!(stallValue >= stallThreshold) /*&& digitalRead(diveStopPin) == HIGH*/){     // drive servo until riseStop switch is hit
    checkStall();
    //digitalWrite(ledPin, HIGH);               // NOT IN USE flash led fast while driving servo (50ms period)
    //delay(15);
    //digitalWrite(ledPin, LOW);
    //delay(35);
    //checkIR();                                // IR Receiver not in use
    //if (pause == true){
    //  pauseMethod();
    //}
  }
  myservo.detach();                             // detaches the servo on servoPin from the servo object 
}

void rise(){
  checkStall();
  myservo.attach(servoPin);                     // attaches the servo on servoPin to the servo object 
  myservo.write(servoRiseCommand);              // drive servo counter-clockwise, pull weight aft (push counterweight & plunger away from servo)
  digitalWrite(ledPin, HIGH);                   // turn on the Arduino's onboard LED
  unsigned long currentMillis = millis();       // declare currentMilis variable this will be used for the diveDriveTime timer
  previousMillis = currentMillis;               // start the diveDriveTime timer
  delay(motorStartTime);                        // ignore startup current spike
  while (!(stallValue >= stallThreshold) && (currentMillis - previousMillis < diveDriveTime) /*&& digitalRead(riseStopPin) == HIGH*/){     // drive servo until riseStop switch is hit
    checkStall();
    currentMillis = millis();                   // update the diveDriveTime timer
    //checkIR();                                // IR Receiver not in use
    //if (pause == true){
    // pauseMethod();
    //}
  }
  myservo.detach();                             // detaches the servo on servoPin from the servo object 
  digitalWrite(ledPin, LOW);                    // turn LED off
}

boolean checkStall(){                           // returns true if stall is detected, also updates global stallValue variable
  stallValue = analogRead(motorLoadPin);        
  //Serial.print("Motor Load = ");
  //if (stallValue > 0 ){
  Serial.println(stallValue);
  //}
  //Serial.print(" DivePin = ");
  //Serial.println(digitalRead(diveStopPin));
  if (stallValue >= stallThreshold){
    stallValue = analogRead(motorLoadPin);
    Serial.println("99");
    if (stallValue >= stallThreshold){
      //stallValue = analogRead(motorLoadPin);
      //if (stallValue >= stallThreshold){
      stallValue = analogRead(motorLoadPin);
      Serial.println("STALL - STALL - STALL");
      Serial.println("115");
      return true;
      //}
    }
  }
  else{
    return false;
  }
}

// --------------------- methods below this line are not in use --------------------

void pauseMethod(){
  myservo.detach();
  //while (pause){
  //}
  myservo.attach(servoPin);  
  //pause = false;
}

void futurepauseMethod(){
  /*
  myservo.detach();
   exitPause = false;
   while(!exitPause){ 
   ledRGB_Write(0,40,40);
   if (irrecv.decode(&results)) {
   switch (results.value) {
   case 0xFD807F:                    //play pause
   exitPause = true;
   myservo.attach(servoPin);
   break;
   case 0xFD609F:                    //stopMode
   digitalWrite(RED_LED, LOW);
   myservo.detach();
   irrecv.resume(); // Receive the next value
   //        boolean exit = false;
   //        While(exit == false){
   //          if (irrecv.decode(&results)) {
   //            if (results.value == 0xFD609F){
   //              exit = true;
   //            } else if (results.value == 0xFD807F){
   // not in use digitalWrite(STAY_ON_PIN, LOW);                 // turn off
   //            }
   //          }
   //        }
   break;
   
   }// end switch(results...
   }// end if(irrecv...
   }// end while(exitPause == false)
   myservo.attach(servoPin);  
   pause = false;                               */
}

void testPause(){
  ledRGB_Write(0,90,90);
  myservo.detach();
  delay(500);
  ledRGB_Write(0,0,0);
  delay(200);
  myservo.attach(servoPin);  
  //  pause = false;
}

void readIR(){
  /*
  if (irrecv.decode(&results)) {
   pause = !pause;
   }
   delay(20);
   if (irrecv.decode(&results)) {
   pause = true;
   }                                         */
}

void checkIR(){
  /*
  irrecv.resume(); // Receive the next value
   if (irrecv.decode(&results)) {
   switch (results.value) {
   case 0xFD807F:                    //play pause
   digitalWrite(GREEN_LED, LOW);
   myservo.detach();
   irrecv.resume(); // Receive the next value
   exitPause = false;
   while(exitPause == false){
   if (irrecv.decode(&results)) {
   if (results.value == 0xFD807F){
   exitPause = true;
   digitalWrite(GREEN_LED, LOW);
   }
   }
   }
   break;
   case 0xFD609F:                    //stopMode
   digitalWrite(RED_LED, LOW);
   myservo.detach();
   irrecv.resume(); // Receive the next value
   //        boolean exit = false;
   //        While(exit == false){
   //          if (irrecv.decode(&results)) {
   //            if (results.value == 0xFD609F){
   //              exit = true;
   //            } else if (results.value == 0xFD807F){
   // not in use digitalWrite(STAY_ON_PIN, LOW);                 // turn off
   //            }
   //          }
   //        }
   break;
   }
   irrecv.resume(); // Receive the next value
   }                                                              */
}

// I don't think this method is actually useful. But I didn't come to that reaolization until after writing and testing it...
void checkEndstop(int endStop, int servoCommand, int otherEndStop){
  if( !digitalRead(endStop) && digitalRead(otherEndStop) ){  
    myservo.attach(servoPin);                   // attaches the servo on servoPin to the servo object 
    myservo.write(servoCommand);                // drive servo counter-clockwise, pull weight aft (push counterweight & plunger away from servo)
    delay(1500);
    myservo.detach();              // drive servo counter-clockwise, pull weight aft (push counterweight & plunger away from servo)
    delay(500);
    if( !digitalRead(endStop) && digitalRead(otherEndStop)){
      myservo.attach(servoPin);
      myservo.write(servoCommand);              // drive servo counter-clockwise, pull weight aft (push counterweight & plunger away from servo)
      delay(1500);      
      myservo.detach();                         // detaches the servo on servoPin from the servo object 
      while( !digitalRead(endStop)){
        Serial.print(" PROBLEM ");
        digitalWrite(ledPin, HIGH);             // flash led fast while driving servo (50ms period)
        delay(1000);
        digitalWrite(ledPin, LOW);
        delay(2000);
      }
    }
    myservo.detach();                           // detaches the servo on servoPin from the servo object 
    delay(500);
  }
}




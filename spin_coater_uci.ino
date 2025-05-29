#include <LiquidCrystal.h>
#include <Servo.h>
#include <TaskScheduler.h>

/*
Sources for Python CSV integration:

https://www.phippselectronics.com/storing-arduino-sensor-data-in-csv-format-using-python/

https://www.instructables.com/Capture-Data-From-Arduino-to-CSV-File-Using-PySeri/

In order to use ReadSerial.py, you must download python and pip.
After that, depending on your environment you need to source the directory to make a
virtual environment for python scripts to run.
Once that is setup, change the serial port to the one to monitor and run:
 python ReadSerial.py
on your device. 


*/

// Defining LED Display pins 
#define PIN_RS 11 
#define PIN_RW 3
#define PIN_E 4
#define PIN_D4 6
#define PIN_D5 7
#define PIN_D6 8
#define PIN_D7 9

// Defining Button Pins
#define PIN_RPM_UP    17 //A3
#define PIN_RPM_DOWN  18 //A4
#define PIN_TIME_UP   15 //A1
#define PIN_TIME_DOWN 16 //A2
#define PIN_START 14 //A0

// Definign Motor Pins
#define PIN_MOTOR 12

// Defining IR Pins (testing)
#define PIN_IR 0 // aka digital pin 2 on uno

// defining buzzer pin (testing)
#define PIN_BUZZER 13 // aka digital pin 2 on uno

// Motor Spinning Constants 
constexpr int preSpinRPM = 600;
constexpr int rampTime = 1000;
constexpr int rampSteps = 50;
constexpr int maxRPM = 12000;

// IR Sensor Variables
volatile unsigned long rpmcount;
unsigned int measuredRpm;
unsigned long timeold;
unsigned long lastMillis = 0;
volatile unsigned long lastInterruptTime = 0;
constexpr double adj_mtr = 1.42857143; // obtained from calculating RPM based off 12V, 2000 RPM 

Servo servo; // Setup the Servo
auto lcd = LiquidCrystal(PIN_RS, PIN_RW, PIN_E, PIN_D4, PIN_D5, PIN_D6, PIN_D7); // Setup LED Display

// Saves Data for the Spin Phase
struct spinState {
  long long rpm = 3000;
  long long duration = 30; 
};

bool debounce(int);
void setSpin(int, int);
spinState menuLoop();
void preSpin();
void Spin();
void setup();
void loop();
void rpm_fun();
int readRpm();
int mapRPM(int);
void startScreen();
void updateValues();

// Handles the interupts of the IR Sensor. Calculate rotations over some period of time
void rpm_fun()
{
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 2) {  // Debounce pulses
    rpmcount++;
    // Serial.println("ADDED RPM");
  }
  lastInterruptTime = interruptTime;
}



// float Vacross,Iamp = 0;
// const int Shunt_Res = 1000; //for 1k resistor
// Uses the IR Sensor Data to calculate RPM
int readRpm(){
  // lastMillis = millis();
  delay(1000);
  noInterrupts();
  unsigned long count = rpmcount;
  rpmcount = 0;
  unsigned long elapsedTime = millis() - timeold;
  timeold = millis();
  interrupts();
  // if (elapsedTime > 0) {
    measuredRpm = (30 * 1000 * count) / elapsedTime;
    // Serial.println("MEASURED TIME" + String(count) + ", " + String(elapsedTime));
  // } //else {
    // measuredRpm = 0;
    // Serial.println("NOELAPSE");
  //}
  // if(measuredRpm != 0){

  // https://www.hackster.io/manodeep/diy-ammeter-using-arduino-5d44e3
  // Vacross = analogRead(A5);
  // Vacross = (Vacross * 5.0) / 1023.0;
  // Iamp = (Vacross * 1000) / Shunt_Res;
  
  // Serial.print("Current = " + Iamp + " mA");
  return measuredRpm;
  // }
}

// Used the Debounce any of the button presses
bool prev_button_states[5] = { false, false, false, false, false };
bool button_states[5] = { false, false, false, false, false };
bool debounce(int buttonNumber) {
  bool state; 
  switch (buttonNumber) {
    case 0: state = digitalRead(PIN_RPM_UP); break;
    case 1: state = digitalRead(PIN_RPM_DOWN); break;
    case 2: state = digitalRead(PIN_TIME_UP); break;
    case 3: state = digitalRead(PIN_TIME_DOWN); break;
    case 4: state = digitalRead(PIN_START); break;
    default: return false; 
  }
  button_states[buttonNumber] = !state; 

  if (button_states[buttonNumber] != prev_button_states[buttonNumber]) {
    prev_button_states[buttonNumber] = button_states[buttonNumber];  
    if (button_states[buttonNumber]) {  
      delay(50);  
      return true;  
    }
  }
  return false;  
}

// Maps input RPM to a value that is understandable by the Servo
int mapRPM(int x){
  return ceil(map(x, 0, maxRPM, 1500, 1000));
}

// Transitions between the current RPM (curr) and the target RPM (target)
void setSpin(int curr, int target){
  float deltaR = (float) (target - curr) / rampSteps;
  float deltaT = (float) (rampTime) / rampSteps;
  for(int i = 0; i < rampSteps; i++){
    int rpm = (int)(curr + deltaR * i);  
    servo.writeMicroseconds(mapRPM(rpm));
    delay(deltaT);
  }
  servo.writeMicroseconds(mapRPM(target));
}

// Initial Start Screen (Press Start to Continue)
void startScreen(){
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("UCI Spin Coater!");
  lcd.setCursor(0, 1); lcd.print("Press Start..."); 
  while(1){if(debounce(4)){return;}}
}

// Menu Loop (User Selects RPM and Duration)
spinState menuLoop(){
  spinState state;
  bool initial = true;
  lcd.clear();
  while(true){
    bool updated = true;
    if(debounce(1)){ // Increases RPM
      state.rpm += 100;
      Serial.println("RPM Up");
    }
    else if(debounce(0)){ // Decreases RPM
      state.rpm -= 100;
      Serial.println("RPM Down");
    }
    else if (debounce(2)) { // Increases Duration by 1 Second
        Serial.println("Duration Up");
        state.duration += 1;
    }
    else if (debounce(3)) { // Decreases Duration by 1 Second
        Serial.println("Duration Down");
        state.duration -= 1;
    }
    else{
      updated = false;
    }

    if(debounce(4)){ // Start Button: Returns the RPM and Duration data for later use
      return state; 
    }

    // Updates the Display with the Current Values
    if(updated || initial){
      lcd.setCursor(0, 0); lcd.print("RPM: "); lcd.print("    "); lcd.setCursor(5, 0); lcd.print((int)(state.rpm / adj_mtr));
      lcd.setCursor(0, 1); lcd.print("Duration: "); lcd.print("    ");  lcd.setCursor(10, 1); lcd.print((int)state.duration);
      initial = false;
    }
  }
}

// Controls the PreSpin Phase (Add Photoresist here)
void preSpin(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Pre-Spin Phase");
  lcd.setCursor(0,1); lcd.print("RPM: 600");

  setSpin(0, preSpinRPM);
  while(1){
    if (debounce(4)){ // Exit using Start Button
      return;
    }
  }
}

// Controls Spin Phase & Display
void Spin(int rpm, int duration){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Spinning...");
  lcd.setCursor(0, 1); lcd.print("Duration: "); lcd.print((int)duration);
  setSpin(preSpinRPM, rpm);
  int progress = 0;
  int startTime = millis();
  int lastDisplayed = -1;

  while(progress < duration * 1000){
    progress = millis() - startTime; 
    int timeLeft = ceil(duration - progress/1000);
    if(timeLeft != lastDisplayed){
      lcd.setCursor(0, 1); lcd.print("Duration: ");lcd.setCursor(10, 1);  lcd.print("     "); lcd.setCursor(10, 1); lcd.print(timeLeft);
      lastDisplayed = timeLeft;
    }
    if(debounce(4)){break;} // Early Exit with Start Button
    Serial.println(readRpm());
  }
  setSpin(rpm, 0);
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Completed Spin!");
  delay(3000);
}

void setup() {
  // PIN Settings
  Serial.begin(9600);
  pinMode(PIN_RPM_UP, INPUT_PULLUP);
  pinMode(PIN_RPM_DOWN, INPUT_PULLUP);
  pinMode(PIN_TIME_UP, INPUT_PULLUP);
  pinMode(PIN_TIME_DOWN, INPUT_PULLUP);
  pinMode(PIN_START, INPUT_PULLUP);
  servo.attach(PIN_MOTOR, 1000, 2000);

  // buzzer
  pinMode(PIN_BUZZER, OUTPUT);

  // Setup LCD
  lcd.begin(16, 2);
  lcd.clear();

  // Setup IR Sensor
  attachInterrupt(PIN_IR, rpm_fun, FALLING);  // Change to RISING or CHANGE if needed
  rpmcount = 0;
  measuredRpm = 0;
  timeold = millis();
}


int n = 40;
int samples = 10;
int data[40]; // make sure this is equal to n size
void test(){
  Serial.println("IN TESTING MODE, SCREEN DISABLED. COMMENT FXN OUT IF NOT IN USE");
  int step = 100;
  int initial = 0;
  setSpin(0, initial);

  // data[j] is each 30 data points and initial + step * (i+1) is the RPM
  for(int i = 0; i < n; i++){
    int ret = 0;
    Serial.println("RPM_SIG," + String(initial + step * (i+1)));
    setSpin(initial+step * i , initial + step * (i+1) );
    digitalWrite(PIN_BUZZER, HIGH);
    bool buzzed = false;
    for(int j = 0; j < samples; j++ ){
      data[j] = readRpm();
      if (!buzzed){
        buzzed = true;
        digitalWrite(PIN_BUZZER, LOW);
      }
      // add data[j]
      Serial.println("IR_RPM," + String(data[j]));
    }
  }
  Serial.println("All tests done, spinning down");
  for (int i = 0; i < 3; i++){
    setSpin(initial+step * n, 1500);
    delay(1000);
  }
  for (int i = 0; i < 3; i++){
    setSpin(initial+step * n, 1200);
    delay(1000);
  }
  setSpin(1200, 0);
  Serial.println("Finishing up file!");
  Serial.flush();
}


// System Loop
void loop() {

  test(); // These two lines are for testing/graphing
  while(1){}

  startScreen();
  while(1){
    spinState state = menuLoop();
    Serial.println("Finished Menu State");
    preSpin();
    Spin(state.rpm, state.duration); 
  }     
} 


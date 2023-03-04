#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Keypad.h>
#include <Servo.h>

// Setup sensors
/*
DHT22 - pin 13
LDR1 - pin 34
LDR2 - pin 35
LCD
  SDA - pin 21
  SCL - pin 22
Keypad pins:
  row pins: 16,17,18,19
  col pins: 26,27,32,33
fans - pin 4
servo 
    pin 12
    pin 14
available: 5,14,15,25,
*/

DHT HT_Sensor(13, DHT22);  //Temperature, Humidity sensor
int LDR1 = 34;
int LDR2 = 35;
int FAN = 4;
LiquidCrystal_I2C LCD(0x27, 16, 2);  //LCD Display parameters
// Setup keypad
byte rowPins[4] = { 16, 17, 18, 19 };  //Keypad input pins
byte colPins[4] = { 26, 27, 32, 33 };  //Keypad input pins
char keys[4][4] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' },
};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, 4, 4);

Servo SolarServo1;
Servo SolarServo2;

// functions
void LDRinit();
void LCDinit();
void displayTempHumi(int, int);
void getTemperature();
void controlTemperature(int);
void Animate();
void image01();
void image02();
void turnSolarPanel();

// Variables
int maxTemp = 32;

double Kp = 2;
double Kd = 2;
double Ki = 0.1;
double PreError = 0;
double TotError = 0;

bool fram = true;

int Def_Servo11 = 90;
int Def_Servo12 = 90;

void setup() {
  // put your setup code here, to run once:
  HT_Sensor.begin();
  Serial.begin(9600);
  Serial.println("Start program");
  LDRinit();
  LCDinit();
  SolarServo1.attach(12);
  SolarServo2.attach(23);
  Serial.println("Start lcd");
  LCD.print("    Welcome!");

  // setup fans
  pinMode(FAN, OUTPUT);

  delay(3000);
  LCD.clear();
}

void loop() {
  Serial.println("begin of loop");
  // put your main code here, to run repeatedly:
  LCD.setCursor(0, 1);

  // Read temperature, humidity
  int Humidity = HT_Sensor.readHumidity();
  int Temperature = HT_Sensor.readTemperature();
  // Display temperature and humidity
  displayTempHumi(Temperature, Humidity);
  // getTemperature();
  controlTemperature(Temperature);
  delay(2000);
  // Rotate solar panel
  turnSolarPanel();


  delay(500);
}

void LDRinit() {
  pinMode(LDR1, INPUT);
  pinMode(LDR2, INPUT);
}

void LCDinit() {
  LCD.init();
  LCD.backlight();
  LCD.setCursor(0, 0);
}

void displayTempHumi(int temperature, int humidity) {
  LCD.setCursor(0, 0);
  LCD.print("Temperature: ");
  LCD.print(temperature);
  LCD.print("C");
  LCD.setCursor(0, 1);
  LCD.print("Humidity: ");
  LCD.print(humidity);
  LCD.print("%");
}

void getTemperature() {
  char key = keypad.getKey();
  // Wait until press A
  LCD.setCursor(0, 0);
  LCD.print("Press A to setup");
  LCD.setCursor(0, 1);
  LCD.print("Temperature.");

  while (!key) {}
  Serial.print(key);
  if (key == 'A') {
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print("Enter Temperature");
    LCD.setCursor(0, 1);
    LCD.print("here");
  }
  delay(1000);
}

void turnSolarPanel() {
  int LDRLeft = analogRead(LDR1);
  int LDRRight = analogRead(LDR2);
  int diff = (LDRRight >= LDRLeft) ? LDRRight - LDRLeft : LDRLeft - LDRRight;

  Serial.print(LDRLeft);
  Serial.print("      ");
  Serial.println(LDRRight);

  while (diff > 150) {
    Serial.print(LDRLeft);
    Serial.print("      ");
    Serial.println(LDRRight);

    if (LDRLeft > LDRRight) {
      if (Def_Servo1 < 180) {
        Def_Servo1 = Def_Servo1 + 1;
      }
      SolarServo1.write(Def_Servo1);
    }
    if (LDRLeft < LDRRight) {
      if (Def_Servo1 > 0) {
        Def_Servo1 = Def_Servo1 - 1;
      }
      SolarServo1.write(Def_Servo1);
    }
    delay(20);
    LDRLeft = analogRead(LDR1);
    LDRRight = analogRead(LDR2);
    diff = (LDRRight >= LDRLeft) ? LDRRight - LDRLeft : LDRLeft - LDRRight;
  }
}

void controlTemperature(int Temperature) {
  if (Temperature > maxTemp) {
    // turn on fans
    // Animate();
    digitalWrite(FAN, LOW);
  } else {
    digitalWrite(FAN, HIGH);
    //until.
  }
  delay(1000);
}

double PID(int setpoint, int feedback) {
  double error = setpoint - feedback;

  double Pterm = error * Kp;
  double Iterm = TotError * Ki;
  double Dterm = (error - PreError) * Kd;

  double pidVal = Pterm + Iterm + Dterm;

  PreError = error;
  TotError += error;

  return pidVal;
}

void Animate() {
  if (fram) {
    image01();
    fram = false;
  } else {
    image02();
    fram = true;
  }
}
void image01() {
  byte img01[] = { B00000, B01110, B11011, B10101, B11011, B01110, B00000, B00000 };
  LCD.createChar(0, img01);
  LCD.setCursor(15, 0);
  LCD.write(byte(0));
}
void image02() {
  byte img02[] = { B00000, B01110, B10101, B11111, B10101, B01110, B00000, B00000 };
  LCD.createChar(0, img02);
  LCD.setCursor(15, 0);
  LCD.write(byte(0));
}

#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Keypad_I2C.h>
#include <Keypad.h>
#include <Servo.h>

#define I2C_Addr_KEYPAD 0x20
#define I2C_Addr_LCD 0x27

// Setup sensors
/*
DHT22 - pin 13

LDR1 - pin 34
LDR2 - pin 35
LCD & keypad
  SDA - pin 21
  SCL - pin 22
fans - pin 4
servo 
    pin 12
    pin 14
    
*/

DHT HT_Sensor(13, DHT22);  //Temperature, Humidity sensor
int LDR1 = 34;
int LDR2 = 35;
int FAN = 4;


// Setup keypad
const byte NbrRows = 4;     // Number of Rows
const byte NbrColumns = 4;  // Number of Columns

char KeyPadLayout[NbrRows][NbrColumns] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

byte PinsLines[NbrRows] = { 0, 1, 2, 3 };       //  ROWS Pins
byte PinsColumns[NbrColumns] = { 4, 5, 6, 7 };  //  COLUMNS Pins

LiquidCrystal_I2C LCD(I2C_Addr_LCD, 16, 2);  //LCD Display parameters
Keypad_I2C i2cKeypad(makeKeymap(KeyPadLayout), PinsLines, PinsColumns, NbrRows, NbrColumns, I2C_Addr_KEYPAD);

Servo SolarServo1;
Servo SolarServo2;

// functions
void LDRinit();
void LCDinit();
void KeyPadinit();
void displayTempHumi(int, int);
void getTemperature();
void controlTemperature(int);
void Animate();
void image01();
void image02();
void turnSolarPanel();

// Variables
int maxTemp = 40;

double Kp = 2;
double Kd = 2;
double Ki = 0.1;
double PreError = 0;
double TotError = 0;

bool fram = true;

int Def_Servo1 = 90;
int Def_Servo2 = 90;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  // setup fans
  pinMode(FAN, OUTPUT);
  digitalWrite(FAN, LOW);
  
  delay(200);
  // Start DHT22 sensor
  HT_Sensor.begin();
  // Start I2C bus
  Wire.begin(21, 22);
  Serial.println("Start program");
  LDRinit();
  LCDinit();
  KeyPadinit();
  SolarServo1.attach(12);
  SolarServo2.attach(23);
  Serial.println("Start lcd");
  LCD.print("    Welcome!");
  
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

  delay(6000);

  getTemperature();

  displayTempHumi(Temperature, Humidity);
  controlTemperature(Temperature);
  delay(2000);
  // Rotate solar panel
  turnSolarPanel();

  delay(5000);
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

void KeyPadinit() {
  i2cKeypad.begin();  // Start i2cKeypad
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
  unsigned long startTime = millis();  //get the start time
  char KeyRead = i2cKeypad.getKey();
  // Wait until press A
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Press A to setup");
  LCD.setCursor(0, 1);
  LCD.print("Temperature.");

  while (KeyRead == NO_KEY && (millis() - startTime) < 10000) {  //wait for input for 10 seconds
    KeyRead = i2cKeypad.getKey();                                //get the keypad input again
  }

  if (KeyRead != NO_KEY) {
    Serial.println(KeyRead);
    if (KeyRead == 'A') {
      LCD.clear();
      LCD.setCursor(0, 0);
      LCD.print("Enter Temperature");
      LCD.setCursor(0, 1);

      char input[3];  // array to store the input
      int i = 0;      // index variable for the array

      while (i < 2 && (millis() - startTime) < 5000) {  //wait for input for 10 seconds
        char newKey = i2cKeypad.getKey();

        if (newKey != NO_KEY && isDigit(newKey)) {  // if a digit key is pressed
          input[i] = newKey;                        // store the digit in the array
          Serial.print(newKey);                     // print the digit to the serial monitor
          // print in lcd
          i++;  // increment the index variable
        }
      }
      input[2] = '\0';        // add null character at the end of the array
      maxTemp = atoi(input);  // convert the input to an integer
      Serial.print("Max Temperature: ");
      Serial.println(maxTemp);
    }
  }

  delay(1000);
}

// Function for rotate solar panel by rotating two servo motors
void turnSolarPanel() {
  // inital reading of ldr
  int LDRLeft = analogRead(LDR1);
  int LDRRight = analogRead(LDR2);
  int diff = (LDRRight >= LDRLeft) ? LDRRight - LDRLeft : LDRLeft - LDRRight;

  Serial.print(LDRLeft);
  Serial.print("      ");
  Serial.println(LDRRight);

  // While the change is not in desireable range, rotate the solar panel
  while (diff > 150) {
    Serial.print(LDRLeft);
    Serial.print("      ");
    Serial.println(LDRRight);

    if (LDRLeft > LDRRight) {
      if (Def_Servo1 < 180 && Def_Servo2 > 0) {
        // Rotate the servo motors in two side
        // One in clockwise and other is anticlockwise
        Def_Servo1 = Def_Servo1 + 1;
        Def_Servo2 = Def_Servo2 - 1;
      }
      SolarServo1.write(Def_Servo1);
      SolarServo2.write(Def_Servo2);
    }
    if (LDRLeft < LDRRight) {
      if (Def_Servo1 > 0 && Def_Servo2 < 180) {
        // Rotate the servo motors in two side
        // One in clockwise and other is anticlockwise
        Def_Servo1 = Def_Servo1 - 1;
        Def_Servo2 = Def_Servo2 + 1;
      }
      SolarServo1.write(Def_Servo1);
      SolarServo2.write(Def_Servo2);
    }
    /***** Reduce this delay to speed up solar panel rotation speed ******/
    delay(50);
    LDRLeft = analogRead(LDR1);
    LDRRight = analogRead(LDR2);
    diff = (LDRRight >= LDRLeft) ? LDRRight - LDRLeft : LDRLeft - LDRRight;

    // When the light intensity change is large, but the servo can't rotate anymore break the loop
    if (Def_Servo1 < 2 || Def_Servo1 > 178 || Def_Servo2 < 2 || Def_Servo2 > 178) {
      break;
    }
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
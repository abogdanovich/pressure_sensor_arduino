// -------------------------------------------------
//      digital pressure instead of mechanic
// -------------------------------------------------

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

//define zone---------------
#define ledG 9      //green led
#define ledR 8      //red led
#define ledY 7      //yellow led
#define relay 2     //relay
#define sensor A0
#define dV 0.004    //5/1023 - each 5V via analog signal
#define PRESSURE_CORRECTION 0.4 //sensor correction
#define PMIN 2.0    //min for LOW level
#define PMAX 4.0    //max for HIGH level 
#define MAX_LCD_WIDTH 16
//-----------------------------

//init libs zone--------------
LiquidCrystal_I2C lcd(0x27, 16, 2); // Устанавливаем дисплей
//-----------------------------

bool b1Status = false;
bool b2Status = false;
bool working = false;
bool SYSTEM_ERROR = false;
uint8_t addr = 0;
unsigned long current_time = 0;
unsigned long old_time = 0;
unsigned long old_time2 = 0;
unsigned long test_millis = 0;
unsigned long prev_predict = 0; //start working
float pressurePascal = 0.0;

//-----------------------------------
float LOW_PRESSURE = 2.2;
float HIGH_PRESSURE = 3.5;
//-----------------------------------
float CURRENT_PRESSURE = 0.0; //current pressure
float Vout = 0.0; //current pressure
uint16_t analogV = 0;
//--------------------------------

void setup() {
  //
  Serial.begin(115200);

  current_time = millis();

  pinMode(ledG, OUTPUT);
  pinMode(ledR, OUTPUT);
  pinMode(ledY, OUTPUT);
  pinMode(relay, OUTPUT);
  pinMode(sensor, INPUT);

  analogV = getAnalogData();
  //Vout = (analogV * dV1) - dP;
  CURRENT_PRESSURE = getPressure(analogV);

  lcd.init();
  lcd.backlight();// Включаем подсветку дисплея
  lcd.clear();
  drawMenu();
  digitalWrite(ledG, LOW);
  digitalWrite(ledY, LOW);
  digitalWrite(ledR, LOW);

  test_millis = current_time;
}
void loop() {

  current_time = millis();

  //regular get sensor data
  analogV = getAnalogData();
  checkSensorHealth(analogV);

  //false means that we do not switch ON relay according to predict....
  //checking and geting other functions
  if (!SYSTEM_ERROR) {
    checkPressure();
    getPressure(analogV);
  }
  drawMenu();
  checkButtons(current_time);
}

//    update menu items
void drawMenu() {

  delay(100);

  lcd.setCursor(0, 0); //x,y
  lcd.print(LOW_PRESSURE);
  lcd.setCursor(6, 0); //x,y
  lcd.print(CURRENT_PRESSURE);
  lcd.setCursor(12, 0); //x,y
  lcd.print(HIGH_PRESSURE);

  if (!SYSTEM_ERROR) {

    uint8_t blocks = (CURRENT_PRESSURE - LOW_PRESSURE) / ((HIGH_PRESSURE - LOW_PRESSURE) / MAX_LCD_WIDTH);

    if (blocks > MAX_LCD_WIDTH) {
      blocks = 0;
    }

    for (uint8_t i = 15; i > blocks; i--) {
      lcd.setCursor(i, 1); //x,y
      lcd.write(254); //total hours working
    }

    //  draw blocks on LCD
    for (uint8_t i = 0; i < blocks - 1; i++) {
      lcd.setCursor(i, 1); //x,y
      lcd.write(255);
    }
  }

  else {
    lcd.setCursor(0, 1); //x,y
    lcd.print("low sensor data!");
  }

}

void calcPressure(uint16_t rawPressureValue) {
  Vout = (analog * (5/1023));
  pressurePascal = (3.0*(Vout-0.47))*1000000.0;
  //convert PSI into BAR
  CURRENT_PRESSURE = pressurePascal/10e5;
  if (CURRENT_PRESSURE < 0) {
    CURRENT_PRESSURE = 0;
  };
}

//    calc pressure from converted analog signal
float getPressure(uint16_t analog) {

  Vout = (analog * dV);
  //updated formula according to:
  // 0.4 - 0bar
  // 4.5 = 12bar (1.2Mpa)
  //−12x+4.1y+4.8=0 
  //we got the following: y(pressure) = (-48/41)+((120*x)/41) | x = voltage (analog * (5/1023) = 0.004))
  CURRENT_PRESSURE = (-48/41)+((120*Vout)/41)  + PRESSURE_CORRECTION; 

  return CURRENT_PRESSURE;
}

//    stop relay in case if sensor through the data less that normal and default 100 from A0 pin....
void checkSensorHealth(uint16_t analog) {
  if ((analog <= 50) or (analog >= 800) and !(SYSTEM_ERROR)) { //800~ its like ~7.5bar
    digitalWrite(ledR, HIGH);
    digitalWrite(relay, LOW); //switch OFF relay incase of error
    SYSTEM_ERROR = true;
  }
  if (SYSTEM_ERROR) {
    alarmErorr();
  }
}

//    alarm error
void alarmErorr(void) {
  if (SYSTEM_ERROR) {
    digitalWrite(ledR, !digitalRead(ledR));
    if (working) {
      working = false;
      digitalWrite(ledG, LOW);
      digitalWrite(relay, LOW);
    }
  }
}


//    check and control main relay using predict param in case of prediction.....
void checkPressure() {

  if (CURRENT_PRESSURE <= LOW_PRESSURE) {
    digitalWrite(ledG, HIGH);
    digitalWrite(relay, HIGH);
    working = true;
  }
  else if (CURRENT_PRESSURE >= HIGH_PRESSURE) {
    lcd.clear();
    digitalWrite(ledG, LOW);
    digitalWrite(relay, LOW);
    working = false;
  }
}


//    getting data from the analog arduino pin - default A0 using TEST param for local testing without sensor.....
uint16_t getAnalogData(void) {

  const uint8_t SIZE_BUF_ADC = 5;
  uint16_t buf_adc[SIZE_BUF_ADC], t;
  uint8_t i, j;

  for (i = 0; i < SIZE_BUF_ADC; i++) {
    buf_adc[i] = analogRead(sensor);

    //TODO: need to check and REMOVE delay
    delay(50);
    //TODO: need to check and REMOVE delay
  }

  //take mediana from buffer

  for (i = 0; i < SIZE_BUF_ADC; i++) {

    for (j = 0; j < SIZE_BUF_ADC - i - 1; j++) {
      if (buf_adc[j] > buf_adc[j + 1]) {
        t = buf_adc[j];
        buf_adc[j] = buf_adc[j + 1];
        buf_adc[j + 1] = t;
      }
    }
  }

  return buf_adc[(SIZE_BUF_ADC - 1) / 2];

}

float readDATA(uint8_t addr) {
  return (float)EEPROM.read(addr) / 10.0;
}

void saveDATA(uint8_t addr, uint8_t data) {

  EEPROM.write(addr, data);

}

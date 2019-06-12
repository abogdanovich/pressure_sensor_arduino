/**
 * digital pressure 
 * @author Alex Bogdanovich
  */
#define b1 6        //left LOW
#define b2 5        //right HIGH
#define ledG 9      //green led - RELAY indicator
#define ledR 8      //red led - error
#define ledY 7      //yellow led - PREDICT
#define RELAY 2     //RELAY!
#define sensor A0   //SENSOR
#define dV 0.004    //5/1023 - each 5V via analog signal
#define PRESSURE_CORRECTION 0.4 //sensor correction
#define PMIN 2.0    //min for LOW level
#define PMAX 4.0    //max for HIGH level 
#define MAX_LCD_WIDTH 16

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // Устанавливаем дисплей

bool b1Status = false;
bool b2Status = false;
bool working = false;
bool SYSTEM_ERROR = false;
uint8_t addr = 0;
unsigned long current_time = 0;
unsigned long old_time = 0;
unsigned long old_time2 = 0;
unsigned long test_millis = 0;
//unsigned long prev_predict = 0; //start working

float LOW_PRESSURE = 2.0;
float HIGH_PRESSURE = 3.0;
float CURRENT_PRESSURE = 0.0; //current pressure

float Vout = 0.0; //current pressure in voltage
uint16_t analogV = 0; //analog signal value

/**
 * setup method that allows to init system
 */
public void setup() {
  //
  Serial.begin(115200);

  //  try to load variables from EEPPROM arduino
  LOW_PRESSURE = readDATA(0);
  if (LOW_PRESSURE == 0.0) LOW_PRESSURE = 1.8;

  HIGH_PRESSURE = readDATA(1);
  if (HIGH_PRESSURE == 0.0) HIGH_PRESSURE = 2.9;

  current_time = millis();

  pinMode(b1, INPUT);
  pinMode(b2, INPUT);
  pinMode(sensor, INPUT);
  
  pinMode(ledG, OUTPUT);
  pinMode(ledR, OUTPUT);
  //pinMode(ledY, OUTPUT);
  pinMode(RELAY, OUTPUT);
  
  analogV = getAnalogData();
  //Vout = (analogV * dV1) - dP;
  CURRENT_PRESSURE = getPressure(analogV);

  lcd.init();
  lcd.backlight();//switch display light
  lcd.clear();

  drawMenu();
  OFF(ledG);
  OFF(ledR);
  //digitalWrite(ledY, LOW);

  test_millis = current_time;
}
public void loop() {

  current_time = millis();

  //regular get sensor data
  analogV = getAnalogData();
  checkSensorHealth(analogV);

  //checking and geting other functions
  if (!SYSTEM_ERROR) {
    checkPressure();
    getPressure(analogV);
  }
  drawMenu();
  checkButtons(current_time);
}

/**
 * update menu items
 */
public void drawMenu() {

  lcd.setCursor(0, 0); //x,y
  lcd.print(LOW_PRESSURE);
  lcd.setCursor(6, 0); //x,y
  lcd.print(CURRENT_PRESSURE);
  lcd.setCursor(12, 0); //x,y
  lcd.print(HIGH_PRESSURE);

  if (!SYSTEM_ERROR) {
    uint8_t blocks = (CURRENT_PRESSURE - LOW_PRESSURE) / ((HIGH_PRESSURE - LOW_PRESSURE) / MAX_LCD_WIDTH);

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

/**
 * calc pressure from converted analog signal
 */
public float getPressure(uint16_t analog) {
  Vout = (analog * dV);
  //updated formula according to:
  // 0.4 - 0bar
  // 4.5 = 12bar (1.2Mpa)
  //−12x+4.1y+4.8=0 
  //we got the following: y(pressure) = (-48/41)+((120*x)/41) | x = voltage (analog * (5/1023) = 0.004))
  CURRENT_PRESSURE = (-48/41)+((120*Vout)/41)  + PRESSURE_CORRECTION; 
  return CURRENT_PRESSURE;
}

/**
 * stop RELAY in case if sensor throws the data less that normal 
 */
public void checkSensorHealth(uint16_t analog) {
  if ((analog <= 50) or (analog >= 800) and !(SYSTEM_ERROR)) { //800~ its like ~7.5bar
    ON(ledR);
    OFF(RELAY);
    SYSTEM_ERROR = true;
  }
  if (SYSTEM_ERROR) {
    alarmErorr();
  }
}

//    alarm error
public void alarmErorr(void) {
  if (SYSTEM_ERROR) {
    ON(ledR);
    if (working) {
      working = false;
      OFF(ledG);
      OFF(RELAY);
    }
  }
}

/**
 * check and control main RELAY
 */
public void checkPressure() {
  if (CURRENT_PRESSURE <= LOW_PRESSURE) {
    ON(ledG);
    ON(RELAY);
    working = true;
  }
  else if (CURRENT_PRESSURE >= HIGH_PRESSURE) {
    lcd.clear();
    OFF(ledG);
    OFF(RELAY);
    working = false;
  }
}


/**
 * get data from the analog arduino pin 
 * default A0 using TEST param for local testing without sensor.....
 */
public uint16_t getAnalogData(void) {
  const uint8_t SIZE_BUF_ADC = 5;
  uint16_t buf_adc[SIZE_BUF_ADC];
  uint16_t t;

  for (uint8_t i = 0; i < SIZE_BUF_ADC; i++) {
    buf_adc[i] = analogRead(sensor);
    delay(50);
  }

  //take mediana from buffer
  for (uint8_t i = 0; i < SIZE_BUF_ADC; i++) {
    for (uint8_t j = 0; j < SIZE_BUF_ADC - i - 1; j++) {
      if (buf_adc[j] > buf_adc[j + 1]) {
        t = buf_adc[j];
        buf_adc[j] = buf_adc[j + 1];
        buf_adc[j + 1] = t;
      }
    }
  }
  return buf_adc[(SIZE_BUF_ADC - 1) / 2];
}

/**
 * check buttons state for PMIN and PMAX corection
 */
public void checkButtons(unsigned long current_time) {

  if (digitalRead(b1) and ((current_time - old_time) > 200) and !b1Status) {
    old_time = current_time;
    b1Status = true;

    if (LOW_PRESSURE >= PMIN) {
      LOW_PRESSURE = 1;
    }
    else {
      LOW_PRESSURE += 0.1;
    }
    drawMenu();
  }
  else if (!digitalRead(b1) and b1Status) {
    b1Status = false;
    saveDATA(0, LOW_PRESSURE * 10);
  }

  // check the second button
  if (digitalRead(b2) and ((current_time - old_time2) > 200) and !b2Status) {
    old_time2 = current_time;
    b2Status = true;

    if (HIGH_PRESSURE >= PMAX) {
      HIGH_PRESSURE = 2;
    }
    else {
      HIGH_PRESSURE += 0.1;
    }
    drawMenu();
  }
  else if (!digitalRead(b2) and b2Status) {
    b2Status = false;
    saveDATA(1, (HIGH_PRESSURE + 0.1) * 10);
  }
}

/**
 * swith ON RELAY\led 
 */
public void ON(uint8_t pin) {
  digitalWrite(pin, HIGH);
}

/**
 * swith ON RELAY\led 
 */
public void OFF(uint8_t pin) {
  digitalWrite(pin, LOW);
}

/**
 * read sensor data
 */
public float readDATA(uint8_t addr) {
  return (float)EEPROM.read(addr) / 10.0;
}

/**
 * save data to EEPOM
 */
public void saveDATA(uint8_t addr, uint8_t data) {
  EEPROM.write(addr, data);
}

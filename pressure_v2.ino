/**
 * digital pressure 
 * @author Alex Bogdanovich
  */
#define b1 6        //left LOW
#define b2 5        //right HIGH
#define ledG 9      //green led - RELAY indicator
#define ledR 8      //red led - error
#define RELAY 2     //RELAY!
#define sensor A0   //SENSOR
#define dV 0.004    //5/1023 - each 5V via analog signal
#define PMIN 2.0    //min for LOW level
#define PMAX 3.0    //max for HIGH level 
#define MAX_LCD_WIDTH 16
#define MIN_SENSOR_VALUE 50
#define MAX_SENSOR_VALUE 800
#define EEPROM_OFFSET 10.0
#define PRESSURE_SHIFT 0.1
#define LIMIT_BUTTON_SECONDS 100

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

float LOW_PRESSURE = 2.0;
float HIGH_PRESSURE = 3.0;
float CURRENT_PRESSURE = 0.0; //current pressure

float Vout = 0.0; //current pressure in voltage
uint16_t analogV = 0; //analog signal value

/**
 * setup method that allows to init system
 */
void setup() {
  blink(3);
//  Serial.begin(115200);

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
  pinMode(RELAY, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  
  analogV = getAnalogData();
  CURRENT_PRESSURE = getPressure(analogV);

  lcd.init();
  lcd.backlight();//switch display light
  lcd.clear();

  drawMenu();
  OFF(ledG);
  OFF(ledR);
}

void blink(int8_t count) {
  for (int8_t i=0; i<count; i++) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(300);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(300);                       // wait for a second  
  }
}
void loop() {

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
void drawMenu() {

  lcd.setCursor(0, 0); //x,y
  lcd.print(LOW_PRESSURE);
  lcd.setCursor(6, 0); //x,y
  lcd.print(CURRENT_PRESSURE);
  lcd.setCursor(12, 0); //x,y
  lcd.print(HIGH_PRESSURE);

  if (!SYSTEM_ERROR) {
    uint8_t blocks = (CURRENT_PRESSURE - LOW_PRESSURE) / ((HIGH_PRESSURE - LOW_PRESSURE) / MAX_LCD_WIDTH);

    //set protected blocks counter
    if (blocks < 0) {
      blocks = 0;
    } else if (blocks > 15) {
      blocks = 15;
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

/**
 * calc pressure from converted analog signal
 */
float getPressure(uint16_t analog) {
  Vout = (analog * dV);
  float pressure_pascal = (3.0*(Vout-0.47))*1000000.0;
  //convert PSI into BAR
  CURRENT_PRESSURE = pressure_pascal/10e5;
  return CURRENT_PRESSURE;
}

/**
 * stop RELAY in case if sensor throws the data less that normal 
 */
void checkSensorHealth(uint16_t analog) {
  if ((analog <= MIN_SENSOR_VALUE) or (analog >= MAX_SENSOR_VALUE) and !(SYSTEM_ERROR)) { //800~ its like ~7.5bar
    ON(ledR);
    OFF(RELAY);
    SYSTEM_ERROR = true;
  }
  if (SYSTEM_ERROR) {
    alarmErorr();
  }
}

//    alarm error
void alarmErorr(void) {
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
void checkPressure() {
  if (CURRENT_PRESSURE <= LOW_PRESSURE) {
    ON(ledG);
    ON(RELAY);
    working = true;
    ON(LED_BUILTIN);
  }
  else if (CURRENT_PRESSURE >= HIGH_PRESSURE) {
    lcd.clear();
    OFF(ledG);
    OFF(RELAY);
    working = false;
    OFF(LED_BUILTIN);
  }
}

/**
 * get data from the analog arduino pin 
 * default A0 using TEST param for local testing without sensor.....
 */
uint16_t getAnalogData(void) {
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
void checkButtons(unsigned long current_time) {

  if (digitalRead(b1) and ((current_time - old_time) > LIMIT_BUTTON_SECONDS) and !b1Status) {
    old_time = current_time;
    b1Status = true;

    if (LOW_PRESSURE >= PMIN+2.0) {
      LOW_PRESSURE = PMIN;
    }
    else {
      LOW_PRESSURE += 0.1;
    }
    drawMenu();
  }
  else if (!digitalRead(b1) and b1Status) {
    b1Status = false;
    saveDATA(0, LOW_PRESSURE * EEPROM_OFFSET);
  }

  // check the second button
  if (digitalRead(b2) and ((current_time - old_time2) > LIMIT_BUTTON_SECONDS) and !b2Status) {
    old_time2 = current_time;
    b2Status = true;

    if (HIGH_PRESSURE >= PMAX + 2.0) {
      HIGH_PRESSURE = PMAX;
    }
    else {
      HIGH_PRESSURE += PRESSURE_SHIFT;
    }
    drawMenu();
  }
  else if (!digitalRead(b2) and b2Status) {
    b2Status = false;
    saveDATA(1, (HIGH_PRESSURE + PRESSURE_SHIFT) * EEPROM_OFFSET);
  }
}

/**
 * swith ON RELAY\led 
 */
void ON(uint8_t pin) {
  digitalWrite(pin, HIGH);
}

/**
 * swith ON RELAY\led 
 */
void OFF(uint8_t pin) {
  digitalWrite(pin, LOW);
}

/**
 * read sensor data
 */
float readDATA(uint8_t addr) {
  return (float)EEPROM.read(addr) / EEPROM_OFFSET;
}

/**
 * save data to EEPOM
 */
void saveDATA(uint8_t addr, uint8_t data) {
  EEPROM.write(addr, data);
}

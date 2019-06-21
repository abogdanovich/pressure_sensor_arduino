/**
 * digital pressure 
 * @author Alex Bogdanovich
  */
#define b1 6                //left LOW
#define b2 5                //right HIGH
#define ledG 9              //green led - RELAY indicator
#define ledR 8              //red led - error
#define RELAY 2             //RELAY!
#define sensor A0           //SENSOR
#define MIN_PRESSURE_THRESHOLD 2.0    //min for LOW level
#define MAX_PRESSURE_THRESHOLD 6.0    //max for HIGH level 
#define MAX_LCD_WIDTH 16
#define MIN_SENSOR_VALUE 50
#define MAX_SENSOR_VALUE 800
#define EEPROM_OFFSET 10.0
#define PRESSURE_OFFSET 0.1
#define LIMIT_BUTTON_SECONDS 100

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); 

bool b1Status = false;
bool b2Status = false;
bool isWorking = false;
bool systemError = false;
uint8_t addr = 0;
unsigned long current_time = 0;
unsigned long timerSeconds1 = 0;
unsigned long timerSeconds2 = 0;

float lowPressure = 2.0;
float highPressure = 5.0;
float currentPressureValue = 0.0; //current pressure
float pressureInVoltage = 0.0;    //current pressure in voltage
uint16_t rawSensorValue = 0;      //analog signal value

//[] char for lcd
char emptySymbol[] {
  B11111,
  B10001,
  B10001,
  B10001,
  B10001,
  B10001,
  B10001,
  B11111  
};

/**
 * setup method that allows to init system
 */
void setup() {
  //  try to load variables from EEPPROM arduino
  uint8_t lowPressure = readDATA(0);
  if (lowPressure > MIN_PRESSURE_THRESHOLD) {
    lowPressure = lowPressure;
  } 
  uint8_t highPressure = readDATA(1);
  if (highPressure > 0.0 && highPressure < MAX_PRESSURE_THRESHOLD) {
    highPressure = highPressure;
  } 
  
  current_time = millis();

  pinMode(b1, INPUT);
  pinMode(b2, INPUT);
  pinMode(sensor, INPUT);
  
  pinMode(ledG, OUTPUT);
  pinMode(ledR, OUTPUT);
  pinMode(RELAY, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  
  rawSensorValue = getAnalogData();
  currentPressureValue = calcPressure(rawSensorValue);
  lcd.init();
  lcd.backlight();//switch display light
  lcd.clear();
  lcd.createChar(0, emptySymbol);
  
  drawMenu();
  OFF(ledG);
  OFF(ledR);
}

void loop() {

  current_time = millis();

  //regular get sensor data
  rawSensorValue = getAnalogData();
  
  checkSensorHealth(rawSensorValue);
 
  //checking and geting other functions
  if (!systemError) {
    checkPressure();
    calcPressure(rawSensorValue);
  }
  drawMenu();
  checkButtons(current_time);
}

/**
 * update menu items
 */
void drawMenu() {
  
  lcd.setCursor(0, 0); //x,y
  lcd.print(lowPressure);
  lcd.setCursor(6, 0); //x,y
  lcd.print(currentPressureValue);
  lcd.setCursor(12, 0); //x,y
  lcd.print(highPressure);

  if (!systemError) {
    //current pressure in percents 
    uint8_t currentPercent = ((currentPressureValue - lowPressure) * 100) / (highPressure - lowPressure); 
    uint8_t blockInPercent = 100 / MAX_LCD_WIDTH;
    uint8_t drawBlocks = currentPercent / blockInPercent;
    
    if (drawBlocks > MAX_LCD_WIDTH) {
      drawBlocks = 0;
    }

    for (int i=0; i<MAX_LCD_WIDTH-1; i++) {
      lcd.setCursor(i, 1);
      lcd.write(emptySymbol);
      
    }
  
    for (int i=0; i<drawBlocks-1; i++) {
      lcd.setCursor(i, 1);
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
float calcPressure(uint16_t rawPressureValue) {
  pressureInVoltage = (rawPressureValue * (5/1023));
  float pressure_pascal = (3.0*(pressureInVoltage-0.47))*1000000.0;
  //convert PSI into BAR
  currentPressureValue = pressure_pascal/10e5;
  if (currentPressureValue < 0) {
    currentPressureValue = 0;
  }
  return currentPressureValue;
}

/**
 * stop RELAY in case if sensor throws the data less that normal 
 */
void checkSensorHealth(uint16_t analog) {
  if ((analog <= MIN_SENSOR_VALUE) or (analog >= MAX_SENSOR_VALUE) and !(systemError)) { //800~ its like ~7.5bar
    ON(ledR);
    OFF(RELAY);
    systemError = true;
  }
  if (systemError) {
    alarmErorr();
  }
}

//    alarm error
void alarmErorr(void) {
  if (systemError) {
    ON(ledR);
    if (isWorking) {
      isWorking = false;
      OFF(ledG);
      OFF(RELAY);
    }
  }
}

/**
 * check and control main RELAY
 */
void checkPressure() {
  if (currentPressureValue <= lowPressure) {
    ON(ledG);
    ON(RELAY);
    isWorking = true;
    ON(LED_BUILTIN);
  }
  else if (currentPressureValue >= highPressure) {
    lcd.clear();
    OFF(ledG);
    OFF(RELAY);
    isWorking = false;
    OFF(LED_BUILTIN);
  }
}

/**
 * get data from the analog arduino pin 
 * default A0 using TEST param for local testing without sensor.....
 */
uint16_t getAnalogData() {
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
 * check buttons state for MIN_PRESSURE_THRESHOLD and MAX_PRESSURE_THRESHOLD corection
 */
void checkButtons(unsigned long current_time) {

  if (digitalRead(b1) and ((current_time - timerSeconds1) > LIMIT_BUTTON_SECONDS) and !b1Status) {
    timerSeconds1 = current_time;
    b1Status = true;

    if (lowPressure >= MIN_PRESSURE_THRESHOLD) {
      lowPressure = MIN_PRESSURE_THRESHOLD;
    }
    else {
      lowPressure += PRESSURE_OFFSET;
    }
    drawMenu();
  }
  else if (!digitalRead(b1) and b1Status) {
    b1Status = false;
    saveDATA(0, lowPressure * EEPROM_OFFSET);
  }

  // check the second button
  if (digitalRead(b2) and ((current_time - timerSeconds2) > LIMIT_BUTTON_SECONDS) and !b2Status) {
    timerSeconds2 = current_time;
    b2Status = true;

    if (highPressure >= MAX_PRESSURE_THRESHOLD + 2.0) {
      highPressure = MAX_PRESSURE_THRESHOLD;
    }
    else {
      highPressure += PRESSURE_OFFSET;
    }
    drawMenu();
  }
  else if (!digitalRead(b2) and b2Status) {
    b2Status = false;
    saveDATA(1, (highPressure + PRESSURE_OFFSET) * EEPROM_OFFSET);
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

/**
 * digital pressure 
 * @author Alex Bogdanovich
  */
#define ledG 9                        //green led - RELAY indicator
#define ledR 8                        //red led - error
#define RELAY 2                       //RELAY!
#define sensor A0                     //SENSOR
#define MAX_LCD_WIDTH 16
#define MAX_LCD_HEIGHT 2
#define MIN_SENSOR_VALUE 50
#define MAX_SENSOR_VALUE 800
#define MILLIS_1H_THRESHOLD 3600000   //60*60*1000 - each 1 hours save
#define HOURS_IN_DAY 24
#define EEPROM_WORKING_HOURS_DATA 2
#define EEPROM_WORKING_DAYS_DATA 3
#define VOLTAGE_STEP 0.004887

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

LiquidCrystal_I2C lcd(0x27, MAX_LCD_WIDTH, MAX_LCD_HEIGHT); 

bool isWorking = false;
bool systemError = false;
uint8_t addr = 0;
unsigned long currentSeconds = 0;
unsigned long timerStart3 = 0;
unsigned long timerWorkingStartStop = 0;

byte totalWorkingHours = 0; //display and save/load in days 0-255 1 bit
unsigned long totalWorkingMillis = 0; //display and save/load in days 0-255 1 bit
byte totalWorkingDays = 0; //display and save/load in days 0-255 1 bit

float lowPressure = 2.3;
float highPressure = 3.5;

uint16_t rawSensorValue = 0;      //analog signal value
float currentPressureValue = 0.0; //current pressure
float pressureInVoltage = 0.0;
float pressure_pascal = 0.0;

/**
 * setup method that allows to init system
 */
void setup() {
  //load the data from EEPPROM memory hours and days of work
  byte totalWorkingHours_temp = readEEPROMPressureData(EEPROM_WORKING_HOURS_DATA);  
  if (totalWorkingHours_temp > 0) {
    totalWorkingHours = totalWorkingHours_temp;
  }
  //load the data from EEPPROM memory hours and days of work
  byte totalWorkingDays_temp = readEEPROMPressureData(EEPROM_WORKING_DAYS_DATA);  
  if (totalWorkingDays_temp > 0) {
    totalWorkingDays = totalWorkingDays_temp;
  }
  //init buttons
  pinMode(sensor, INPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledR, OUTPUT);
  pinMode(RELAY, OUTPUT);
  
  rawSensorValue = getAnalogData();
  calcPressure(rawSensorValue);
  lcd.init();
  lcd.backlight();//switch display light
  lcd.clear();
  
  drawMenu();

  OFF(ledG);
  OFF(ledR);
}

void loop() {
  currentSeconds = millis();
  rawSensorValue = getAnalogData();
  checkSensorHealth(rawSensorValue);
  //checking and geting other functions
  if (!systemError) {
    checkPressure();
    calcPressure(rawSensorValue);
  }
  drawMenu();
  calcAndSaveTotalWork(currentSeconds);
}

/**
 * Calculate total working hours\days and save the data into EEPROM
 * EEPROM (100,000/24/365) write/erase cycles ~ 11 years for writing each hour
 */
 void calcAndSaveTotalWork(unsigned long currentSeconds) {
  if ((currentSeconds - timerStart3) > MILLIS_1H_THRESHOLD) {
    if (!isWorking) {
      //save data to EEPPROM when we do not work
      if (totalWorkingHours >= HOURS_IN_DAY) {
        totalWorkingDays++;
        totalWorkingHours -= HOURS_IN_DAY;
        saveEEPROMPressureData(EEPROM_WORKING_HOURS_DATA, totalWorkingHours);  
        saveEEPROMPressureData(EEPROM_WORKING_DAYS_DATA, totalWorkingDays);  
      } else {
        saveEEPROMPressureData(EEPROM_WORKING_HOURS_DATA, totalWorkingHours);    
      }
    }
  }
 }

/**
 * Update LCD menu items 16*2
 */
void drawMenu() {
  
  if (!systemError) {
    lcd.setCursor(0, 0); 
    lcd.print("H:");
    lcd.setCursor(2, 0); 
    lcd.print(highPressure);
  
    lcd.setCursor(0, 1); 
    lcd.print("L:");
    lcd.setCursor(2, 1); 
    lcd.print(lowPressure);
  
    lcd.setCursor(5, 0); 
    lcd.print("|");
    lcd.setCursor(5, 1); 
    lcd.print("|");
  
    lcd.setCursor(7, 0); 
    lcd.print(currentPressureValue);
    lcd.setCursor(12, 0); 
    lcd.print("bar");
  
     
    //total hours \ days totalWorkingMillis | totalWorkingHours | totalWorkingDays
    if (totalWorkingHours > 1) {
      lcd.setCursor(7, 1);
      lcd.print(totalWorkingHours);
      lcd.setCursor(12, 1); 
      lcd.print("h");
    } else if (totalWorkingMillis > 1000*60) {
      lcd.setCursor(7, 1);
      lcd.print(totalWorkingMillis/1000/60);
      lcd.setCursor(12, 1); 
      lcd.print("m");
    } else {
      lcd.setCursor(7, 1);
      lcd.print(totalWorkingMillis/1000);
      lcd.setCursor(12, 1); 
      lcd.print("s");
    }
    
  }   
  else {
    lcd.setCursor(0, 1); 
    lcd.print("low sensor data!");
  }
}

/**
 * calc pressure from converted analog signal
 */
void calcPressure(uint16_t rawPressureValue) {
  pressureInVoltage = (rawPressureValue * VOLTAGE_STEP);
  pressure_pascal = (3.0*(pressureInVoltage-0.47))*1000000.0;
  //convert PSI into BAR
  currentPressureValue = pressure_pascal/10e5;
  if (currentPressureValue < 0) {
    currentPressureValue = 0;
  }
}

/**
 * stop RELAY in case if sensor throws the data less that normal 
 */
void checkSensorHealth(uint16_t analog) {
  if (analog <= MIN_SENSOR_VALUE or analog >= MAX_SENSOR_VALUE && !(systemError)) { //800~ its like ~7.5bar
    ON(ledR);
    OFF(RELAY);
    systemError = true;
  }
  if (systemError) {
    alarmErorr();
  }
}

/**
 * alarm error allows to disable the system and save the pump
 */
void alarmErorr() {
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
 * count pump working hours by converting from millis into hours
 */
void countPumpWorkingTime() {
    totalWorkingMillis += (millis() - timerWorkingStartStop);
    if ((totalWorkingMillis / MILLIS_1H_THRESHOLD) >= 1) {
      totalWorkingHours++; //inc working hours
      totalWorkingMillis -= MILLIS_1H_THRESHOLD;
    }
}

/**
 * check and control the main RELAY
 */
void checkPressure() {
  if (currentPressureValue <= lowPressure && isWorking == false) {
    ON(ledG);
    ON(RELAY);
    isWorking = true;
    //start counting working millis
    timerWorkingStartStop = millis();
    //
    ON(LED_BUILTIN);
  }
  else if (currentPressureValue >= highPressure && isWorking == true) {
    lcd.clear();
    OFF(ledG);
    OFF(RELAY);
    isWorking = false;
    //count working millis
    countPumpWorkingTime();
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
  byte i;
  byte j;

  for (i = 0; i < SIZE_BUF_ADC; i++) {
    buf_adc[i] = analogRead(sensor);
    delay(50);
  }

  //take the mediana from buffer
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
float readEEPROMPressureData(uint8_t addr) {
  return (float)EEPROM.read(addr);
}

/**
 * save data to EEPOM
 */
void saveEEPROMPressureData(uint8_t addr, uint8_t data) {
  EEPROM.write(addr, data);
}

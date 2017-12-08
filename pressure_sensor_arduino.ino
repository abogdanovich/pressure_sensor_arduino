// -------------------------------------------------
//      digital pressure instead of mechanic 
// -------------------------------------------------

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

//define zone---------------
#define b1 6        //left LOW
#define b2 5        //right HIGH
#define ledG 9      //green led
#define ledR 8      //red led
#define ledY 7      //yellow led
#define relay 2     //relay
#define sensor A0   
#define dV 0.004    //5/1023 - each 5V via analog signal
#define dP 0.40     //default 0.4 signal from sensor
#define Pcorrection 0.2 //sensor correction
#define Pmin 2.0    //min for LOW level
#define Pmax 4.0    //max for HIGH level 

//-----------------------------

//init libs zone--------------
LiquidCrystal_I2C lcd(0x27, 16, 2); // Устанавливаем дисплей
//-----------------------------

bool b1Status = false;
bool b2Status = false;
bool working = false;
bool SYSTEM_ERROR = false;
uint8_t addr = 0;
unsigned long cur_time = 0;
unsigned long old_time = 0;
unsigned long old_time2 = 0;
unsigned long test_millis = 0;
unsigned long prev_predict = 0; //start working

//-----------------------------------
float P_low = 1.8;
float P_high = 2.9;
//-----------------------------------
float P_c = 0.0; //current pressure
float Pprev = 0.0;
float Vout = 0.0; //current pressure
uint16_t analogV = 0;
//--------------------------------

void setup() {
//
//  Serial.begin(115200);

  //  try to load variables from EEPPROM arduino
  P_low = readDATA(0);
  if (P_low == 0.0) P_low = 1.8;
  
  P_high = readDATA(1);
  if (P_high == 0.0) P_high = 2.9;
  
  cur_time = millis(); 
  
  pinMode(b1, INPUT);
  pinMode(b2, INPUT);

  pinMode(ledG, OUTPUT);
  pinMode(ledR, OUTPUT);
  pinMode(ledY, OUTPUT);
  pinMode(relay, OUTPUT);
  pinMode(sensor, INPUT);

  analogV = getAnalogData();
  Vout = (analogV * dV) - dP;
  P_c = getPressure(analogV, cur_time);
  Pprev = P_c;

  if (Vout < 0) Vout = 0.0;
  
  lcd.init();
  lcd.backlight();// Включаем подсветку дисплея
  lcd.clear();

  drawMenu();
  digitalWrite(ledG, LOW);
  digitalWrite(ledY, LOW);
  digitalWrite(ledR, LOW);

  test_millis = cur_time;
}
void loop() {

  cur_time = millis();
  
  //regular get sensor data
  analogV = getAnalogData();
  checkSensorHealth(analogV);

  //false means that we do not switch ON relay according to predict....
  //checking and geting other functions
  if (!SYSTEM_ERROR) {
    checkPressure();
    getPressure(analogV, cur_time);
  }
  drawMenu();
  checkButtons(cur_time);
}

//    update menu items
void drawMenu() {

  lcd.setCursor(0, 0); //x,y
  lcd.print(P_low);
  lcd.setCursor(6, 0); //x,y
  lcd.print(P_c);
  lcd.setCursor(12, 0); //x,y
  lcd.print(P_high);

  if (!SYSTEM_ERROR) {

    float blok = ((P_high-P_low))/16; //one block
    uint8_t blocks = (P_c - P_low) / blok;

    for (uint8_t i=16; i>blocks; i--) {
      lcd.setCursor(i, 1); //x,y
      lcd.write(254); //total hours working
    }

    //  draw blocks on LCD
    for (uint8_t i=0; i<blocks; i++) {
      lcd.setCursor(i, 1); //x,y
      lcd.write(255); //total hours working
    }
    
    

  }

  else {
    lcd.setCursor(0, 1); //x,y
    lcd.print("low sensor data!");
  }

}

//    calc pressure from converted analog signal
float getPressure(uint16_t analog, unsigned long cur_time) {

  Vout = (analog * dV) - dP;
  if (Vout < 0) Vout = 0.0;
  P_c = ((11.5 * Vout + 2.25) / 4.5) + Pcorrection;
  
  return P_c;

}

//    stop relay in case if sensor through the data less that normal and default 100 from A0 pin....
void checkSensorHealth(uint16_t analog) {
  if ((analog <= 50) and !SYSTEM_ERROR) {
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
      digitalWrite(ledY, LOW); //switch ON predict LED
    }
    
  }
}


//    check and control main relay using predict param in case of prediction.....
void checkPressure() {
  
  if (P_c <= P_low) {
    digitalWrite(ledG, HIGH);
    digitalWrite(relay, HIGH);
    working = true;
  }
  else if (P_c >= P_high) {
    digitalWrite(ledG, LOW);
    digitalWrite(relay, LOW);
    working = false;
    digitalWrite(ledY, LOW); //switch ON predict LED
  }

}


//    getting data from the analog arduino pin - default A0 using
uint16_t getAnalogData(void) {

  const uint8_t SIZE_BUF_ADC = 5;
  uint16_t buf_adc[SIZE_BUF_ADC], t;
  uint8_t i, j;

  for (i = 0; i<SIZE_BUF_ADC; i++) {
    buf_adc[i] = analogRead(sensor);
  }

  //take mediana from buffer

   for (i = 0; i<SIZE_BUF_ADC; i++) {

    for (j = 0; j<SIZE_BUF_ADC-i-1; j++) {
      if (buf_adc[j] > buf_adc[j+1]) {
        t = buf_adc[j];
        buf_adc[j] = buf_adc[j+1];
        buf_adc[j+1] = t;
      }
    }
   }

   return buf_adc[(SIZE_BUF_ADC-1)/2];

  //try to check simulation.....
  //analogV = analogRead(sensor);
 
  //return analogV;

}


//    check box buttons
void checkButtons(unsigned long cur_time) {

  if (digitalRead(b1) and ((cur_time - old_time) > 200) and !b1Status) {

    old_time = cur_time;
    b1Status = true;

    if (P_low >= Pmin) {
      P_low = 1;
    }
    else {
      P_low += 0.1;
    }
    drawMenu();

    

  }
  else if (!digitalRead(b1) and b1Status) {
    b1Status = false;
    saveDATA(0, P_low*10);
  }

  // check the second button

  if (digitalRead(b2) and ((cur_time - old_time2) > 200) and !b2Status) {

    old_time2 = cur_time;
    b2Status = true;

    if (P_high >= Pmax) {
      P_high = 2;
    }
    else {
      P_high += 0.1;
    }
    drawMenu();
    
    

  }
  else if (!digitalRead(b2) and b2Status) {
    b2Status = false;
    saveDATA(1, (P_high+0.1)*10);
  }

}

float readDATA(uint8_t addr) {
  return (float)EEPROM.read(addr)/10.0;  
}

void saveDATA(uint8_t addr, uint8_t data) {
  
  EEPROM.write(addr, data);
  
}

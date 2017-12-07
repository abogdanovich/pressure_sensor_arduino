// -------------------------------------------------
//      digital pressure instead of mechanic 
//      author: Alex Bogdanovich | bogdanovich.alex[at]gmail.com
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
#define Pcorrection 0.3 //sensor correction
#define Pmin 2.0    //min for LOW level
#define Pmax 4.0    //max for HIGH level 
#define TEST false  //testing purpose without sensor
//-----------------------------

//init libs zone--------------
LiquidCrystal_I2C lcd(0x27, 16, 2); // Устанавливаем дисплей
//-----------------------------

bool b1Status = false;
bool b2Status = false;
float test_inc = -0.01;
bool working = false;
bool SYSTEM_ERROR = false;
int addr = 0;
float lowIndex = 0.05;

signed long cur_time = 0;
signed long old_time = 0;
signed long old_time2 = 0;
signed long test_millis = 0;
signed long prev_predict = 0; //start working

//-----------------------------------
float P_low = 1.8;
float P_high = 2.9;
//-----------------------------------
float P_c = 0.0; //current pressure
float Pprev = 0.0;
float Vout = 0.0; //current pressure
int analogV = 0;
float HALF_HYDRO_TANK = P_low + ((P_high - P_low) / 2);
float tempindex = lowIndex;
//--------------------------------

void setup() {
//
//  Serial.begin(115200);

  //  try to load variables from EEPPROM arduino
  P_low = readDATA(0);
//  Serial.println(P_low);
  if (P_low == 0.0) P_low = 1.8;
  
  P_high = readDATA(1);
//  Serial.println(P_high);
  if (P_high == 0.0) P_high = 2.9;

  
  cur_time = millis(); 
  
  pinMode(b1, INPUT);
  pinMode(b2, INPUT);

  pinMode(ledG, OUTPUT);
  pinMode(ledR, OUTPUT);
  pinMode(ledY, OUTPUT);
  pinMode(relay, OUTPUT);
  pinMode(sensor, INPUT);

  if (TEST) {
    analogV = 100;
    P_c = P_high;
  }

  analogV = getAnalogData(sensor);
  Vout = (analogV * dV) - dP;
  P_c = getPressure(analogV, cur_time);
  Pprev = P_c;

  if (Vout < 0) {
    Vout = 0.0;
  }
  
  lcd.init();
  lcd.backlight();// Включаем подсветку дисплея
  lcd.clear();

  drawMenu();
  digitalWrite(ledG, LOW);
  digitalWrite(ledY, LOW);
  digitalWrite(ledR, LOW);

  test_millis = cur_time;
  prev_predict = cur_time;
}
void loop() {

  cur_time = millis();
  
  //regular get sensor data
  if (!TEST) analogV = getAnalogData(sensor);
  checkSensorHealth(analogV);

  //false means that we do not switch ON relay according to predict....
  //checking and geting other functions
  if (!SYSTEM_ERROR) {
    checkPressure(false);
    getPressure(analogV, cur_time);
    predictPressure(P_c, cur_time);
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

    lcd.setCursor(0, 1); //x,y
    lcd.print(lowIndex);
    lcd.setCursor(6, 1); //x,y
    
    float blok = ((P_high-P_low))/10; //one block
    int blocks = (P_c - P_low) / blok;

    //  draw blocks on LCD
    for (int i=0; i<blocks; i++) lcd.write(255); //total hours working
    for (int i=0; i<10; i++) lcd.write(254); //total hours working

  }

  else {
    lcd.setCursor(0, 1); //x,y
    lcd.print("low sensor data!");
  }

}

//    calc pressure from converted analog signal
float getPressure(int analog, long cur_time) {

  if (TEST) {
    if (cur_time - test_millis >= 2000) {
      //Serial.println(test_inc);
      if (working) {
        test_inc = 0.1;
      }
      else {
        if (P_c <= P_low) {
          test_inc = 0.01;
        }
        else if (P_c >= P_high) {
          test_inc = -0.01;
        }
      }
      test_millis = cur_time;
      P_c += test_inc;
    }
  }

  else {
    Vout = (analog * dV) - dP;
  if (Vout < 0) {
      Vout = 0.0;
    }
    P_c = ((11.5 * Vout + 2.25) / 4.5) + Pcorrection;
  }
  
  return P_c;

}

//    stop relay in case if sensor through the data less that normal and default 100 from A0 pin....
void checkSensorHealth(int analog) {
  if (analog <= 50 and !SYSTEM_ERROR) {
    digitalWrite(ledR, HIGH);
    digitalWrite(relay, LOW); //switch OFF relay incase of error
    SYSTEM_ERROR = true;
  }
  if (SYSTEM_ERROR) {
    alarmErorr();
  }
}

//    alarm error
void alarmErorr() {
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
void checkPressure(bool predict) {
  
  if ((P_c <= P_low) or predict) {
    digitalWrite(ledG, HIGH);
    digitalWrite(relay, HIGH);
    working = true;
  }
  else if (P_c >= P_high) {
    digitalWrite(ledG, LOW);
    digitalWrite(relay, LOW);
    working = false;
    digitalWrite(ledY, LOW); //switch ON predict LED
    Pprev = P_c; //need to update Pprev because of full tank
  }

}


//    predict pressure quick falling using adopted lowIndex param
void predictPressure(float Pcur, long cur_time) {
  //check each second
  if ((cur_time - prev_predict) >= 9000 and (!working)) {
      
    //in case when we we have a halt of boil
    if (Pcur <= HALF_HYDRO_TANK) {
      
      tempindex = (Pprev - Pcur);

      if ((tempindex <= lowIndex) and tempindex > 0.01) {
        
        //quickly lowest pressure ~-0.2Bar per second
        //predic and switch ON relay
        //save a new lowindex and start
        lowIndex = tempindex;
        checkPressure(true); //call predict TRUE and turn ON relay :)
        digitalWrite(ledY, HIGH); //switch ON predict LED

      }
      Pprev = Pcur;
    }  
    prev_predict = cur_time;
  }
}

//    getting data from the analog arduino pin - default A0 using TEST param for local testing without sensor.....
int getAnalogData(byte sensor) {

  //try to check simulation.....
  analogV = analogRead(sensor);
 
  return analogV;

}


//    check box buttons
void checkButtons(long cur_time) {

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

float readDATA(byte addr) {

  return (float)EEPROM.read(addr)/10.0;  
}

void saveDATA(byte addr, byte data) {
  
  EEPROM.write(addr, data);
  
}

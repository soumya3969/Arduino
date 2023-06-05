#include "DHT.h"
#define Type DHT11
#include <LiquidCrystal.h>   //7,8,6,5,4,3
int rs=7;
int en=8;
int d4=6;
int d5=5;
int d6=4;
int d7=3;
LiquidCrystal lcd(rs,en,d4,d5,d6,d7);
 
int sensePin=9;
DHT HT(sensePin,Type);
float humidity;
float tempC;
float tempF;
int setTime=5000;
int dt=1000;
 
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
HT.begin();
delay(setTime);
lcd.begin(16,2);
 
}
 
void loop() {
humidity=HT.readHumidity();
tempC=HT.readTemperature();
tempF=HT.readTemperature(true);
 
lcd.setCursor(0,0);
lcd.print("Temp F= ");
lcd.print(tempF);
lcd.setCursor(0,1);
lcd.print("Humidity= ");
lcd.print(humidity);
lcd.print(" %");
delay(500);
lcd.clear();
 
Serial.print("Humidity: ");
Serial.print(humidity);
Serial.print("% Temperature ");
Serial.print(tempC);
Serial.print(" C ");
Serial.print(tempF);
Serial.println(" F ");
 
}

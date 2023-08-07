#include <LiquidCrystal_I2C.h>
 
// initialize the LCD library with I2C address and LCD size
LiquidCrystal_I2C lcd (0x27, 16,2); 
 
void setup()
{
  Serial.begin(9600);
Serial.println ("Arduino Anemometer"); 
  // Initialize the LCD connected 
lcd. init ();
// Turn on the backlight on LCD. 
lcd. backlight ();
lcd.print ("CIRCUITSCHOOLS..");
lcd. setCursor (0, 1);
lcd.print ("ANEMOMETER");
lcd.clear();  
  delay(3000);
}
 
void loop()
{
  float sensorValue = analogRead(A0);
  Serial.print("Analog Value =");
  Serial.println(sensorValue);
 
  float voltage = (sensorValue / 1024) * 5; //Arduino ADC resolution 0-1023
  Serial.print("Voltage =");
  Serial.print(voltage);
  Serial.println(" V");
 
  float wind_speed = mapfloat(voltage, 0.4, 2, 0, 32.4);
  float speed_mph = ((wind_speed *3600)/1609.344);
  Serial.print("Wind Speed =");
  Serial.print(wind_speed);
  Serial.println("m/s");
  Serial.print(speed_mph);
  Serial.println("mph");
 
  lcd. setCursor (0, 0);
  lcd.print ("Wind Speed");
//Here cursor is placed on second line 
lcd. setCursor (0, 1);
lcd.print (wind_speed);
lcd.print ("m/s");
 
  Serial.println(" ");
  delay(300);
}
 
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



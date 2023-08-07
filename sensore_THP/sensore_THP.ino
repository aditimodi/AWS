#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1011.80)
#define buzzerPin 4
Adafruit_BME280 bme;

void setup() {
  pinMode(buzzerPin, OUTPUT);
	Serial.begin(9600);

	if (!bme.begin(0x76)) {
		Serial.println("Could not find a valid BME280 sensor, check wiring!");
		while (1);
	}
}

void loop() {
	Serial.print("Temperature = ");
	Serial.print(bme.readTemperature());
	Serial.println("*C");

	Serial.print("Pressure = ");
	Serial.print(bme.readPressure() / 100.0F);
	Serial.println("hPa");

	Serial.print("Approx. Altitude = ");
	Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
	Serial.println("m");

	Serial.print("Humidity = ");
	Serial.print(bme.readHumidity());
	Serial.println("%");

	Serial.println();

    if (bme.readTemperature() > 27.0) {
      digitalWrite(buzzerPin, HIGH); // turn the LED on (HIGH is the voltage level)
      delay(500);
      digitalWrite(buzzerPin, LOW); // turn the LED on (HIGH is the voltage level)
    }

	delay(1000);
}

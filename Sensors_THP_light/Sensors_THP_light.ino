#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>

#define SEALEVELPRESSURE_HPA (1011.80)
#define buzzerPin 4
Adafruit_BME280 bme;
BH1750 lightMeter;

void setup() {
  pinMode(buzzerPin, OUTPUT);
	Serial.begin(9600);

	if (!bme.begin(0x76)) {
		Serial.println("Could not find a valid BME280 sensor, check wiring!");
		while (1);
	}

  pinMode(buzzerPin, OUTPUT);

  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin();
  // On esp8266 you can select SCL and SDA pins using Wire.begin(D4, D3);
  // For Wemos / Lolin D1 Mini Pro and the Ambient Light shield use
  // Wire.begin(D2, D1);

  lightMeter.begin();

  Serial.println(F("BH1750 Test begin"));
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
  float lux = lightMeter.readLightLevel();
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
  delay(1000);

    if (lux > 1000.) {
      // digitalWrite(buzzerPin, HIGH); // turn the LED on (HIGH is the voltage level)
      tone(buzzerPin, 500, 500);

      delay(500);
      // digitalWrite(buzzerPin, LOW); // turn the LED on (HIGH is the voltage level)
    }

	delay(1000);
}

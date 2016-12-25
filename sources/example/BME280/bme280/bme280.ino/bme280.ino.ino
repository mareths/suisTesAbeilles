// Ajouter la librairie via Croquis/Inclure une librairie/Gerer les bibliothéques, et chercher bme280,
//    rajouter la librairie "Adafruit BME280"
// notez que l adresse du capteur est 0x76 alors que par defaut la lib va chercher 0x77 (CS sur vcc pour avoir cette adresse) 



#include <Adafruit_BME280.h>

#define BME_SCK A5
#define BME_MOSI A4

Adafruit_BME280 bme; // I2C

void setup() {
  Serial.begin(9600);
  Serial.println(F("BME280 test"));

  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

void loop() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
    delay(2000);
}

// Ajouter la librairie via Croquis/Inclure une librairie/Gerer les bibliothéques, et chercher ds18b20, 
//    rajouter la librairie "DallasTemperature"
// Ajouter la librairie via Croquis/Inclure une librairie/Gerer les bibliothéques, et chercher onewire,
//    rajouter la librairie "MAX31850 OneWire by Adafruit"

#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 12

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
DeviceAddress sondeNumero0, sondeNumero1;

// global variable for temp0 and temp1
float temp0 = 0;
float temp1 = 1;

void setup(void)
{
  // start serial port
  Serial.begin(9600);
  Serial.println("Dallas Temperature IC Control Library Demo");

  // Start up the library
  sensors.begin();
  
  // locate devices on the bus
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // search for devices on the bus and assign based on an index.
  if (!sensors.getAddress(sondeNumero0, 0)) Serial.println("Unable to find address for Device 0"); 
  if (!sensors.getAddress(sondeNumero1, 1)) Serial.println("Unable to find address for Device 1");

  // Sensibilty set at 0.0625°C
  sensors.setResolution(sondeNumero0, 12);
  sensors.setResolution(sondeNumero1, 12);

}

void loop(void)
{ 
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures();
  Serial.println("DONE");
  // delay for conversion due to sensibility (i don't know where we should put this delay...)
  delay(800);

  temp0 = sensors.getTempC(sondeNumero0);
  // delay for conversion due to sensibility (i don't know where we should put this delay...)
  delay(800);
  Serial.print("Sonde 0 : ");
  Serial.print(temp0);
  Serial.print(" ");
  temp1 = sensors.getTempC(sondeNumero1);
  // delay for conversion due to sensibility (i don't know where we should put this delay...)
  delay(800);
  Serial.print("Sonde 1 : ");
  Serial.println(temp1);

  delay(10000);
}

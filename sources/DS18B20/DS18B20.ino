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

  // show the addresses we found on the bus
  Serial.print("Device 0 Address: ");
  printAddress(sondeNumero0);
  Serial.println();

  Serial.print("Device 0 Alarms: ");
  printAlarms(sondeNumero0);
  Serial.println();
  
  Serial.print("Device 1 Address: ");
  printAddress(sondeNumero1);
  Serial.println();

  Serial.print("Device 1 Alarms: ");
  printAlarms(sondeNumero1);
  Serial.println();
  
  Serial.println("Setting alarm temps...");

  // alarm when temp is higher than -25C
  sensors.setHighAlarmTemp(sondeNumero0, -25);
  
  // alarm when temp is lower than 50C
  sensors.setLowAlarmTemp(sondeNumero0, 50);
  
  // alarm when temp is higher than -25C
  sensors.setHighAlarmTemp(sondeNumero1, -25);
  
  // alarn when temp is lower than 50C
  sensors.setLowAlarmTemp(sondeNumero1, 50);
  
  Serial.print("New Device 0 Alarms: ");
  printAlarms(sondeNumero0);
  Serial.println();
  
  Serial.print("New Device 1 Alarms: ");
  printAlarms(sondeNumero1);
  Serial.println();
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.print(DallasTemperature::toFahrenheit(tempC));
}

void printAlarms(uint8_t deviceAddress[])
{
  char temp;
  temp = sensors.getHighAlarmTemp(deviceAddress);
  Serial.print("High Alarm: ");
  Serial.print(temp, DEC);
  Serial.print("C/");
  Serial.print(DallasTemperature::toFahrenheit(temp));
  Serial.print("F | Low Alarm: ");
  temp = sensors.getLowAlarmTemp(deviceAddress);
  Serial.print(temp, DEC);
  Serial.print("C/");
  Serial.print(DallasTemperature::toFahrenheit(temp));
  Serial.print("F");
}

// main function to print information about a device
void printData(DeviceAddress deviceAddress)
{
  Serial.print("Device Address: ");
  printAddress(deviceAddress);
  Serial.print(" ");
  printTemperature(deviceAddress);
  Serial.println();
}

void checkAlarm(DeviceAddress deviceAddress)
{
  if (sensors.hasAlarm(deviceAddress))
  {
    Serial.print("ALARM: ");
    printData(deviceAddress);
  }
}

void loop(void)
{ 
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures();
  Serial.println("DONE");

  // Method 1:
  // check each address individually for an alarm condition
  checkAlarm(sondeNumero0);
  checkAlarm(sondeNumero1);
/*
  // Alternate method:
  // Search the bus and iterate through addresses of devices with alarms
  
  // space for the alarm device's address
  DeviceAddress alarmAddr;
  Serial.println("Searching for alarms...");
  
  // resetAlarmSearch() must be called before calling alarmSearch()
  sensors.resetAlarmSearch();
  
  // alarmSearch() returns 0 when there are no devices with alarms
  while (sensors.alarmSearch(alarmAddr))
  {
    Serial.print("ALARM: ");
    printData(alarmAddr);
  }
*/

}

#include <Wire.h>
#include <Bmp180.h>

Bmp180 bmp180;

void setup(){
 Serial.begin(9600);
 Wire.begin();
 bmp180.calibration();
}
void loop()
{
 // Collect data
 float temperature = bmp180.getTemperature(bmp180.readUT()); //MUST be called first
 float pressure = bmp180.getPressure(bmp180.readUP());
 float atm = bmp180.calcAtm(pressure);
 float altitude = bmp180.calcAltitude(pressure); //Uncompensated caculation - in Meters
 
 // Log the data
 Serial.print("Temperature: ");
 Serial.print(temperature, 2); //display 2 decimal places
 Serial.println("deg C");
 Serial.print("Pressure: ");
 Serial.print(pressure, 0); //whole number only.
 Serial.println(" Pa");
 Serial.print("Standard Atmosphere: ");
 Serial.println(atm, 4); //display 4 decimal places
 Serial.print("Altitude: ");
 Serial.print(altitude, 2); //display 2 decimal places
 Serial.println(" M");
 Serial.println();//line break
 
 //wait a second and get values again.
 delay(1000);
}

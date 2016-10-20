
// ---------------------------------------------------------------------
// Include
// ---------------------------------------------------------------------
#include <arm.h> //ATIM library for LoRaWAN connection
#include <SoftwareSerial.h> // More information at https://www.arduino.cc/en/Reference/SoftwareSerial
#include <Wire.h>
#include <Bmp180.h>

// Debug flag
bool debug = true;

// Create object from library
Bmp180 bmp180;

// We define the pins for the software Serial that will allows us to
// debug our code.
  SoftwareSerial mySerial(10, 11); // Pin 10 will work as RX and Pin 11 as TX


// ---------------------------------------------------------------------
// Global variables
// ---------------------------------------------------------------------
byte msgg[6];           // Store the data to be uploaded to Objeniou's Network
                      
int pressureBMP180;
int temperatureBMP180;

//Instance of  the class Arm
Arm Objenious; // Needed to make work the LoRaWAN module


// ---------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------
void setup()
{     
    // set the data rate for the SoftwareSerial port (Debug mode only)
    if (debug) {
      mySerial.begin(9600); 
      mySerial.println("Software serial test OK!"); 
      // If you see the message on your serial monitor it is working!
    }

    // BMP180 init
    Wire.begin();
    bmp180.calibration();

// ---------------------------------------------------------------------
// LoRaWAN module Init and configuration
// ---------------------------------------------------------------------
    delay(1000); // delay needed for the module to be ready to initialize.
    
    //Init of the LoRaWAN module - Red light if error, Green light if Ok 
    if(Objenious.Init(&Serial) != ARM_ERR_NONE)
    {
      if (debug) {
        mySerial.println("Network Error"); // Debug
      }
    }
    else
    {
      if (debug) {
        mySerial.println("Connected to Objenious"); // Debug
      }
    }

    // Configuration of the LoRaWAN module
    Objenious.SetMode(ARM_MODE_LORAWAN);

    Objenious.LwEnableRxWindows(true);
    Objenious.LwEnableTxAdaptiveSpeed(true);
    Objenious.LwEnableDutyCycle(true);
    Objenious.LwEnableTxAdaptiveChannel(true);
    Objenious.LwEnableRx2Adaptive(true);
    
    Objenious.LwEnableOtaa(true);
    
    //Apply the configuration to the module.

    Objenious.UpdateConfig();
    
    delay(8000); // delay needed for the module to connect to Objenious

    
// ---------------------------------------------------------------------
// Here starts your code :D
// ---------------------------------------------------------------------

    msgg[0]=7; // This byte will indicate to Objeniou's platform what kind
               // of sketch we are using anf hence how to decode the data:
               //   - 1 = Temperature data
               //   - 2 = Push button data
               //   - 3 = Window/Door open data
               //   - 7 = SuisTesAbeilles data
}


// ---------------------------------------------------------------------
// How the code works:
// The sensor is read by the "Thermistor" function and then is multiplied
// by 100 in order to avoid decimal number.
// The mySerial function will print the information on the virtual Serial
// so we can debug.
// The temperature data is an Int, hence 2bytes of data. This data is stored
// in the "msgg" buffer before being uploaded to Objeniou's platform. To do 
// that we need to copy byte by byte. Example:
// int temp = 2348 (23,48°C * 100) // example valule...
//
// dec  ->   Byte 1     Byte 2
// 2348 -> 0000 1001  0010 1100 (binary representation of 2348. http://www.exploringbinary.com/binary-converter/)
//
// Then we store the fisrt Byte in msgg[1] and the sencond byte in msgg[2]
// Objenious.Send will uoload the data to our LoRaNetwork.
// ---------------------------------------------------------------------

void loop()
{
  // Collect data
  collectData();

  // Put temperature in the msgg
  msgg[1] = (byte) (temperatureBMP180>>8);
  msgg[2] = (byte) temperatureBMP180;

  // Put temperature in the msgg
  msgg[3] = (byte) (pressureBMP180>>16);
  msgg[4] = (byte) (pressureBMP180>>8);
  msgg[5] = (byte) pressureBMP180;

  // For Debug
  if (debug) {
    logDebug();
  }

  Objenious.Send(msgg, sizeof(msgg));               // Send the temp to Objenious network        

  delay(20000);                                     // Send the temperature every 20 seconds
}

void collectData() {
  //BMP180
  temperatureBMP180 = (bmp180.getTemperature(bmp180.readUT())*100); //MUST be called first
  pressureBMP180 = bmp180.getPressure(bmp180.readUP());
}

void logDebug() {
  mySerial.print("Celsius: "); 
  mySerial.print(temperatureBMP180); 
  mySerial.print(" - "); 
  mySerial.print(msgg[1]); 
  mySerial.print(" "); 
  mySerial.println(msgg[2]); 

  mySerial.print("Pression: "); 
  mySerial.print(pressureBMP180); 
  mySerial.print(" - "); 
  mySerial.print(msgg[3]); 
  mySerial.print(" "); 
  mySerial.print(msgg[4]); 
  mySerial.print(" "); 
  mySerial.println(msgg[5]); 
}



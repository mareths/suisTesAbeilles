// Comment for the real life !
//#define DEBUG
// Uncomment number of sensor
//#define DS18B20_0 // 1 DS18B20
//#define DS18B20_1 // 2 DS18B20
//#define DS18B20_2 // 3 DS18B20
//#define DS18B20_3 // 4 DS18B20
#define DS18B20_4 // 5 DS18B20


// ---------------------------------------------------------------------
// Include
// ---------------------------------------------------------------------
#ifdef DEBUG
#include <SoftwareSerial.h> // More information at https://www.arduino.cc/en/Reference/SoftwareSerial
#endif

#include <arm.h> //ATIM library for LoRaWAN connection
#include <Wire.h> // for barometric sensor
// Ajouter la librairie BMP180 dans le repertoire lib
#include <Bmp180.h> // for barometric sensor
#include <avr/sleep.h> // for idle mode
#include <avr/power.h> // for idle mode
// Ajouter la librairie via Croquis/Inclure une librairie/Gerer les bibliothéques, et chercher dht22
#include "DHT.h" // for DHT22
// Ajouter la librairie HX711 dans le repertoire lib
#include "HX711.h" // for weight sensor
// Ajouter la librairie via Croquis/Inclure une librairie/Gerer les bibliothéques, et chercher onewire,
//    rajouter la librairie "MAX31850 OneWire by Adafruit"
#include <OneWire.h> // for DS18B20 sensor
// Ajouter la librairie via Croquis/Inclure une librairie/Gerer les bibliothéques, et chercher ds18b20, 
//    rajouter la librairie "DallasTemperature"
#include <DallasTemperature.h> // for DS18B20 sensor

// Idle mode
int nbMinuteTimeout = 2; // delay of mode idle
volatile int timer1=1; // count timer1 cycle before wakeup

// Barometric sensor
Bmp180 bmp180;

// Switch open roof sensor
const byte interruptPin = 3; // only one interrupt pin on the airboard

// DHT22
#define DHTPIN 13 // broche ou l'on a branche le capteur
#define DHTTYPE DHT22 // DHT 22 (AM2302)
DHT dht(DHTPIN, DHTTYPE);//déclaration du capteur

// Weight sensor
// OUT          - pin 11
// Clock (SDK)  - pin 10
HX711 scale(11, 10);

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 12

// No Weight sensor in DEBUG mode
#ifdef DEBUG
// We define the pins for the software Serial that will allows us to
// debug our code.
  SoftwareSerial mySerial(10, 11); // Pin 10 will work as RX and Pin 11 as TX
#endif

// ---------------------------------------------------------------------
// Global variables
// ---------------------------------------------------------------------
byte msgRoof[1];         // Message wich be send then roof is open 

////////////////////
// BMP180
int pressureBMP180;
int temperatureBMP180;

////////////////////
// DHT22
int humidityDHT22;
int temperatureDHT22;
int indexTemperatureDHT22;

////////////////////
// Weight sensor
long weight;

////////////////////
// DS18B20
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
// Lenght of message to Objenious platform depends of number of DS18B20 sensors number
#ifdef DS18B20_0
byte msgData[16];           // Store the data to be uploaded to Objeniou's Network
// arrays to hold device addresses
DeviceAddress sondeNumero0;
// global variable for temp0 and temp1
int temp0;
#endif
#ifdef DS18B20_1
byte msgData[18];           // Store the data to be uploaded to Objeniou's Network
// arrays to hold device addresses
DeviceAddress sondeNumero0, sondeNumero1;
// global variable for temp0 and temp1
int temp0, temp1;
#endif
#ifdef DS18B20_2
byte msgData[20];           // Store the data to be uploaded to Objeniou's Network
// arrays to hold device addresses
DeviceAddress sondeNumero0, sondeNumero1, sondeNumero2;
// global variable for temp0 and temp1
int temp0, temp1, temp2;
#endif
#ifdef DS18B20_3
byte msgData[22];           // Store the data to be uploaded to Objeniou's Network
// arrays to hold device addresses
DeviceAddress sondeNumero0, sondeNumero1, sondeNumero2, sondeNumero3;
// global variable for temp0 and temp1
int temp0, temp1, temp2, temp3;
#endif
#ifdef DS18B20_4
byte msgData[24];           // Store the data to be uploaded to Objeniou's Network
// arrays to hold device addresses
DeviceAddress sondeNumero0, sondeNumero1, sondeNumero2, sondeNumero3, sondeNumero4;
// global variable for temp0 and temp1
int temp0, temp1, temp2, temp3, temp4;
#endif

//Instance of  the class Arm
Arm Objenious; // Needed to make work the LoRaWAN module

// ---------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------
void setup()
{     
    // set the data rate for the SoftwareSerial port (Debug mode only)
#ifdef DEBUG
    mySerial.begin(9600); 
    mySerial.println("Software serial test OK!"); 
    // If you see the message on your serial monitor it is working!
#endif

// ---------------------------------------------------------------------
// LoRaWAN module Init and configuration
// ---------------------------------------------------------------------
    delay(1000); // delay needed for the module to be ready to initialize.
    
    /* Init of the LoRaWAN module */
    if(Objenious.Init(&Serial) != ARM_ERR_NONE)
    {
#ifdef DEBUG
        mySerial.println("Network Error"); // Debug
    }
    else
    {
        mySerial.println("Connected to Objenious"); // Debug
#endif
    }

    /* Configuration of the LoRaWAN module */
    Objenious.SetMode(ARM_MODE_LORAWAN);
    Objenious.LwEnableRxWindows(true);
    Objenious.LwEnableTxAdaptiveSpeed(true);
    Objenious.LwEnableDutyCycle(true);
    Objenious.LwEnableTxAdaptiveChannel(true);
    Objenious.LwEnableRx2Adaptive(true);
    Objenious.LwEnableOtaa(true);
    
    /* Apply the configuration to the module. */
    Objenious.UpdateConfig();
    
    delay(8000); // delay needed for the module to connect to Objenious

// ---------------------------------------------------------------------
// Configure the timer
// ---------------------------------------------------------------------
 
    /* Normal timer operation.*/
    TCCR1A = 0x00; 
    
    /* Clear the timer counter register.
    * You can pre-load this register with a value in order to 
    * reduce the timeout period, say if you wanted to wake up
    * ever 4.0 seconds exactly.
    */
    /* Define a 4 secondes for timer1 cycle, better to have an real number of cycle to have one minute,
    than the default value of 4.09 when TCCR1B is define at 5 */
    TCNT1=0x0FA0; 
    
    /* Configure the prescaler for 1:1024, giving us a 
    * timeout of 4.09 seconds.
    */
    TCCR1B = 0x05;
    
    /* Enable the timer overlow interrupt. */
    TIMSK1=0x01;

// ---------------------------------------------------------------------
// Configure the interruption for switch open roof sensor
// ---------------------------------------------------------------------

    pinMode(interruptPin, INPUT_PULLUP);
    /* Configure sensor to interrupt program or idle mode only when somebody open the roof */
    attachInterrupt(digitalPinToInterrupt(interruptPin), switchCapteurOuverture, RISING);
    delay(100);

// ---------------------------------------------------------------------
// Build the message wich be send then roof is open
// ---------------------------------------------------------------------

  // Build of the message to Objenious
  msgRoof[0] = 2; // This byte will indicate to Objeniou's platform what kind
               // of sketch we are using anf hence how to decode the data:
               //   - 1 = SuisTesAbeilles data
               //   - 2 = Switch open roof alarm

// ---------------------------------------------------------------------
// BMP180 init
// ---------------------------------------------------------------------

    Wire.begin();
    bmp180.calibration();

// ---------------------------------------------------------------------
// DHT22 init
// ---------------------------------------------------------------------

    dht.begin();

// ---------------------------------------------------------------------
// Weight sensor init
// ---------------------------------------------------------------------

  scale.set_scale(-24000.f);
  scale.tare();

// ---------------------------------------------------------------------
// DS18B20 sensor init
// ---------------------------------------------------------------------
  sensors.begin();

#if (defined DS18B20_0 || defined DS18B20_1 || defined DS18B20_2 || defined DS18B20_3 || defined DS18B20_4)
    // search for devices on the bus and assign based on an index.
  if (!sensors.getAddress(sondeNumero0, 0)) {
#ifdef DEBUG
   mySerial.println("Unable to find address for Device 0");
#endif
  } else {
  // Sensibilty set at 0.0625°C
    sensors.setResolution(sondeNumero0, 12);
  }
#endif

#if (defined DS18B20_1 || defined DS18B20_2 || defined DS18B20_3 || defined DS18B20_4)
  if (!sensors.getAddress(sondeNumero1, 1)) {
#ifdef DEBUG
    mySerial.println("Unable to find address for Device 1");
#endif
  } else {
  // Sensibilty set at 0.0625°C
    sensors.setResolution(sondeNumero1, 12);
  }
#endif

#if (defined DS18B20_2 || defined DS18B20_3 || defined DS18B20_4)
  if (!sensors.getAddress(sondeNumero2, 2)) {
#ifdef DEBUG
   mySerial.println("Unable to find address for Device 2");
#endif
  } else {
  // Sensibilty set at 0.0625°C
    sensors.setResolution(sondeNumero2, 12);
  }
#endif

#if (defined DS18B20_3 || defined DS18B20_4)
  if (!sensors.getAddress(sondeNumero3, 3)) {
#ifdef DEBUG
    mySerial.println("Unable to find address for Device 3");
#endif
  } else {
  // Sensibilty set at 0.0625°C
    sensors.setResolution(sondeNumero3, 12);
  }
#endif

#if (defined DS18B20_4)
  if (!sensors.getAddress(sondeNumero4, 4)) {
#ifdef DEBUG
   mySerial.println("Unable to find address for Device 4");
#endif
  } else {
  // Sensibilty set at 0.0625°C
    sensors.setResolution(sondeNumero4, 12);
  }
#endif

} // End of setup()


// ---------------------------------------------------------------------
// How the code works:
// The temperature data is an Int, hence 2bytes of data. This data is stored
// in the "msgData" buffer before being uploaded to Objeniou's platform. To do 
// that we need to copy byte by byte. Example:
// int temperature = 2348 (23,48°C * 100) // example valule...
//
// dec  ->   Byte 1     Byte 2
// 2348 -> 0000 1001  0010 1100 (binary representation of 2348.
//
// Then we store the fisrt Byte in msgData[1] and the sencond byte in msgData[2]
// Objenious.Send will uoload the data to our LoRaNetwork.
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// Mapping of the message:
// .....................................................................
// msgData [0] : 1 = SuisTesAbeilles data
//  msgData [1] : 1st byte of temperature from the BMP180
//  msgData [2] : 2nd byte of temperature from the BMP180
//  msgData [3] : 1st byte of pression from the BMP180
//  msgData [4] : 2nd byte of pression from the BMP180
//  msgData [5] : 3rd byte of pression from the BMP180
//  msgData [6] : 1st byte of humidity from the DHT22
//  msgData [7] : 2nd byte of humidity from the DHT22
//  msgData [8] : 1st byte of temperature from the DHT22
//  msgData [9] : 2nd byte of temperature from the DHT22
//  msgData [10] : 1st byte of the index temperature from the DHT22
//  msgData [11] : 2nd byte of the index temperature from the DHT22
//  msgData [12] : 1st byte of the weight from the weight sensor
//  msgData [13] : 2nd byte of the weight from the weight sensor
//  msgData [14] : 1st byte of the temperature from the DS18B20 0
//  msgData [15] : 2nd byte of the temperature from the DS18B20 0
//  msgData [16] : 1st byte of the temperature from the DS18B20 1
//  msgData [17] : 2nd byte of the temperature from the DS18B20 1
//  msgData [18] : 1st byte of the temperature from the DS18B20 2
//  msgData [19] : 2nd byte of the temperature from the DS18B20 2
//  msgData [20] : 1st byte of the temperature from the DS18B20 3
//  msgData [21] : 2nd byte of the temperature from the DS18B20 3
//  msgData [22] : 1st byte of the temperature from the DS18B20 4
//  msgData [23] : 2nd byte of the temperature from the DS18B20 4
// .....................................................................
// msgRoof [0] : 2 = Alarm of switch open roof 
// ---------------------------------------------------------------------

void loop()
{
#ifdef DEBUG
  mySerial.println("Go to sleep");
#endif

  // Management of the idle mode
  sleepNow();     // Pass to idle mode for 4s

  // 15 timer1 cycle to have one minute
  if (timer1 > 15*nbMinuteTimeout) {
#ifdef DEBUG
    mySerial.println("Wake up !");
#endif

    // Collect the data from the sensor
#ifdef DEBUG
    mySerial.println("Collecting data...");
#endif
    collectData();
  
#ifdef DEBUG
    mySerial.println("Building msgData...");
#endif
    // Build of the message to Objenious
    msgData[0] = 1; // This byte will indicate to Objeniou's platform what kind
                 // of sketch we are using anf hence how to decode the data:
                 //   - 1 = SuisTesAbeilles data
                 //   - 2 = Switch open roof alarm
  
    // Put BMP180 temperature in the msgData
    msgData[1] = (byte) (temperatureBMP180>>8);
    msgData[2] = (byte) temperatureBMP180;
  
    // Put BMP180 temperature in the msgData
    msgData[3] = (byte) (pressureBMP180>>16);
    msgData[4] = (byte) (pressureBMP180>>8);
    msgData[5] = (byte) pressureBMP180;
  
    // Put DHT22 humidity in the msgData
    msgData[6] = (byte) (humidityDHT22>>8);
    msgData[7] = (byte) humidityDHT22;
  
    // Put DHT22 temperature in the msgData
    msgData[8] = (byte) (temperatureDHT22>>8);
    msgData[9] = (byte) temperatureDHT22;
  
    // Put DHT22 index of temperature in the msgData
    msgData[10] = (byte) (indexTemperatureDHT22>>8);
    msgData[11] = (byte) indexTemperatureDHT22;
  
    // Put weight in the msgData
    msgData[12] = (byte) (weight>>8);
    msgData[13] = (byte) weight;
  
#if (defined DS18B20_0 || defined DS18B20_1 || defined DS18B20_2 || defined DS18B20_3 || defined DS18B20_4)
    // Put temp0 in the msgData
    msgData[14] = (byte) (temp0>>8);
    msgData[15] = (byte) temp0;
#endif
  
#if (defined DS18B20_1 || defined DS18B20_2 || defined DS18B20_3 || defined DS18B20_4)
    // Put temp1 in the msgData
    msgData[16] = (byte) (temp1>>8);
    msgData[17] = (byte) temp1;
#endif
  
#if (defined DS18B20_2 || defined DS18B20_3 || defined DS18B20_4)
    // Put temp2 in the msgData
    msgData[18] = (byte) (temp2>>8);
    msgData[19] = (byte) temp2;
#endif
  
#if (defined DS18B20_3 || defined DS18B20_4)
    // Put temp3 in the msgData
    msgData[20] = (byte) (temp3>>8);
    msgData[21] = (byte) temp3;
#endif
  
#if (defined DS18B20_4)
    // Put temp4 in the msgData
    msgData[22] = (byte) (temp4>>8);
    msgData[23] = (byte) temp4;
#endif
  
#ifdef DEBUG
    logDebugData();
#endif
  
#ifdef DEBUG
    mySerial.println("Sending msgData...");
#endif
    Objenious.Send(msgData, sizeof(msgData));               // Send the collected data to Objenious network        
  
#ifdef DEBUG
    mySerial.println("Done");
#endif
    timer1 = 1; // reinit of the timer1
  }
  
} // End of loop()

/***************************************************
 *  Name:        ISR(TIMER1_OVF_vect)
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  TIMER1_OVF_vect.
 *
 *  Description: Timer1 Overflow interrupt.
 *
 ***************************************************/
ISR(TIMER1_OVF_vect)
{
  timer1 += 1;
#ifdef DEBUG
  mySerial.println("Timer++");
#endif
} // End of ISR(TIMER1_OVF_vect)

/***************************************************
 *  Name:        collectData()
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Call method to collect data from sensor.
 *
 ***************************************************/
void collectData() {
  // BMP180
  temperatureBMP180 = (bmp180.getTemperature(bmp180.readUT())*100); // collect the temperature in Celsius,
                                                                    // multiple by 100 to have an integer value
  pressureBMP180 = bmp180.getPressure(bmp180.readUP()); // collect the pressure
  
  // DHT22
  humidityDHT22 = (dht.readHumidity()*100); // collect hygrometry, multiple by 100 to have an integer value
  temperatureDHT22 = (dht.readTemperature()*100); // collect the temperature in Celsius,
                                            // multiple by 100 to have an integer value
 
  if (isnan(humidityDHT22) || isnan(temperatureDHT22)) // check if the collect is OK
  {
#ifdef DEBUG
    mySerial.println("Failed to read from DHT sensor!");
#endif
    humidityDHT22 = 0;
    temperatureDHT22 = 0;
  }

  indexTemperatureDHT22 = (dht.computeHeatIndex(temperatureDHT22, humidityDHT22, false)*100); // calcul the index
                                               // of temperature in Celsius, multiple by 100 to have an integer value

  // Weight sensor
  weight = scale.read_average(10);

  // DS18B20
  sensors.requestTemperatures();
  // delay for conversion due to sensibility (i don't know where we should put this delay...)
  delay(800);

#if (defined DS18B20_0 || defined DS18B20_1 || defined DS18B20_2 || defined DS18B20_3 || defined DS18B20_4)
  temp0 = (sensors.getTempC(sondeNumero0)*100);
  // delay for conversion due to sensibility (i don't know where we should put this delay...)
  delay(800);
#endif
#if (defined DS18B20_1 || defined DS18B20_2 || defined DS18B20_3 || defined DS18B20_4)
  temp1 = (sensors.getTempC(sondeNumero1)*100);
  // delay for conversion due to sensibility (i don't know where we should put this delay...)
  delay(800);
#endif
#if (defined DS18B20_2 || defined DS18B20_3 || defined DS18B20_4)
  temp2 = (sensors.getTempC(sondeNumero2)*100);
  // delay for conversion due to sensibility (i don't know where we should put this delay...)
  delay(800);
#endif
#if (defined DS18B20_3 || defined DS18B20_4)
  temp3 = (sensors.getTempC(sondeNumero3)*100);
  // delay for conversion due to sensibility (i don't know where we should put this delay...)
  delay(800);
#endif
#if (defined DS18B20_4)
  temp4 = (sensors.getTempC(sondeNumero4)*100);
  // delay for conversion due to sensibility (i don't know where we should put this delay...)
  delay(800);
#endif

  
} // End of collectData()

/***************************************************
 *  Name:        sleepNow()
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Manage the IDLE MODE.
 *
 ***************************************************/
void sleepNow() {
  /* Configure sensor to interrupt program or idle mode only when somebody open the roof */
  attachInterrupt(digitalPinToInterrupt(interruptPin), switchCapteurOuverture, RISING); 
  delay(100);
  set_sleep_mode(SLEEP_MODE_IDLE);   // MODE_IDLE to be wake up by timer1
  sleep_enable();          // enables the sleep bit in the mcucr register

  /* Weight sensor shutdown */
  scale.power_down();

  /* Disable all of the unused peripherals. This will reduce power
   * consumption further and, more importantly, some of these
   * peripherals may generate interrupts that will wake our Arduino from
   * sleep!
   */
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer2_disable();
  power_twi_disable();
    
  sleep_mode();            // here the device is actually put to sleep!!  
  
  // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP  
  sleep_disable();         // first thing after waking from sleep: disable sleep...

  /* Re-enable the peripherals. */
  power_all_enable();

  /* Wake up weight sensor */
  scale.power_up();

} // End of sleepNow()

/***************************************************
 *  Name:        switchCapteurOuverture()
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Send a alarm msg to Objenious plateform with id = 2
 *
 ***************************************************/
void switchCapteurOuverture() {
  sleep_disable();         // first thing after waking from sleep: disable sleep...
  /* Re-enable the peripherals. */
  power_all_enable();
  detachInterrupt(digitalPinToInterrupt(interruptPin));
#ifdef DEBUG
  mySerial.println("Roof is open !");
  mySerial.println("Sending msgRoof...");
#endif
  Objenious.Send(msgRoof, sizeof(msgRoof));               // Send the roof alarm to Objenious network        
#ifdef DEBUG
  mySerial.println("Done");
#endif

  delay(100);
  attachInterrupt(digitalPinToInterrupt(interruptPin), switchCapteurOuverture, RISING);
} // End of switchCapteurOuverture()

#ifdef DEBUG
/***************************************************
 *  Name:        logDebugData()
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Display collected data before transmit to Objenious
 *
 ***************************************************/
void logDebugData() {
  mySerial.print("BMP180 - Temperature: "); 
  mySerial.print(temperatureBMP180); 
  mySerial.print(" - "); 
  mySerial.print(msgData[1]); 
  mySerial.print(" "); 
  mySerial.println(msgData[2]); 

  mySerial.print("BMP180 - Pressure: "); 
  mySerial.print(pressureBMP180); 
  mySerial.print(" - "); 
  mySerial.print(msgData[3]); 
  mySerial.print(" "); 
  mySerial.print(msgData[4]); 
  mySerial.print(" "); 
  mySerial.println(msgData[5]); 

  mySerial.print("DHT22 - Humidity: "); 
  mySerial.print(humidityDHT22); 
  mySerial.print(" - "); 
  mySerial.print(msgData[6]); 
  mySerial.print(" "); 
  mySerial.println(msgData[7]); 

  mySerial.print("DHT22 - Temperature: "); 
  mySerial.print(temperatureDHT22); 
  mySerial.print(" - "); 
  mySerial.print(msgData[8]); 
  mySerial.print(" "); 
  mySerial.println(msgData[9]);

  mySerial.print("DHT22 - Index of Temperature: "); 
  mySerial.print(indexTemperatureDHT22); 
  mySerial.print(" - "); 
  mySerial.print(msgData[10]); 
  mySerial.print(" "); 
  mySerial.println(msgData[11]);

  mySerial.print("Weight sensor - Weight: "); 
  mySerial.print(weight); 
  mySerial.print(" - "); 
  mySerial.print(msgData[12]); 
  mySerial.print(" "); 
  mySerial.println(msgData[13]);

#if (defined DS18B20_0 || defined DS18B20_1 || defined DS18B20_2 || defined DS18B20_3 || defined DS18B20_4)
  mySerial.print("DS18B20 sensor - temp0: "); 
  mySerial.print(temp0); 
  mySerial.print(" - "); 
  mySerial.print(msgData[14]); 
  mySerial.print(" "); 
  mySerial.println(msgData[15]);
#endif

#if (defined DS18B20_1 || defined DS18B20_2 || defined DS18B20_3 || defined DS18B20_4)
  mySerial.print("DS18B20 sensor - temp1: "); 
  mySerial.print(temp1); 
  mySerial.print(" - "); 
  mySerial.print(msgData[16]); 
  mySerial.print(" "); 
  mySerial.println(msgData[17]);
#endif

#if (defined DS18B20_2 || defined DS18B20_3 || defined DS18B20_4)
  mySerial.print("DS18B20 sensor - temp2: "); 
  mySerial.print(temp2); 
  mySerial.print(" - "); 
  mySerial.print(msgData[18]); 
  mySerial.print(" "); 
  mySerial.println(msgData[19]);
#endif

#if (defined DS18B20_3 || defined DS18B20_4)
  mySerial.print("DS18B20 sensor - temp3: "); 
  mySerial.print(temp3); 
  mySerial.print(" - "); 
  mySerial.print(msgData[20]); 
  mySerial.print(" "); 
  mySerial.println(msgData[21]);
#endif

#if (defined DS18B20_4)
  mySerial.print("DS18B20 sensor - temp4: "); 
  mySerial.print(temp4); 
  mySerial.print(" - "); 
  mySerial.print(msgData[22]); 
  mySerial.print(" "); 
  mySerial.println(msgData[23]);
#endif


} // End of logDebugData()
#endif



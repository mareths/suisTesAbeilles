#include <RF24.h>
#include <SPI.h>
#include <DHT.h>

// The DHT data line is connected to pin 2 on the Arduino
#define DHTPIN 2

// Leave as is if you're using the DHT22. Change if not.
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

int txbuffer[3] = {1, 0, 0};
int temperature;
int humidite;

RF24 radio(9, 10);
byte addresses[][6] = {"1Node","2Node","3Node"};

void setup(void)
{
// Set up the Serial Monitor
  Serial.begin(9600);

// Initialize all radio related modules
  radio.begin();
// Set the PA Level low to prevent power supply related issues since this is a
// getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW); //puissance maximum
  radio.setChannel(0x20); //canal 32
  radio.setDataRate(RF24_1MBPS);
  radio.openWritingPipe(addresses[2]);
//Enable dynamically-sized payloads.
//This way you don't always have to send large packets just to send them once in a while. This enables dynamic payloads on ALL pipes.
  radio.enableDynamicPayloads();
//Enable or disable auto-acknowlede packets.
//This is enabled by default, so it's only needed if you want to turn it off for some reason.
//  radio.setAutoAck(1);
//Set the number and delay of retries upon failed submit.
//  radio.setRetries(15,15);

// Initialize the DHT library
  dht.begin();
  
}

void loop() {

  temperature = dht.readTemperature()*10;
  humidite = dht.readHumidity()*10;
  txbuffer[1] = temperature;
  txbuffer[2] = humidite;
  Serial.println(humidite);
  Serial.println(temperature);

    if (radio.write(&txbuffer, sizeof(txbuffer))) {
      Serial.print("Message sent\n"); 
    } else {
      Serial.print("Could not send message\n"); 
    }

  delay(1000);
}

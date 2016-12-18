#include <RF24.h>
#include <SPI.h>

RF24 radio(9,10);                // nRF24L01(+) radio attached using Getting Started board 

int rxbuffer[3] = {0, 0, 0};
int temperature;
int humidite;

byte addresses[][6] = {"1Node","2Node","3Node"};

void setup()
{
  Serial.begin(9600);
 
  radio.begin();
// Set the PA Level low to prevent power supply related issues since this is a
// getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
radio.setPALevel(RF24_PA_LOW);
radio.setChannel(0x20); //canal 32
radio.setDataRate(RF24_1MBPS);
// Open a writing and reading pipe on each radio, with opposite addresses
// for 3 modules, 
radio.openReadingPipe(1, addresses[0]);
radio.openReadingPipe(2, addresses[1]);
radio.openReadingPipe(3, addresses[2]);
//Enable dynamically-sized payloads.
//This way you don't always have to send large packets just to send them once in a while. This enables dynamic payloads on ALL pipes.
radio.enableDynamicPayloads();
//Enable or disable auto-acknowlede packets.
//This is enabled by default, so it's only needed if you want to turn it off for some reason.
//radio.setAutoAck(1);
//Set the number and delay of retries upon failed submit.
//radio.setRetries(15,15);
//Leave low-power mode - making radio more responsive.
//radio.powerUp();
// Start the radio listening for data
radio.startListening();
}

void loop(){

while ( radio.available() ) {     // Is there anything ready for us?

    radio.read(&rxbuffer,sizeof(rxbuffer));
    int capteur = (int)rxbuffer[0];
    Serial.print("Capteur = ");
    Serial.println(capteur);
    temperature = rxbuffer[1];
    Serial.print("Temperature = ");
    Serial.println(temperature*.1);
    humidite = rxbuffer[2];
    Serial.print("Humidite = ");
    Serial.println(humidite*.1);
    delay(1000);
  }
}


#include <avr/sleep.h>
#include <avr/power.h>

const byte interruptPin = 3;
int nbMinuteTimeout = 2;
volatile int timer=1;

void setup() {
  Serial.begin(9600);
  Serial.println("Start");

  /*** Configure the timer.***/
  
  /* Normal timer operation.*/
  TCCR1A = 0x00; 
  
  /* Clear the timer counter register.
   * You can pre-load this register with a value in order to 
   * reduce the timeout period, say if you wanted to wake up
   * ever 4.0 seconds exactly.
   */
   /* Valeur pour 4 secondes */
  TCNT1=0x0FA0; 
  
  /* Configure the prescaler for 1:1024, giving us a 
   * timeout of 4.09 seconds.
   */
  TCCR1B = 0x05;
  
  /* Enable the timer overlow interrupt. */
  TIMSK1=0x01;

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), switchCapteurOuverture, RISING);
  delay(100);
}

/***************************************************
 *  Name:        ISR(TIMER1_OVF_vect)
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Timer1 Overflow interrupt.
 *
 ***************************************************/
ISR(TIMER1_OVF_vect)
{
  timer += 1;
  Serial.println("Timer++");
}

void loop() {
  Serial.println("Je m'endors");
  sleepNow();     // sleep function called here

  //cycle d'une minute
  if (timer > 15*nbMinuteTimeout) {
    Serial.println("Je me reveille");
    delay(5000);
    Serial.println("Je collecte des donnes");
    delay(5000);
    Serial.println("Je les envoie au module Lora");
    delay(5000);
    Serial.println("J'ai fini");
    timer = 1;
  }
}

void sleepNow() {  
  attachInterrupt(digitalPinToInterrupt(interruptPin), switchCapteurOuverture, RISING);
  delay(100);
  set_sleep_mode(SLEEP_MODE_IDLE);   // sleep mode is set here  
  sleep_enable();          // enables the sleep bit in the mcucr register

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
} 


void switchCapteurOuverture() {
  sleep_disable();         // first thing after waking from sleep: disable sleep...
  /* Re-enable the peripherals. */
  power_all_enable();
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  Serial.println("Ca s'ouvre");
  delay(100);
  attachInterrupt(digitalPinToInterrupt(interruptPin), switchCapteurOuverture, RISING);
}

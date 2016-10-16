#include "HX711.h"

// Entree digitale
// OUT	        - pin 11
// Clock (SDK)	- pin 10

HX711 scale(11, 10);

void setup() {
// pour suivre sur la sortie (a desactiver...?)  
  Serial.begin(38400);
// on va tarer la balance à l'initialisation du prog  (bin oui, comme toute balance)
// avec la valeur trouvee selon la methode de la lib (differente pour chacun des modeles de celllule)
  scale.set_scale(-24000.f);
  scale.tare();
}

void loop() {
// on prend 10 mesure et on fait la moyenne
  Serial.println(scale.get_units(10), 1);
// on met en sommeil l'ampli
  scale.power_down();
// en sommeil pendant (en ms)
  delay(5000);
// on reveille pour prendre les mesures
  scale.power_up();
}

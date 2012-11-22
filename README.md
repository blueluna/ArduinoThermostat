# Arduino Thermostat

A thermostat built and written with Arduino.

Sorry, Swedish for now.

Arduino-baserad termostat.

## Komponenter

### Temperatursensorer

 * DS18B20, 1-wire sensor med 0.5 grads noggranhet. Upp till 12-bitar
   upplösning. Temperaturområde -55 - 125 grader. ~50 SEK.
 * LM35DZ, Sensor som ger spänning. 1 grads noggranhet. Temperaturområde 0 -
   100 grader (mer med negativ matning). ~16 SEK.
 * RHT03, Temperatur och fuktighetssensor med 1-wire liknande protokoll. 0.5
   graders noggranhet. 0.1 grads upplösning. Temperaturområde -40 - 80 grader.
   ~10 USD.
 * LM75B, I2C sensor. ~8 SEK

### Relä

 * Omron G6D-1A-ASI 5VDC

### Rotationsenkoder / Potentiometer

Jag antar att fördelen med rotationsenkodern är att man inte har ett "felaktigt"
tillstånd efter omstart.

 * Rotationsenkoder, 24 pulser per varv,
   http://www.electrokit.com/rotationsenkoder-24-p-v-vertikal.43274
 * Rotationsenkoder, 24 pulser per varv,
   http://www.electrokit.com/rotationsenkoder-24p-v-vertikal-led-rod-gron-switch.48860

### Display

 * 7-segment display, 4 tecken,
   http://www.electrokit.com/seriell-display-7segment-gron.50300

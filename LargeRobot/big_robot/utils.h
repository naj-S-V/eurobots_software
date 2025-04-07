#ifndef UTILS_H
#define UTILS_H
#include <Arduino.h>

void test();

struct Movement{
  float timeDeplacement;
  void (*movement)();
};

/**
 * @brief Structure pour un capteur ultrason.
 *
 * Contient les informations de pin, la distance mesurée et un booléen indiquant si un obstacle est
 * détecté à proximité.
 */
struct UltrasonicSensor {
  int trigPin;    ///< Pin de déclenchement
  int echoPin;    ///< Pin d'écho
  int distance;   ///< Distance mesurée en cm
  bool isNear;    ///< Indique si un obstacle est à moins de 15 cm
};

// Test modification
#endif
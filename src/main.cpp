#include <Arduino.h>

void setup(void)
{
  // config led
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop(void)
{
  // blink led
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(700);
}

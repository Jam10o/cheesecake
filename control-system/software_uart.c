//credit to WhatEverFloatsYourBoat and Helen Harman for this code
#include "Arduino.h"
// bits per second, measured in microseconds.
#define HALF_BIT_4800 104;
#define FULL_BIT_4800 208;
unsigned char uart_rx(int pin, int baud) {
  unsigned char rec = 0;
  // double pause = 0.0;
  // double half_pause = 0.0;
  int i = 0;
  // this is a bit pointless for now. we're only reading the GPS at 4800bps.
  // but might want to read it quicker in future...
  /* switch(baud) {
   case 4800:
   default:
   pause = FULL_BIT_4800;
   half_pause = HALF_BIT_4800;
   break;
   } */
  double pause = FULL_BIT_4800;
  double half_pause = HALF_BIT_4800;
start_again:
  rec = 0;
  while(digitalRead(pin) == LOW); // wait for line to be high
  while(digitalRead(pin) == HIGH); // wait for line to fall (start bit)
  delayMicroseconds(half_pause);
  if(digitalRead(pin) != LOW) goto start_again;
  delayMicroseconds(pause);
  if(digitalRead(pin) == HIGH) rec += 1;
  delayMicroseconds(pause);
  if(digitalRead(pin) == HIGH) rec += 2;
  delayMicroseconds(pause);
  if(digitalRead(pin) == HIGH) rec += 4;
  delayMicroseconds(pause);
  if(digitalRead(pin) == HIGH) rec += 8;
  delayMicroseconds(pause);
  if(digitalRead(pin) == HIGH) rec += 16;
  delayMicroseconds(pause);
  if(digitalRead(pin) == HIGH) rec += 32;
  delayMicroseconds(pause);
  if(digitalRead(pin) == HIGH) rec += 64;
  delayMicroseconds(pause);
  if(digitalRead(pin) == HIGH) rec += 128;
  delayMicroseconds(pause);
  if(digitalRead(pin) != HIGH) goto start_again;
  return rec;
}
void uart_tx(char data, int baud, int pin) {
  /* double pause = 0;
   switch(baud) {
   case 4800:
   default:
   pause = FULL_BIT_4800;
   break;
   } */
  double pause = FULL_BIT_4800;
  digitalWrite(pin, LOW); // transmit start bit
  delayMicroseconds(pause);
  if(data & 1) digitalWrite(pin, HIGH);
  else digitalWrite(pin, LOW);
  delayMicroseconds(pause);
  if(data & 2) digitalWrite(pin, HIGH);
  else digitalWrite(pin, LOW);
  delayMicroseconds(pause);
  if(data & 4) digitalWrite(pin, HIGH);
  else digitalWrite(pin, LOW);
  delayMicroseconds(pause);
  if(data & 8) digitalWrite(pin, HIGH);
  else digitalWrite(pin, LOW);
  delayMicroseconds(pause);
  if(data & 16) digitalWrite(pin, HIGH);
  else digitalWrite(pin, LOW);
  delayMicroseconds(pause);
  if(data & 32) digitalWrite(pin, HIGH);
  else digitalWrite(pin, LOW);
  delayMicroseconds(pause);
  if(data & 64) digitalWrite(pin, HIGH);
  else digitalWrite(pin, LOW);
  delayMicroseconds(pause);
  if(data & 128) digitalWrite(pin, HIGH);
  else digitalWrite(pin, LOW);
  delayMicroseconds(pause);
  digitalWrite(pin, HIGH); // transmit stop bit
  delayMicroseconds(pause);
}
void uart_str(char *str, int baud, int pin) {
  int i;
  for(i=0; i<strlen(str); i++) {
    uart_tx(str[i], baud, pin);
  }
}


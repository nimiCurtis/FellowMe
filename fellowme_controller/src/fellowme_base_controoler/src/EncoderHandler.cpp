#include "EncoderHandler.h"

EncoderHandler::EncoderHandler(int pinA, int pinB) : encoder(pinA, pinB) {
  previousPosition = 0;
  currentPosition = 0;
}

void EncoderHandler::readEncoder() {
  currentPosition = encoder.read();
  if (currentPosition != previousPosition) {
    previousPosition = currentPosition;
  }
}

long EncoderHandler::getCurrentPosition() {
  return currentPosition;
}

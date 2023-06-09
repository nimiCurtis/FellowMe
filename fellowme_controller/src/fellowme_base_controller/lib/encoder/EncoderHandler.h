#ifndef ENCODER_HANDLER_H
#define ENCODER_HANDLER_H

#include <Encoder.h>

class EncoderHandler {
public:
  long previousPosition;
  long currentPosition;
  EncoderHandler(int pinA, int pinB);
  void readEncoder(long& currentPos, long& previousPos);
private:
  Encoder encoder;
};

#endif  // ENCODER_HANDLER_H

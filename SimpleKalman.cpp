
#include "SimpleKalman.h"


//Constructor
SimpleKalman::SimpleKalman() {
  _lastPosition = 0;
  _lastVelocity = 0;
  _position = 0;
  _velocity = 0;

  _processErrorQ = 0.03;
  _sensorErrorR = 0.1;

  _P[0][0] = 0.5;
  _P[0][1] = 0;
  _P[1][0] = 0;
  _P[1][1] = 0.5;

};


//Setters ---------------------


#include "SimpleKalman.h"


//Constructor -------------------
SimpleKalman::SimpleKalman() {
  _lastPosition = 0;
  _lastVelocity = 0;
  _position = 0;
  _velocity = 0;
  _estimatedPosition = 0;
  _estimatedVelocity = 0;

  _processErrorQ = 0.03;
  _sensorErrorR = 0.1;
  _translationH = 1;

  _P[0][0] = 0.5;
  _P[0][1] = 0;
  _P[1][0] = 0;
  _P[1][1] = 0.5;

};


//Setters ---------------------

void SimpleKalman::setPosition(double pos) {
  this->_position = pos;
};

void SimpleKalman::setVelocity(double velocity){
  this->_velocity = velocity;
};

void SimpleKalman::setEstimatedCovariance(float cov){
  this->_P[0][0] = cov;
  this->_P[1][1] = cov;
};

void SimpleKalman::setProcessError(float err_q){
  this->_processErrorQ = err_q;
};

void SimpleKalman::setSensorError(float err_r){
  this->_sensorErrorR = err_r;
};

void SimpleKalman::setTranslation(float h){
  this->_translationH = h;
};


//Getters ----------------------------

double SimpleKalman::getPosition(){
  return this->_estimatedPosition;
};

double SimpleKalman::getVelocity(){
  return this->_estimatedVelocity;
};


//Calculations ---------------------

void SimpleKalman::predictEstimate(double u, double dt){
  _lastPosition = _position;
  _lastVelocity = _velocity;
  //Predict xhat
  _position = _lastPosition + _lastVelocity*dt + 0.5*dt*dt*u;
  _velocity = _lastVelocity + dt*u;

  //Update Covariance Matrix
  // Pn = A(Pn-1)AT + Q
  //[1 dt][p00 p01][1  0]
  //[0 1 ][p10 p11][dt 1]
  _P[0][0] = _P[0][0] + _P[1][0]*dt + _P[0][1]*dt + _P[1][1]*dt*dt + _processErrorQ;
  _P[0][1] = _P[1][1]*dt + _P[0][1];
  _P[1][0] = _P[1][0] + _P[1][1]*dt;
  _P[1][1] = _P[1][1] + _processErrorQ;
};


void SimpleKalman::updateEstimate(double pos, double velocity, double dt){

  //Observation Step --------------------------
  double _innovationPosition = pos - _position;
  double _innovationVelocity = velocity -_velocity;

  double _S[2][2] = {{0,0},{0,0}};
  _S[0][0] = _P[0][0] + _sensorErrorR;
  _S[0][1] = _P[0][1];
  _S[1][0] = _P[1][0];
  _S[1][1] = _P[1][1] + _sensorErrorR;

  //Kalman Gain
  double _K[2][2] = {{0,0},{0,0}};
  _K[0,0] = P[0][0]*_

};

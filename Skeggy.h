#ifndef SKEGGY_H
#define SKEGGY_H

// a1-left, a2-right;

void overturn();
void mpuStart();
int ultrasonicSensorStart(int trigPin, int echoPin);
void ladder();
void goLadder();
void upLadder();
void ObliqueUp();
void ObliqueDown();
void ObliqueUpFast();
void Autoline();
void stopMotors();
#endif
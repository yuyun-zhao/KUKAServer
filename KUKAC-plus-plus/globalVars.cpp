#include "stdafx.h"
#include "globalVars.h"

double KUKA::EEFPos_[6] = { 0.0 };
double KUKA::ExternalTorque_[7] = { 0.0 };
double KUKA::jPos_[7] = { 0.0 };
double KUKA::EEFForce_[3] = { 0.0 };
double KUKA::MeasuredTorque_[7] = { 0.0 };
double KUKA::EEFMoment_[3] = { 0.0 };
bool KUKA::thread_beginflag_ = false;
bool KUKA::endflag_ = false;
bool KUKA::handguiding_endflag_ = false;
/*
Copyright (c) 2013, Drexel University, iSchool, Applied Informatics Group
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "HuboMotor.h"
#include <string>
#include <iostream>

HuboMotor::HuboMotor(){

    //NEW_DATA
    currGoal = 0;
	interStep = 0;
	interVel = .3; //Default 1/3 of a radian per second.

	currVel = 0;
	currPos = 0;
	currCurrent = 0;
	currTemp = 0;
	frequency = 0;

	enabled = false;
	homed = false;
	zeroed = false;

	buffer = NULL;
	i = 0;
}

HuboMotor::HuboMotor(long mpos1, long mpos2, long kp, long kd, long ki,
                     long dz, long off, long hlim, long hld, long hv1,
                     long hv2, long hma, long sm, long ers, long as, long md,
                     long v_max, long a_max, long jam_lim, long jamd,
                     long pwm_lim, long i_err, long b_err){
    this->mpos1 = mpos1;
    this->mpos2 = mpos2;
    this->Kp = kp;
    this->Kd = kd;
    this->Ki = ki;
    this->dz = dz;
    this->off = off;
    this->hlim = hlim;
    this->hld = hld;
    this->hv1 = hv1;
    this->hv2 = hv2;
    this->hma = hma;
    this->sm = sm;
    this->ers = ers;
    this->as = as;
    this->md = md;
    this->v_max = v_max;
    this->a_max = a_max;
    this->jam_lim = jam_lim;
    this->jamd = jamd;
    this->pwm_lim = pwm_lim;
    this->i_err = i_err;
    this->b_err = b_err;

    currGoal = 0;
	interStep = 0;
	interVel = 0;

	currVel = 0;
	currPos = 0;
	currCurrent = 0;
	currTemp = 0;

	enabled = false;
	homed = false;
	zeroed = false;

	i = 0;
	buffer = NULL;
}

HuboMotor::HuboMotor(const HuboMotor& rhs){
    this->mpos1 = rhs.mpos1;
    this->mpos2 = rhs.mpos2;
    this->Kp = rhs.Kp;
    this->Kd = rhs.Kd;
    this->Ki = rhs.Ki;
    this->dz = rhs.dz;
    this->off = rhs.off;
    this->hlim = rhs.hlim;
    this->hld = rhs.hld;
    this->hv1 = rhs.hv1;
    this->hv2 = rhs.hv2;
    this->hma = rhs.hma;
    this->sm = rhs.sm;
    this->ers = rhs.ers;
    this->as = rhs.as;
    this->md = rhs.md;
    this->v_max = rhs.v_max;
    this->a_max = rhs.a_max;
    this->jam_lim = rhs.jam_lim;
    this->jamd = rhs.jamd;
    this->pwm_lim = rhs.pwm_lim;
    this->i_err = rhs.i_err;
    this->b_err = rhs.b_err;

    //NEW_DATA
    this->name = rhs.name;
    this->currGoal = rhs.currGoal;
    this->interStep = rhs.interStep;
    this->interVel = rhs.interVel;

    this->currVel = rhs.currVel;
    this->currPos = rhs.currPos;
    this->currCurrent = rhs.currCurrent;
    this->currTemp = rhs.currTemp;

    this->enabled = rhs.enabled;
    this->homed = rhs.homed;
    this->zeroed = rhs.zeroed;

    this->buffer = rhs.buffer; //SHALLOW COPY
    this->i = rhs.i;
}

void HuboMotor::setUpperLimit(long limit){
    this->mpos2 = limit;
}

void HuboMotor::setLowerLimit(long limit){
    this->mpos1 = limit;
}

void HuboMotor::setPositionGain(long kp, long kd, long ki){
    this->Kp = kp;
    this->Kd = kd;
    this->Ki = ki;
}

void HuboMotor::setCurrentGain(long kpt, long kdt, long kf){
    this->Kpt = kpt;
    this->Kdt = kdt;
    this->Kf = kf;
}

void HuboMotor::setDeadZone(int dz){
    this->dz = dz;
}

void HuboMotor::setHomeset1(long off, long hlim, long hld){
    this->off = off;
    this->hlim = hlim;
    this->hld = hld;
}

void HuboMotor::setHomeset2(long hv1, long hv2, long hma, long sm){
    this->hv1 = hv1;
    this->hv2 = hv2;
    this->hma = hma;
    this->sm = sm;
}

void HuboMotor::setEncoderResolution(long ers, long as, long md){
    this->ers = ers;
    this->as = as;
    this->md = md;
}

void HuboMotor::setSpeedLimit(long vel, long acc){
    this->v_max = vel;
    this->a_max = acc;
}

void HuboMotor::setJamPowerLimit(long jam_lim, long jamd, long pwm_lim){
    this->jam_lim = jam_lim;
    this->jamd = jamd;
    this->pwm_lim = pwm_lim;
}

void HuboMotor::setErrorLimit(long i_err, long b_err){
    this->i_err = i_err;
    this->b_err = b_err;
}

void HuboMotor::setGearRatios(long drive, long driven, long harm, long enc){
	this->drive = drive;
	this->driven = driven;
	this->harm = harm;
	this->enc = enc;
}

long HuboMotor::getUpperLimit(){
    return this->mpos2;
}

long HuboMotor::getLowerLimit(){
    return this->mpos1;
}

long HuboMotor::getKp(){
    return this->Kp;
}

long HuboMotor::getKi(){
    return this->Ki;
}

long HuboMotor::getKd(){
    return this->Kd;
}

long HuboMotor::getKpt(){
    return this->Kpt;
}

long HuboMotor::getKdt(){
    return this->Kdt;
}

long HuboMotor::getKf(){
    return this->Kf;
}

long HuboMotor::getDz(){
    return this->dz;
}

long HuboMotor::getOff(){
    return this->off;
}

long HuboMotor::getHlim(){
    return this->hlim;
}

long HuboMotor::getHld(){
    return this->hld;
}

long HuboMotor::getHv1(){
    return this->hv1;
}

long HuboMotor::getHv2(){
    return this->hv2;
}

long HuboMotor::getHma(){
    return this->hma;
}

long HuboMotor::getSm(){
    return this->sm;
}

long HuboMotor::getErs(){
    return this->ers;
}

long HuboMotor::getAs(){
    return this->as;
}

long HuboMotor::getMd(){
    return this->md;
}

long HuboMotor::getVmax(){
    return this->v_max;
}

long HuboMotor::getAmax(){
    return this->a_max;
}

long HuboMotor::getJamLim(){
    return this->jam_lim;
}

long HuboMotor::getJamd(){
    return this->jamd;
}

long HuboMotor::getPwmLim(){
    return this->pwm_lim;
}

long HuboMotor::getIerr(){
    return i_err;
}

long HuboMotor::getBerr(){
    return b_err;
}

bool HuboMotor::requiresMotion(){
	return currGoal != interStep;
}

/*
 * ** DEPRECATED **
long HuboMotor::interpolate(int MAX_STEP, int MIN_STEP){
	const float LEAP_PERCENTAGE = .5;
	const int FREQUENCY = 100; //Hertz
	if (MAX_STEP == 0)
		MAX_STEP = radiansToTicks(omega) / 100;

	int error = desired_position - ticks_position;
	int output = desired_position;

	if((abs(error) > MIN_STEP)){
		output = (int)(LEAP_PERCENTAGE * error);

		if(abs(output) > MAX_STEP)
			output = output < 0 ? -MAX_STEP : MAX_STEP;

	} else
		output = error;

	output += ticks_position;
	ticks_position = output;
	return output;
}
*/

vector<double> *HuboMotor::getBuffer(){
	if (buffer == NULL)
		buffer = new vector<double>(10);
	i = 0;
	return buffer;
}

double HuboMotor::interpolate(){
	if (frequency == 0) return interStep; //If the frequency is 0, no motion occurs.

	const float LEAP_PERCENTAGE = .5;
	const double MIN_STEP = .00001;
	const double MAX_STEP = interVel/frequency; //Radians per second, divided by our operating frequency.

	double error = currGoal - interStep;
	if (error == 0) return currGoal;
	double output = currGoal;

	if((fabs(error) > MIN_STEP)){
		output = (LEAP_PERCENTAGE * error);

		if(fabs(output) > MAX_STEP)
			output = output < 0 ? -MAX_STEP : MAX_STEP;

	} else
		output = error;

	output += interStep;
	interStep = output;
	return interStep;
}

double HuboMotor::nextPosition(){
	if (buffer->size() == 0 || i >= buffer->size())
		return interStep;
	interStep = (*buffer)[i];
	currGoal = (*buffer)[i];
	return (*buffer)[i++];
}


void HuboMotor::setName(string name){
	this->name = name;
}

void HuboMotor::setGoalPosition(double rads){
	currGoal = rads;
}

void HuboMotor::setInterVelocity(double omega){
	interVel = omega;
}

void HuboMotor::setFrequency(double frequency){
	this->frequency = frequency;
}

void HuboMotor::update(double position, double velocity, double temperature, double current, bool homed, int errors){
	currPos = position;
	currVel = velocity;
	currTemp = temperature;
	currCurrent = current;
	this->homed = homed;
	this->errors = errors;
}

void HuboMotor::setEnabled(bool enabled){
	this->enabled = enabled;
	//interStep = currGoal; //If we have recently changed from non-interpolation to interpolation, the step MUST be updated.
}

void HuboMotor::setInterStep(double rads){
	//This method should ONLY be used when switching control mode to interpolation.
	//The argument to this method should be the current actual position of the motor.
	this->interStep = rads;
}

void HuboMotor::setZeroed(bool zeroed){
	this->zeroed = zeroed;
}

string HuboMotor::getName(){
	return name;
}

double HuboMotor::getGoalPosition(){
	return currGoal;
}

double HuboMotor::getPosition(){
	return currPos;
}

double HuboMotor::getVelocity(){
	return currVel;
}

double HuboMotor::getTemperature(){
	return currTemp;
}

double HuboMotor::getCurrent(){
	return currCurrent;
}

bool HuboMotor::isEnabled(){
	return enabled;
}

bool HuboMotor::isHomed(){
	return homed;
}

bool HuboMotor::isZeroed(){
	return zeroed;
}

bool HuboMotor::hasError(){
	return errors != 0;
}

bool HuboMotor::hasError(PROPERTY error){
	switch (error){
	case JAM_ERROR:
		return (bool)(errors & 0x200); 	// 1000000000
	case PWM_SATURATED_ERROR:
		return (bool)(errors & 0x100); 	// 0100000000
	case BIG_ERROR:
		return (bool)(errors & 0x80);  	// 0010000000
	case ENC_ERROR:
		return (bool)(errors & 0x40);	// 0001000000
	case DRIVE_FAULT_ERROR:
		return (bool)(errors & 0x20);	// 0000100000
	case POS_MIN_ERROR:
		return (bool)(errors & 0x10);	// 0000010000
	case POS_MAX_ERROR:
		return (bool)(errors & 0x8);	// 0000001000
	case VELOCITY_ERROR:
		return (bool)(errors & 0x4);	// 0000000100
	case ACCELERATION_ERROR:
		return (bool)(errors & 0x2);	// 0000000010
	case TEMP_ERROR:
		return (bool)(errors & 0x1);	// 0000000001
	default:
		return false;
	}
}

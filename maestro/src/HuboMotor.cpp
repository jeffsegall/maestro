#include "HuboMotor.h"
#include <iostream>

HuboMotor::HuboMotor(){
    this->ticks_position = 0;
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
    this->ticks_position = 0;
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
    this->ticks_position = rhs.ticks_position;
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

void HuboMotor::setTicksPosition(long ticks){
    this->ticks_position = ticks;
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

long HuboMotor::getTicksPosition(){
    return this->ticks_position;
}

double ticksToRadians(long ticks){
	return (ticks * (double)(drive * 2 * M_PI))/(driven * harm * enc);
}

long radiansToTicks(double rad){
	return (long)(rad * ((double)(driven * harm * enc))/(drive * 2 * M_PI));
}

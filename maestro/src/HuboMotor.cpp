#include "HuboMotor.h"

HuboMotor::HuboMotor(){
    this->ticks_position = 0;
}

void HuboMotor::setUpperLimit(long limit){
    this->limit_upper = limit;
}

void HuboMotor::setLowerLimit(long limit){
    this->limit_lower = limit;
}

void HuboMotor::setPositionGain0(long kp0, long kd0, long ki0){
    this->Kp0 = kp0;
	this->Kd0 = kd0;
	this->Ki0 = ki0;
}

void HuboMotor::setPositionGain1(long kp1, long kd1, long ki1){
    this->Kp1 = kp1;
	this->Kd1 = kd1;
	this->Ki1 = ki1;
}

void HuboMotor::setCurrentGain0(long kpt0, long kdt0, long kf0){
    this->Kpt0 = kpt0;
	this->Kdt0 = kdt0;
	this->Kf0 = kf0;
}

void HuboMotor::setCurrentGain1(long kpt1, long kdt1, long kf1){
    this->Kpt1 = kpt1;
	this->Kdt1 = kdt1;
	this->Kf1 = kf1;
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

void HuboMotor::setErrorLimit(long l_err, long b_err){
    this->l_err = l_err;
	this->b_err = b_err;
}

void HuboMotor::setTicksPosition(long ticks){
    this->ticks_position = ticks;
}

long HuboMotor::getTicksPosition(){
    return this->ticks_position;
}

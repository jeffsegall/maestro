#ifndef HUBOMOTOR_H
#define HUBOMOTOR_H

/* The HuboMotor class defines some state parameters for a single motor in the HUBO+ */

class HuboMotor {

    private:
	
	//This is an exhaustive list of parameters supported by the motors and may be overkill
	//in the initial stages of the system.
	
	long limit_upper, limit_lower;  //Limit position
	long Kp0, Ki0, Kd0;             //Position Gain 0
	long Kp1, Ki1, Kd1;             //Position Gain 1
	long Kpt0, Kdt0, Kf0;           //Current Gain 0
	long Kpt1, Kdt1, Kf1;           //Current Gain 1
	int dz;                         //Dead zone
	long off, hlim, hld;            //Homeset 1
	long hv1, hv2, hma, sm;         //Homeset 2
	long ers, as, md;               //Encoder Resolution
	long v_max, a_max;              //Speed and acceleration limits
	long jam_lim, jamd, pwm_lim;    //Jam and power sat limits
	long l_err, b_err;              //Error limits
	
	long ticks_position;            //Current position in ticks from zero
	
	public:

    HuboMotor();
	
	void setUpperLimit(long limit);
	void setLowerLimit(long limit);
	void setPositionGain0(long kp0, long kd0, long ki0);
	void setPositionGain1(long kp1, long kd1, long ki1);
	void setCurrentGain0(long kpt0, long kdt0, long kf0);
	void setCurrentGain1(long kpt1, long kdt1, long kf1);
	void setDeadZone(int dz);
	void setHomeset1(long off, long hlim, long hld);
	void setHomeset2(long hv1, long hv2, long hma, long sm);
	void setEncoderResolution(long ers, long as, long md);
	void setSpeedLimit(long vel, long acc);
	void setJamPowerLimit(long jam_lim, long jamd, long pwm_lim);
	void setErrorLimit(long l_err, long b_err);
	void setTicksPosition(long ticks);
	
	//@TODO:  Need getter methods for these variables.  Individual?  Group?
	
	long getTicksPosition();
};

#endif

#ifndef HUBOMOTOR_H
#define HUBOMOTOR_H

#include <math.h>
#include <stdlib.h>
#include <string>

#include "Property.h"

#define MAX_ANGULAR_VELOCITY 2

using namespace std;

/* The HuboMotor class defines some state parameters for a single motor in the HUBO+ */

class HuboMotor {

    private:
	
	//This is an exhaustive list of parameters supported by the motors and may be overkill
	//in the initial stages of the system.
	
	long mpos1, mpos2;              //Limit positions
	long Kp, Kd, Ki;                //Position Gain
	long Kpt, Kdt, Kf;              //Current Gain
	int dz;                         //Dead zone
	long off, hlim, hld;            //Homeset 1
	long hv1, hv2, hma, sm;         //Homeset 2
	long ers, as, md;               //Encoder Resolution
	long v_max, a_max;              //Speed and acceleration limits
	long jam_lim, jamd, pwm_lim;    //Jam and power sat limits
	long i_err, b_err;              //Error limits
	
	long drive, driven, harm, enc; 	//gear ratios for conversion from radians to ticks

	//Identification
	string name;

	//Output Data
	double currGoal;				//Goal position in radians
	double interStep;				//Current interpolated step in radians
	double interVel;				//Current interpolated velocity in rad/sec

	//Sensor Data
	double currVel;					//Reported velocity (units?)
	double currPos;					//Reported position in radians
	double currCurrent;				//Reported current (units?)
	double currTemp;				//Reported temperature (units?)

	//Internal State
	bool enabled;					//Whether the motor has motion enabled
	bool homed;						//Whether the motor has been homed or not
	bool zeroed;					//Whether the sensors have been zeroed or not

	public:

	HuboMotor();
	HuboMotor(long mpos1, long mpos2, long kp, long kd, long ki,
			  long dz, long off, long hlim, long hld, long hv1,
			  long hv2, long hma, long sm, long ers, long as, long md,
			  long v_max, long a_max, long jam_lim, long jamd,
			  long pwm_lim, long i_err, long b_err);
	HuboMotor(const HuboMotor& rhs);

	// ***** OLD ****
	void setUpperLimit(long limit);
	void setLowerLimit(long limit);
	void setPositionGain(long kp, long kd, long ki);
	void setCurrentGain(long kpt, long kdt, long kf);
	void setDeadZone(int dz);
	void setHomeset1(long off, long hlim, long hld);
	void setHomeset2(long hv1, long hv2, long hma, long sm);
	void setEncoderResolution(long ers, long as, long md);
	void setSpeedLimit(long vel, long acc);
	void setJamPowerLimit(long jam_lim, long jamd, long pwm_lim);
	void setErrorLimit(long i_err, long b_err);
	void setGearRatios(long drive, long driven, long harm, long enc);

	// ******* OLD ******
	long getUpperLimit();
	long getLowerLimit();
	long getKp();
	long getKi();
	long getKd();
	long getKpt();
	long getKdt();
	long getKf();
	long getDz();
	long getOff();
	long getHlim();
	long getHld();
	long getHv1();
	long getHv2();
	long getHma();
	long getSm();
	long getErs();
	long getAs();
	long getMd();
	long getVmax();
	long getAmax();
	long getJamLim();
	long getJamd();
	long getPwmLim();
	long getIerr();
	long getBerr();

	void setName(string name);
	void setGoalPosition(double rads);
	void setInterVelocity(double omega);
	void update(double position, double velocity, double temperature, double current);
	void setEnabled(bool enabled);
	void setHomed(bool homed);
	void setZeroed(bool zeroed);
	double interpolate();

	string getName();
	double getGoalPosition();
	double getPosition();
	double getVelocity();
	double getTemperature();
	double getCurrent();
	bool isEnabled();
	bool isHomed();
	bool isZeroed();
	bool requiresMotion();

	//******** OLD **********
	//long interpolate(int MAX_STEP, int MIN_STEP); //DEPRECATED

};

#endif

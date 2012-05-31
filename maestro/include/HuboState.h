#ifndef HUBOSTATE_H
#define HUBOSTATE_H

/*Due to the nature of the HUBO+ CAN protocol, it is often necessary to keep track of the current state of some joints.
  For example, if a user only wants to move one motor, but requires a transmitted packet that affects two motors, it is 
  necessary to keep track of the position of the other motor to send along with the new position.
  
  The HuboState class is a collection of states that defines the current positions of the motors in the HUBO system.
*/

#include "HuboMotor.h"
#include <map>
using namespace std;

// Motor names from the HUBO+ protocol

enum {
    RHY, RHR, RHP1, RHP2, RKN1, RKN2, RAP, RAR, LHY, LHR, LHP1, LHP2,
	LKN1, LKN2, LAP, LAR, SRP, SRS, SRY, REB, LSP, LSR, LSY, LEB, RWY,
	RWP, LWY, LWP, NKY, NK1, NK2, WST, RH0, RH1, RH2, RH3, RH4, RH5,
	LH0, LH1, LH2, LH3, LH4, LH5;
} MOTOR_NAME;

class HuboState{

    private:
    
	map<MOTOR_NAME, HuboMotor*> motors;
	
	public:

	HuboState(){
	    //Initialize all of the motors to an empty device
	    for (int i = MOTOR_NAME.RHY, i <= MOTOR_NAME.LH5, i++){
		    motors[static_cast<MOTOR_NAME>(i)] = new HuboMotor();
		}
	}
	
	HuboMotor* getMotorByName(MOTOR_NAME name) {
	    return motors[name];
	}

#endif
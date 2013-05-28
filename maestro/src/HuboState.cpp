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
#include "HuboState.h"
#include <string>
#include <iostream>
#include "pugixml.hpp"
#include "huboCan.h"
#include <queue>

using std::queue;
using std::string;

HuboState::HuboState(){
	propertyMap["position"] = POSITION;
	propertyMap["velocity"] = VELOCITY;
	propertyMap["temp"] = TEMPERATURE;
	propertyMap["homed"] = HOMED;
	propertyMap["zeroed"] = ZEROED;
	propertyMap["enabled"] = ENABLED;
	propertyMap["errored"] = ERRORED;
	propertyMap["jamError"] = JAM_ERROR;
	propertyMap["PWMSaturatedError"] = PWM_SATURATED_ERROR;
	propertyMap["bigError"] = BIG_ERROR;
	propertyMap["encoderError"] = ENC_ERROR;
	propertyMap["driveFaultError"] = DRIVE_FAULT_ERROR;
	propertyMap["posMinError"] = POS_MIN_ERROR;
	propertyMap["posMaxError"] = POS_MAX_ERROR;
	propertyMap["velocityError"] = VELOCITY_ERROR;
	propertyMap["accelerationError"] = ACCELERATION_ERROR;
	propertyMap["tempError"] = TEMP_ERROR;
	propertyMap["x_acc"] = X_ACCEL;
	propertyMap["y_acc"] = Y_ACCEL;
	propertyMap["z_acc"] = Z_ACCEL;
	propertyMap["x_rot"] = X_ROTAT;
	propertyMap["y_rot"] = Y_ROTAT;
	propertyMap["m_x"] = M_X;
	propertyMap["m_y"] = M_Y;
	propertyMap["f_z"] = F_Z;
}

/******************************************************************************
* initHuboWithDefaults
*
* Reads in an XML representation of a HUBO and sets up initial data structures.
*
* @param	path		The file path of the XML representation
******************************************************************************/
void HuboState::initHuboWithDefaults(string path, double frequency, queue<hubomsg::HuboCommand>* outQueue){
    pugi::xml_document doc;
    if (!doc.load_file(path.c_str())){
        std::cout << "No such file, " << path.c_str() << std::endl;
        return;
    }
    pugi::xml_node robot = doc.child("robot");

    //Loop through each board

    for (pugi::xml_node board = robot.first_child(); board; board = board.next_sibling())
    {
        std::cout << "Board: ";

        int BNO = board.attribute("number").as_int();
        int channels = board.attribute("channels").as_int();
 
        std::cout << "Number " << BNO << std::endl;
        std::cout << "Channels " << channels << std::endl;

        std::cout << "Before new mb" << std::endl;

        MotorBoard* mb = new MotorBoard((boardNum)BNO, channels, outQueue);

        std::cout << "After new mb" << std::endl;

        //Loop through each motor on each board.  Initialize a blank motor and then use
        //the board methods to set values.  This way the values are also sent to the
        //hardware.

        for (pugi::xml_node motor = board.first_child(); motor; motor = motor.next_sibling()){
        	string name = motor.attribute("name").as_string();
            std::cout << "Motor: " << name << std::endl;
            
            HuboMotor* hm = new HuboMotor();
            int CH = motor.attribute("channel").as_int();
        
            mb->addMotor(hm, CH);
            std::cout << "Added motor to: " << mb->getMotorByChannel(CH) << std::endl;
            mb->getMotorByChannel(CH)->setName(name);
            mb->getMotorByChannel(CH)->setFrequency(frequency);

            assert(motorMap.count(name) == 0); //Multiple motors should not have the same name.
            motorMap[name] = hm;

            // Remnants of Original CAN Communication. These do nothing now.
            // Due to be phased out.
            mb->resetEncoderToZero(CH);
            mb->initBoard();
            mb->setLowerPosLimit(CH, 3, motor.attribute("mpos1").as_int());
            mb->setUpperPosLimit(CH, 3, motor.attribute("mpos2").as_int());
            if (channels > 2)
                mb->setPositionGain(0, motor.attribute("kp").as_int(),
                                       motor.attribute("ki").as_int(),
                                       motor.attribute("kd").as_int());
            else
                mb->setPositionGain(CH, motor.attribute("kp").as_int(),
                                        motor.attribute("ki").as_int(),
                                        motor.attribute("kd").as_int());
            mb->setDeadZone(CH, motor.attribute("dz").as_int());
            mb->setHomeSearchParams(CH, motor.attribute("hlim").as_int(),
                                        motor.attribute("hld").as_int(),
                                        motor.attribute("off").as_int());
            mb->setHomeVelAcc(CH, motor.attribute("hma").as_int(),
                                  motor.attribute("hv1").as_int(),
                                  motor.attribute("hv2").as_int(),
                                  motor.attribute("sm").as_int(),
                                  motor.attribute("hld").as_int());
            mb->setEncoderResolution(CH, motor.attribute("ers").as_int(),
                                         motor.attribute("as").as_int(),
                                         motor.attribute("md").as_int()); 
            mb->setMaxAccVel(CH, motor.attribute("a_max").as_int(),
                                 motor.attribute("v_max").as_int());
            //Only do this on the last motor of each board.
            if (CH == (channels - 1)){
                mb->setJamPwmSatLim(motor.attribute("jam_lim").as_int(),
                                    motor.attribute("pwm_lim").as_int(),
                                    motor.attribute("hld").as_int(),
                                    motor.attribute("jamd").as_int());
                mb->setRequestBoardInfo(5);
            }

            mb->getMotorByChannel(CH)->setGearRatios(motor.attribute("drive").as_int(),
													 motor.attribute("driven").as_int(),
													 motor.attribute("harm").as_int(),
													 motor.attribute("enc").as_int()); //Set Gear Ratios for Radian <-> Tick conversions
        }
        this->addBoard(mb);
    }


    IMU0 = new IMUBoard(BNO_IMU_0, "IMU");
    IMUSensorMap["IMU"] = IMU0;
    IMU1 = new IMUBoard(BNO_IMU_1, "LAI");
    IMUSensorMap["LAI"] = IMU1;
    IMU2 = new IMUBoard(BNO_IMU_2, "RAI");
    IMUSensorMap["RAI"] = IMU2;

    leftAnkle = new FTSensorBoard(BNO_L_FOOT_FT, "LAT");
    FTSensorMap["LAT"] = leftAnkle;
    rightAnkle = new FTSensorBoard(BNO_R_FOOT_FT, "RAT");
    FTSensorMap["RAT"] = rightAnkle;
    leftWrist = new FTSensorBoard(BNO_L_WRIST_FT, "LWT");
    FTSensorMap["LWT"] = leftWrist;
    rightWrist = new FTSensorBoard(BNO_R_WRIST_FT, "RWT");
    FTSensorMap["RWT"] = rightWrist;
}

/******************************************************************************
* getBoardByNumber
* 
* Returns the motor board with the given number.
*
* @param	number		The motor board number
* @return	The motor board with the given number.  NULL if a board does
*		not exist with the given number.
******************************************************************************/
MotorBoard* HuboState::getBoardByNumber(int number){
    return this->getBoardByNumber((boardNum)number);
}

/******************************************************************************
* getBoardByNumber
* 
* Returns the motor board with the given number.
*
* @param	number		The motor board number
* @return	The motor board with the given number.  NULL if a board does
*		not exist with the given number.
******************************************************************************/
MotorBoard* HuboState::getBoardByNumber(boardNum number){
	for (int i = 0; i < boards.size(); i++){
		if (boards[i]->getBoardNumber() == number){
			return boards[i];
		}
	}
	return NULL;
}

HuboMotor* HuboState::getMotorByName(string name){
	for (vector<MotorBoard*>::iterator it = boards.begin(); it != boards.end(); it++){
		for (int i = 0; i < (*it)->getNumChannels(); i++){
			if ((*it)->getMotorByChannel(i)->getName().compare(name) == 0)
				return (*it)->getMotorByChannel(i);
		}
	}
	return NULL;
}

/******************************************************************************
* addBoard
* 
* Adds a motor board to the robot with the given number.
*
* @param	num		The motor board number
* @param	board		The motor board to add
******************************************************************************/
void HuboState::addBoard(MotorBoard* board){
    this->boards.push_back(board);
}

vector<MotorBoard*> HuboState::getBoards(){
    return this->boards;
}

map<string, HuboMotor*> HuboState::getBoardMap(){
	return this->motorMap;
}

map<string, PROPERTY> HuboState::getPropertyMap(){
	return this->propertyMap;
}

map<string, FTSensorBoard*> HuboState::getFTSensorMap(){
	return FTSensorMap;
}

map<string, IMUBoard*> HuboState::getIMUSensorMap(){
	return IMUSensorMap;
}

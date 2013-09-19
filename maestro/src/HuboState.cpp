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

        int channels = board.attribute("channels").as_int();
 
        std::cout << "Channels " << channels << std::endl;

        std::cout << "Before new mb" << std::endl;

        MotorBoard* mb = new MotorBoard(channels, outQueue);

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

        }
        this->addBoard(mb);
    }


    IMU0 = new IMUBoard("IMU");
    IMUSensorMap["IMU"] = IMU0;
    IMU1 = new IMUBoard("LAI");
    IMUSensorMap["LAI"] = IMU1;
    IMU2 = new IMUBoard("RAI");
    IMUSensorMap["RAI"] = IMU2;

    leftAnkle = new FTSensorBoard("LAT");
    FTSensorMap["LAT"] = leftAnkle;
    rightAnkle = new FTSensorBoard("RAT");
    FTSensorMap["RAT"] = rightAnkle;
    leftWrist = new FTSensorBoard("LWT");
    FTSensorMap["LWT"] = leftWrist;
    rightWrist = new FTSensorBoard("RWT");
    FTSensorMap["RWT"] = rightWrist;
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

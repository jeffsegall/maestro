#include "HuboState.h"
#include <string>
#include <iostream>
#include "pugixml.hpp"
#include "huboCan.h"

using std::string;

/******************************************************************************
* initHuboWithDefaults
*
* Reads in an XML representation of a HUBO and sets up initial data structures.
*
* @param	path		The file path of the XML representation
******************************************************************************/
void HuboState::initHuboWithDefaults(string path){
    pugi::xml_document doc;
    if (!doc.load_file(path)) return -1;

    pugi::xml_node robot = doc.child("robot");

    //Loop through each board

    for (pugi::xml_node board = robot.first_child(); board; board = board.next_sibling())
    {
        std::cout << "Board: ";

        int BNO = board.attibute("number").as_int();
        int channels = board.attribute("channels").as_int();
 
        std::cout << "Number " << BNO << std::endl;
        std::cout << "Channels " << channels << std::endl;

        MotorBoard* mb = new MotorBoard((boardNum)BNO, channels);

        //Loop through each motor on each board.  Initialize a blank motor and then use
        //the board methods to set values.  This way the values are also sent to the
        //hardware.

        for (pugi::xml_node motor = board.first_child(); motor; motor = motor.next_sibling()){
            std::count << "Motor: ";
            
            HuboMotor* hm = new HuboMotor();
            int CH = motor.attribute("channel").as_int();
        
            mb->addMotor(hm, CH);
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
            mb->setDeadZone(CH, motor.attribute("dz"));
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
            mb->setJamPwmSatLim(CH, motor.attribute("jam_lim").as_int(),
                                    motor.attribute("pwm_lim").as_int(),
                                    motor.attribute("hld").as_int(),
                                    motor.attribute("jamd").as_int());
        }

        this->addBoard(BNO, mb);
    }
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
    this->getBoardByNumber((boardNum)number);
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
    map<boardNum, HuboMotor*>::iterator it;

    it = this->boards.find(number);
    if (it == this->board.end())
        return NULL;

    return it->second;
}

/******************************************************************************
* addBoard
* 
* Adds a motor board to the robot with the given number.
*
* @param	num		The motor board number
* @param	board		The motor board to add
******************************************************************************/
void HuboState::addBoard(int num, MotorBoard* board){
    this->boards[num] = board;
}

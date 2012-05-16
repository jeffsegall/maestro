/* 
    Conductor: High Degree of Freedom Robot Controller Framework
    Copyright (C) 2010, 2011 Robert Sherbert
    bob.sherbert@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    If you are interested in licensing this software for commercial purposes
    please contact the author. The software can be licensed to you under
    non-free license terms by the copyright holder.

    As a courtesy to the author, and in the spirit of fair attribution, you are
    asked to cite the following paper if any work based upon or utilizing this
    framework is published in the scientific literature: 
    Sherbert, Robert M. and Oh, Paul Y. "Conductor: A Controller Development
    Framework for High Degree of Freedom Systems." Intelligent Robots and
    Systems (IROS), 2011 IEEE/RSJ International Conference on. 
*/

#ifndef ACES_HUBOCANi_HPP
#define ACES_HUBOCAN_HPP

#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdio.h>
#include <sys/time.h>
#include <assert.h>
#include <string>
#include "can4linux.h"  //Include the can4linux data structures

using namespace std;

//For reference, the canmsg_t data structure from can4linux
/*
typedef struct {
    // flags, indicating or controlling special message properties 
    int             flags;
    int             cob;	     // < CAN object number, used in Full CAN  
    unsigned   long id;		     // < CAN message ID, 4 bytes  
    struct timeval  timestamp;	 // < time stamp for received messages 
    short      int  length;	     // < number of bytes in the CAN message 
    unsigned   char data[CAN_MSG_LENGTH]; // < data, 0...8 bytes 
} canmsg_t;
*/

//CAN Notes
/*
    High ID -> Low Priority
    Low ID -> High Priority
*/

    enum messageType { CAN_NONE = 0x0, 
                       TX_MOTOR_CMD = 0x1, TX_SENSOR_CMD = 0x2, TX_REF = 0x10, 
                       RX_FT_SENSOR = 0x40, RX_TILT_SENSOR = 0x50,
                       RX_ENC_VAL = 0x60, CUR_RXDF = 0x90, PM_RXDF = 0x120,
                       RX_STATUS = 0x150, RX_BOARD_INFO = 0x190,
                       RX_BOARD_PARA_CUR = 0x1C0 };
                       
    enum cmdType { //Common Commands
                   CMD_SETREQ_BOARD_INFO = 0x1, CMD_REQ_BOARD_PARAM = 0x24, 
                   CMD_NULL = 0x81, CMD_SET_BNO_FREQ = 0xA8,
                   //Motor Control Board Commands
                   CMD_NONE = 0x0, CMD_REQ_BOARD_STATUS = 0x02,
                   CMD_REQ_ENC_POS, CMD_REQ_CURR, CMD_RESET_ENC_ZERO = 0x6, 
                   CMD_SET_POS_GAIN_0, CMD_SET_POS_GAIN_1, CMD_SET_CUR_GAIN_0,
                   CMD_SET_CUR_GAIN_1, CMD_HIP_ENABLE, CMD_OPEN_LOOP = 0xD, 
                   CMD_CONTROLLER_ON, CMD_CONTROLLER_OFF, CMD_SET_CTRL_MODE, 
                   CMD_GO_HOME_OFFSET, CMD_SET_DEAD_ZONE = 0x20, 
                   CMD_SET_HOME_SEARCH = 0x30, CMD_SET_ENC_RESOLUTION = 0x38, 
                   CMD_SET_MAX_ACC_VEL = 0x40, CMD_SET_LOWER_POS_LIMIT = 0x50, 
                   CMD_SET_UPPER_POS_LIMIT = 0x56, CMD_SET_HOME_ACC_VEL = 0x60, 
                   CMD_SET_GAIN_OVR = 0x6F, CMD_NEW_BOARD_NUM = 0xF0, 
                   CMD_SET_JAM_PWM_SAT_LIMIT = 0xF2, CMD_SET_ERR_BOUND = 0xF3, 
                   CMD_INIT_BOARD = 0xFA, 
                   //FT Sensor Board Commands
                   CMD_SET_FT_0 = 0xA0, CMD_SET_FT_1,
                   CMD_SET_FT_2, CMD_SET_INCLINO_SCALE = 0xA5,
                   //Power Control Board Commands
                   CMD_SET_SWITCH = 0x81, CMD_REQ_ALARM, CMD_REQ_BEEP,
                   CMD_REQ_VOLT_CUR = 0xE0, CMD_REQ_TIME_STATUS,
                   //IMU Board Commands
                   CMD_CALIBRATE = 0x82 }; 

    enum boardNum { BNO_R_HIP_YAW_ROLL = 0x0, BNO_R_HIP_PITCH, BNO_R_KNEE, BNO_R_ANKLE_PITCH_ROLL,
                    BNO_L_HIP_YAW_ROLL, BNO_L_HIP_PITCH, BNO_L_KNEE, BNO_L_ANKLE_PITCH_ROLL,
                    BNO_R_SHOULDER_PITCH_ROLL, BNO_R_SHOULDER_YAW_ELBOW,
                    BNO_L_SHOULDER_PITCH_ROLL, BNO_L_SHOULDER_YAW_ELBOW, BNO_EXTRA_JMC12,
                    BNO_EXTRA_JMC13, BNO_SMART_POWER, BNO_EXTRA_JMC15, BNO_R_WRIST_YAW_PITCH = 0x20,
                    BNO_L_WRIST_YAW_PITCH, BNO_NECK_YAW_1_2, BNO_WAIST, BNO_R_HAND, BNO_L_HAND,
                    BNO_EXTRA_EJMC6, BNO_R_FOOT_FT = 0x30, BNO_L_FOOT_FT, BNO_IMU_0, BNO_IMU_1,
                    BNO_IMU_2, BNO_R_WRIST_FT, BNO_L_WRIST_FT };

    class canMsg {
        public:

            //@TODO: Finish restructuring this class to better represent CAN,
            //       including output to serial.
            canMsg();
            canMsg(boardNum BNO, messageType type, cmdType subType);
            canMsg(boardNum BNO, messageType type, cmdType subType,
                   unsigned long r1, unsigned long r2,
                   unsigned long r3, unsigned long r4,
                   unsigned long r5);
            boardNum getBNO();
            messageType getType();
            cmdType getCmd();

            unsigned long getR1();
            unsigned long getR2();
            unsigned long getR3();
            unsigned long getR4();
            unsigned long getR5();

            void printme();
            static unsigned long bitStuff15byte(long bs);
            static unsigned long bitStuff3byte(long bs);
            static unsigned long bitStuffCalibratePacket(long bs);
            static unsigned long unpack2byte();
            static unsigned long unpack4byte();
            static unsigned char bitStrip(unsigned long src, int byteNum);
            canmsg_t* toCAN();
            string toSerial();
            canmsg_t* toLineType();
        private:                       //   [bits] Meaning
            canmsg_t* processCMD(canmsg_t*);
            canmsg_t* processREF(canmsg_t*);

            boardNum BNO;
            messageType type;
            cmdType subType;

            unsigned long r1;  //multi-purpose configuration registers
            unsigned long r2;
            unsigned long r3;
            unsigned long r4;
            unsigned long r5;
    };
#endif

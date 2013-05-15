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
    aint with this program.  If not, see <http://www.gnu.org/licenses/>.

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

#ifndef ACES_HUBOCAN_HPP
#define ACES_HUBOCAN_HPP

#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdio.h>
#include <sys/time.h>
#include <assert.h>
#include <string>
#include <algorithm>
#include "can4linux.h"  //Include the can4linux data structures

using namespace std;

//For reference, the canmsg_t data structure from can4linux
/*
typedef struct {
    // flags, indicating or controlling special message properties 
    int             flags;
    int             cob;	     // < CAN object number, used in Full CAN  
    unsigned   int id;		     // < CAN message ID, 4 bytes  
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

    #define BOFF 0x80

    enum messageType { CAN_NONE = 0x0, 
                       TX_MOTOR_CMD = 0x1, TX_SENSOR_CMD = 0x2, TX_REF = 0x10, 
                       RX_FT_SENSOR = 0x40, RX_TILT_SENSOR = 0x50, RX_ENC_VAL = 0x60, 
                       RX_STATUS = 0x150, RX_BOARD_INFO = 0x190, 
                       RX_BOARD_PARA_CUR = 0x1C0 };
                       
    enum cmdType { //Common Commands
                   CMD_SETREQ_BOARD_INFO = 0x1, CMD_REQ_BOARD_PARAM = 0x24, 
                   CMD_NULL = 0x81, CMD_SET_BNO_FREQ = 0xA8,
                   //Motor Control Board Commands
                   CMD_NONE = 0x0, CMD_REQ_BOARD_STATUS = 0x2,
                   CMD_REQ_ENC_POS = 0x3, CMD_REQ_CURR = 0x4, CMD_RESET_ENC_ZERO = 0x6, 
                   CMD_SET_POS_GAIN_0 = 0x7, CMD_SET_POS_GAIN_1 = 0x8, CMD_SET_CUR_GAIN_0 = 0x9,
                   CMD_SET_CUR_GAIN_1 = 0xA, CMD_HIP_ENABLE = 0xB, CMD_OPEN_LOOP = 0xD, 
                   CMD_CONTROLLER_ON = 0xE, CMD_CONTROLLER_OFF = 0xF, CMD_SET_CTRL_MODE = 0x10, 
                   CMD_GO_HOME_OFFSET = 0x11, CMD_SET_DEAD_ZONE = 0x20, 
                   CMD_SET_HOME_SEARCH = 0x30, CMD_SET_ENC_RESOLUTION = 0x38, 
                   CMD_SET_MAX_ACC_VEL = 0x40, CMD_SET_LOWER_POS_LIMIT = 0x50, 
                   CMD_SET_UPPER_POS_LIMIT = 0x56, CMD_SET_HOME_ACC_VEL = 0x60, 
                   CMD_SET_GAIN_OVR = 0x6F, CMD_NEW_BOARD_NUM = 0xF0, 
                   CMD_SET_JAM_PWM_SAT_LIMIT = 0xF2, CMD_SET_ERR_BOUND = 0xF3, 
                   CMD_INIT_BOARD = 0xFA, 
                   //FT Sensor Board Commands
                   CMD_SET_FT_0 = 0xA0, CMD_SET_FT_1 = 0xA1,
                   CMD_SET_FT_2 = 0xA2, CMD_SET_INCLINO_SCALE = 0xA5,
                   //Power Control Board Commands
                   CMD_SET_SWITCH = 0x81, CMD_REQ_ALARM = 0x82, CMD_REQ_BEEP = 0x83,
                   CMD_REQ_VOLT_CUR = 0xE0, CMD_REQ_TIME_STATUS = 0xE1,
                   //IMU Board Commands
                   CMD_CALIBRATE = 0x82 }; 

    enum boardNum { BNO_R_HIP_YAW_ROLL = 0x0, BNO_R_HIP_PITCH = 0x1, BNO_R_KNEE = 0x2, 
                    BNO_R_ANKLE_PITCH_ROLL = 0x3, BNO_L_HIP_YAW_ROLL = 0x4, BNO_L_HIP_PITCH = 0x5, 
                    BNO_L_KNEE = 0x6, BNO_L_ANKLE_PITCH_ROLL = 0x7, BNO_R_SHOULDER_PITCH_ROLL = 0x8, 
                    BNO_R_SHOULDER_YAW_ELBOW = 0x9, BNO_L_SHOULDER_PITCH_ROLL = 0xA, 
                    BNO_L_SHOULDER_YAW_ELBOW = 0xB, BNO_EXTRA_JMC12 = 0xC, BNO_EXTRA_JMC13 = 0xD, 
                    BNO_SMART_POWER = 0xE, BNO_EXTRA_JMC15 = 0xF, BNO_R_WRIST_YAW_PITCH = 0x20,
                    BNO_L_WRIST_YAW_PITCH = 0x21, BNO_NECK_YAW_1_2 = 0x22, BNO_WAIST = 0x23, 
                    BNO_R_HAND = 0x24, BNO_L_HAND = 0x25, BNO_EXTRA_EJMC6 = 0x26, BNO_R_FOOT_FT = 0x30, 
                    BNO_L_FOOT_FT = 0x31, BNO_IMU_0 = 0x32, BNO_IMU_1 = 0x33, BNO_IMU_2 = 0x34, 
                    BNO_R_WRIST_FT = 0x35, BNO_L_WRIST_FT = 0x36 };

    class canMsg {
        public:

            //@TODO: Finish restructuring this class to better represent CAN,
            //       including output to serial.
            canMsg();
            canMsg(boardNum BNO, messageType type, cmdType subType);
            canMsg(boardNum BNO, messageType type, cmdType subType,
                   int r1, int r2,
                   int r3, int r4,
                   int r5, int r6,
                   int r7, int r8);
            canMsg(canmsg_t* msg);
            boardNum getBNO();
            messageType getType();
            cmdType getCmd();

            int getR1();
            int getR2();
            int getR3();
            int getR4();
            int getR5();
            int getR6();
            int getR7();
            int getR8();

            void setR1(int r1);
            void setR2(int r2);
            void setR3(int r3);
            void setR4(int r4);
            void setR5(int r5);
            void setR6(int r6);
            void setR7(int r7);
            void setR8(int r8);

            void printme();
            static int bitStuff15byte(int bs);
            static int bitStuff3byte(int bs);
            static int bitStuffCalibratePacket(int bs);
            static int unpack2byte();
            static int unpack4byte();
            static unsigned char bitStrip(int src, int byteNum);
            canmsg_t* toCAN();
            string toSerial();
            canmsg_t* toLineType();
            static canMsg fromLineType(canmsg_t* msg);
        private:                       //   [bits] Meaning
            canmsg_t* processCMD(canmsg_t*);
            canmsg_t* processREF(canmsg_t*);

            boardNum BNO;
            messageType type;
            cmdType subType;

            int r1;  //multi-purpose configuration registers
            int r2;
            int r3;
            int r4;
            int r5;
            int r6;
            int r7;
            int r8;
    };
#endif

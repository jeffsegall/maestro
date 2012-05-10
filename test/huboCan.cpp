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

#include "huboCanDS.hpp"

namespace Hubo{

    canMsg::canMsg(): id(0), type(CAN_NONE), subType(CMD_NONE), r1(0), r2(0),
                      r3(0), r4(0), r5(0)
    {}

    canMsg::canMsg(unsigned long ID, huboCanType Type, cmdType st) :
        id(ID), type(Type), subType(st), r1(0), r2(0), r3(0), r4(0), r5(0){}

    canMsg::canMsg(unsigned long ID, huboCanType Type, cmdType st,
                   unsigned long R1, unsigned long R2,
                   unsigned long R3, unsigned long R4,
                   unsigned long R5)
            : id(ID), type(Type), subType(st), r1(R1), r2(R2), r3(R3), r4(R4),
              r5(R5){}

    unsigned long canMsg::getID(){
        return id;
    }

    huboCanType canMsg::getType(){
        return type;
    }

    cmdType canMsg::getCmd(){
        return subType;
    }

    unsigned long canMsg::getR1(){
        return r1;
    }

    unsigned long canMsg::getR2(){
        return r2;
    }

    unsigned long canMsg::getR3(){
        return r3;
    }

    unsigned long canMsg::getR4(){
        return r4;
    }

    unsigned long canMsg::getR5(){
        return r5;
    }

    void canMsg::printme(){
        std::ostringstream s(std::ostringstream::out);

        s << "[" << std::setbase(16) << id << "]{"
                 << std::setbase(16) << type << "/"
                 << std::setbase(16) << subType
                 << "} "; 
        s << "R1 = 0x" << std::hex << r1 << ", ";
        s << "R2 = 0x" << std::hex << r2 << ", ";
        s << "R3 = 0x" << std::hex << r3 << ", ";
        s << "R4 = 0x" << std::hex << r4 << ", ";
        s << "R5 = 0x" << std::hex << r5;
        RTT::Logger::log() << s.str() << RTT::endlog();
    }

    /* The bitStuffing algorithems are pulled directly from the original hubo
     * code base. They're wrong in that they use a sign bit instead of a two's
     * complement representation, but I'm maintaing them because they're needed
     * to ensure compatibility with the motor controllers */
    unsigned long canMsg::bitStuff15byte(long bs){
        if (bs < 0) return( (unsigned long)(((-bs) & 0x000007FF) | (1<<11)) );
        else	return( (unsigned long)(bs) );
    }

    unsigned long canMsg::bitStuff3byte(long bs){
        if (bs < 0) return( (unsigned long)(((-bs) & 0x007FFFFF) | (1<<23)) );
        else	return( (unsigned long)(bs) );
    }

    unsigned long canMsg::bitStuffCalibratePacket(long bs){
        if (bs < 0) return( (unsigned long)(((-bs) & 0x0007FFFF) | (1<<19)) );
        else	return( (unsigned long)(bs) );
    }

    unsigned char canMsg::bitStrip(unsigned long src, int byteNum){
        return (unsigned char)( (src >> (8*byteNum)) & 0x000000FFu );
    }

    canmsg_t* canMsg::toLineType(){
        canmsg_t* cm = new canmsg_t;
        cm->flags = 0;
        cm->cob = 0;
        //cm->timestamp = 0;
        
        switch(type){
            //How to construct the outbound packets
            case TX_CMD:
		cm->id = (unsigned char) type;
                processCMD(cm);
                break;
            case TX_SENSOR_CMD:
		cm->id = (unsigned char) type;
                cm->length = 1;
                cm->data[0] = 0;
                break;
            case TX_REF:
		cm->id = id + (unsigned char) type;
                processREF(cm);
                break;
            //Inbound packet types are not allowed here, and represent an error
            case SENSOR_FT_RXDF:
            case SENSOR_AD_RXDF:
            case ENC_RXDF:
            case CUR_RXDF:
            case PM_RXDF:
            case STAT_RXDF:
            case NAME_RXDF:
            case DAOFFSET_RXDF:
            case ADOFFSET_RXDF:
            case OFFSET_RXDF:
            default:
                assert(false);
        }
        return cm;
    }

    canmsg_t* canMsg::processCMD(canmsg_t* cm){
        cm->data[0] = (unsigned char)id;
        cm->data[1] = (unsigned char)subType;
        switch(subType){
            /*  Command messages are split into various register to byte combinations.  See
                comment above each block for mapping diagram. */

            /*******************************************************************
               |  B0  |  B1  |  B2  |  B3  |  B4  |  B5  |  B6  |  B7  |
                                                                   
               |  BNO |  sub |     r1      |     r2      |     r3      |
            *******************************************************************/

            case CMD_SET_POS_GAIN_0:
            case CMD_SET_POS_GAIN_1:
            case CMD_SET_CUR_GAIN_0:
            case CMD_SET_CUR_GAIN_1:
            case CMD_SET_ERR_BOUND:
            case CMD_SET_FT_0:
            case CMD_SET_FT_1:
            case CMD_SET_FT_2:
            case CMD_SET_INCLINO_SCALE:
                cm->data[2] = bitStrip(r1, 0);
                cm->data[3] = bitStrip(r1, 1);
                cm->data[4] = bitStrip(r2, 0);
                cm->data[5] = bitStrip(r2, 1);
                cm->data[6] = bitStrip(r3, 0);
                cm->data[7] = bitStrip(r3, 1);
                cm->length = 8;
                break;

            /*******************************************************************
               |  B0  |  B1  |  B2  |  B3  |  B4  |  B5  |  B6  |  B7  |
                                                                   
               |  BNO |  sub |  r1  |  --  |  --  |  --  |  --  |  --  |
            *******************************************************************/

            case CMD_SETREQ_BOARD_INFO:
            case CMD_REQ_ENC_POS:
            case CMD_RESET_ENC_ZERO:
            case CMD_HIP_ENABLE:
            case CMD_SET_CTRL_MODE:
            case CMD_SET_DEAD_ZONE:
            case CMD_REQ_BOARD_PARAM:
            case CMD_NULL:
            case CMD_SET_SWITCH:
            case CMD_REQ_ALARM:
            case CMD_REQ_BEEP:
                cm->data[2] = r1;
                cm->length = 3; 
                break;
          
            /*******************************************************************
               |  B0  |  B1  |  B2  |  B3  |  B4  |  B5  |  B6  |  B7  |
                                                                   
               |  BNO |  sub |  --  |  --  |  --  |  --  |  --  |  --  |
            *******************************************************************/
            
            case CMD_REQ_BOARD_STATUS:
            case CMD_REQ_CURR:
            case CMD_CONTROLLER_ON:
            case CMD_CONTROLLER_OFF:
            case CMD_INIT_BOARD:
            case CMD_REQ_VOLT_CUR:
            case CMD_REQ_TIME_STATUS:
            case CMD_CALIBRATE: 
                cm->length = 2;
                break;


        }
        return cm;
    }

    canmsg_t* canMsg::processREF(canmsg_t* cm){
        cm->length = 6;
        switch((int)subType){
            case 2:
                cm->data[0] = bitStrip(r1, 0);
                cm->data[1] = bitStrip(r1, 1);
                cm->data[2] = bitStrip(r1, 2);
                cm->data[3] = bitStrip(r2, 0);
                cm->data[4] = bitStrip(r2, 1);
                cm->data[5] = bitStrip(r2, 2);
                break;
            case 3:
                cm->data[0] = bitStrip(r1, 0);
                cm->data[1] = bitStrip(r1, 1);
                cm->data[2] = bitStrip(r2, 0);
                cm->data[3] = bitStrip(r2, 1);
                cm->data[4] = bitStrip(r3, 0);
                cm->data[5] = bitStrip(r3, 1);
                break;
            default:
                assert(false);
        }
        return cm;
    }
} 

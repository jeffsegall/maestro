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
#ifndef FTSENSORBOARD_H
#define FTSENSORBOARD_H

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <rtt/Component.hpp>
#include <hubomsg/typekit/HuboState.h>

using namespace RTT;

class FTSensorBoard {

    private:
        string name;
        
        double mX, mY, fZ;

        //PUBLISH 
        //OutputPort<hubomsg::HuboState>* huboDownPort;

        //SUBSCRIBE
        //InputPort<hubomsg::CanMessage>* canUpPort;

    public:

        FTSensorBoard();
        FTSensorBoard(string name);
   
        //PROTOCOL COMMAND MESSAGES
        /*
        void setRequestBoardInfo(char CANR);
        void requestBoardParameters(char PARM);
        void requestExecuteNULL(char EFS);
        void setFTMatrixConstant0(int SFT00, int SFT01, int SFT02);
        void setFTMatrixConstant1(int SFT10, int SFT11, int SFT12);
        void setFTMatrixConstant2(int SFT20, int SFT21, int SFT22);
        void setInclinoScaleFactor(int SIF0, int SIF1, int SIF2);
        void setNewBoardNumberFilterFreq(char NEW_BNO, int FREQ10);
        void initBoard();

        //PROTOCOL READ MESSAGES
        void requestFTTiltDigit();
        void requestFTScaleTiltScale();
        void requestFTScaleTiltDigit();
        void requestFTDigitTiltScale();
        void requestFTDigit();
        void requestFTScale();
        void requestTiltDigit();
        void requestTildScale();
        void requestGyroTempData();
        */

        //NEW DATA
        string getName();
        double getMX();
        double getMY();
        double getFZ();

        void update(double mX, double mY, double fZ);
        void setName(string name);
};

#endif

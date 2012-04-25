#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <sys/ioctl.h>
#include <assert.h>
#include <errno.h>
#include "huboCanDS.hpp"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

using namespace Hubo;
using namespace std;

char* strToSerial(string packet){
    char* data = new char[packet.length() + 1];

    strcpy(data, packet.c_str());

    data[packet.length()] = (char) 0x0D;
    
    return data;
}

vector<float> trajectoryValues(){
    vector<float> val;

    float f;

    ifstream is;
    is.open("./trajfile");

    while (!is.eof()){
        is >> f;
        val.push_back(f); 
    } 

    return val;
}

int main(){

    int channel;

    cout << "Attempting to connect to CAN hardware on /dev/ttyUSB0..." << endl;

    channel = open("/dev/ttyUSB0", O_RDWR | O_NONBLOCK);

    //Direction (0, -1)
    //Ratio (0, 10, 25)
    //Harmonic (0, 100)
    //EncoderSize (0, 4000)
    //Zero (0, -18100, 1)
    //Gains (SET_POS_GAIN_A, 200, 0, 500)
    //HIP
    //run
    cout << "Connected! Channel " << channel << endl;

    char* speed_packet = new char[3];
    char* echo_packet = new char[3];
    char* open_packet = new char[3];
    char* close_packet = new char[3];

    string name_info_str = "t0013000105";

    char* req_enc = new char[12];
    req_enc[0] = 't';
    req_enc[1] = '0';
    req_enc[2] = '0';
    req_enc[3] = '1';
    req_enc[4] = '3';
    req_enc[5] = '0';
    req_enc[6] = '0';
    req_enc[7] = '0';
    req_enc[8] = '3';
    req_enc[9] = '0';
    req_enc[10] = '0';
    req_enc[11] = (char)0x0D;

    string hip_off_str = "t0013000B00";
    string hip_str = "t0013000B01";

    char* pos_gain = new char[22];
    pos_gain[0] = 't';
    pos_gain[1] = '0';
    pos_gain[2] = '0';
    pos_gain[3] = '1';
    pos_gain[4] = '8';
    pos_gain[5] = '0';
    pos_gain[6] = '0';
    pos_gain[7] = '0';
    pos_gain[8] = '7';
    pos_gain[9] = '0';
    pos_gain[10] = '0';
    pos_gain[11] = 'C';
    pos_gain[12] = '8';
    pos_gain[13] = '0';
    pos_gain[14] = '0';
    pos_gain[15] = '0';
    pos_gain[16] = '0';
    pos_gain[17] = '0';
    pos_gain[18] = '1';
    pos_gain[19] = 'F';
    pos_gain[20] = '4';
    pos_gain[21] = (char)0x0D;

    string run_str = "t0012000E";
    string stop_str = "t0012000F";

    string enc_zero = "t001300060F";


    int dir = -1;
    int ppr = 25 * 100 * 4000;
    int setPoint = 500;
    float RAD2DEG = 3.14159 / 180.0;

    long data = (long)((setPoint * RAD2DEG) * dir * (ppr/360.));
    //long finalData = canMsg::bitStuff3byte(data);

    string set_str = "t0108CCCCCCCC88888888";
    string set2_str = "t01080000000000000000";
    string fbc_str = "t0013001000";
 
    write(channel, strToSerial("s8"), 3);
    write(channel, strToSerial("O"), 2);
//    write(channel, strToSerial("E"), 2);


    sleep(2);

    cout << "writing name info, fbc packet" << endl;
 
    write(channel, strToSerial(name_info_str), 12);
    //write(channel, strToSerial(fbc_str), 12);

    sleep(2);

    cout << "writing gains, hip, run packets" << endl;

    write(channel, pos_gain, 22);

    write(channel, strToSerial(hip_str), 12);
    write(channel, strToSerial(run_str), 10);

    sleep(5);

    string set_start = "t0108";

    int ticks = 0;
    char* hexbuf = new char[8];
    string hexstr = "";

    vector<float> trajVal = trajectoryValues();

    for (int i = 0; i < trajVal.size(); i++){

        ticks = (int)trajVal.at(i);
        sprintf(hexbuf, "%08X", ticks);
        hexstr = set_start + hexbuf + hexbuf;
        cout << hexstr << endl;
        if (write(channel, strToSerial(hexstr), 22) < 22)
            cout << "write error" << endl;
/*
    for (int i = 0; i < 100; i++){

       ticks += 20000;
       if (ticks >= 2147483647)
           ticks = 0;

       sprintf(hexbuf, "%08X", ticks);
        
       hexstr = set_start + hexbuf + hexbuf; 
       //cout << "hexstr = " << hexstr << endl;
       if (write(channel, strToSerial(hexstr), 22) < 22)
           cout << "write error" << endl;
/*

       if ((i % 2) == 0)
           if (write(channel, strToSerial(set_str), 22) < 0)
               cout << "write error" << endl;
       else
           if (write(channel, strToSerial(set2_str), 22) < 0)
               cout << "write 2 error" << endl;
*/       usleep(20 * 1000);
    }
    
    cout << "Writing hip off, stop packets" << endl;

    write(channel, strToSerial(hip_off_str), 12);
    write(channel, strToSerial(stop_str), 10);

    cout << "writing close packet" << endl;

    usleep(20 * 1000);

    write(channel, strToSerial(enc_zero), 12);

    write(channel, strToSerial("C"), 2);

 /* 
    char* rx_buffer = new char[100];

    while(1){
        cout << "writing req enc packet" << endl;
        if (write(channel, req_enc, 12) < 12)
            cout << "Write error!" << endl;

        int num_bytes = read(channel, rx_buffer, 100);
        if (num_bytes > 0){
            cout << "Got data of size: " << num_bytes << endl;
            for (int j = 0; j < num_bytes; j++)
              cout << rx_buffer[j];
 
            cout << endl; 
        }
        else{
            cout << "Read error" << endl;
        }
        sleep(1);
    }





/*    while(true){

        t = write(channel, &reqstate, 1);
        if (t > 0)
            cout << "Sent Enc Request Message." << endl;

        FD_ZERO(&fds);
        FD_SET(channel, &fds);
        if (select(channel+1, &fds, NULL, NULL, NULL) > 0)
            if (FD_ISSET(channel, &fds)){
        int rxSize = (int)read(channel, &rx, 1);
        cout << rxSize << " , " << errno << endl; 
        if (rxSize > 0){
            cout << "Got response..." << endl;
            canmsg_t* msg = new canmsg_t;
            msg->flags = rx.flags;
            msg->cob = rx.cob;
            msg->id = rx.id;
            msg->timestamp = rx.timestamp;
            msg->length = rx.length;
            for(int j = 0; j < msg->length; j++){
                msg->data[j] = rx.data[j];
                cout << "0x" << std::setbase(16) << (int)(msg->data[j]) << ", ";
            }
            cout << endl;
        }
        }
    
        sleep(1);
    }
  */  
    cout << "Disconnecting..." << endl;

    close(channel);


    cout << "Disconnected!" << endl;


}

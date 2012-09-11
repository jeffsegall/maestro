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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include "can4linux.h"
#include "huboCan.h"

//using namespace Hubo;
using namespace std;

void showCANStat(int fd)
{
CanStatusPar_t status;
char *m;

    ioctl(fd, CAN_IOCTL_STATUS, &status);
    switch(status.type) {
        case  CAN_TYPE_SJA1000:
            m = "NXP sja1000";
            break;
        case  CAN_TYPE_FlexCAN:
            m = "Freescale FlexCan";
            break;
        case  CAN_TYPE_TouCAN:
            m = "Freescale TouCAN";
            break;
        case  CAN_TYPE_82527:
            m = "I82527";
            break;
        case  CAN_TYPE_TwinCAN:
            m = "Infineon TwinCAN";
            break;
        case  CAN_TYPE_BlackFinCAN:
            m = "AD BlackFinCAN 537";
            break;
        case  CAN_TYPE_AT91SAM9263:
            m = "ATMEL AT91SAM9263";
            break;
    case CAN_TYPE_UNSPEC:
    default:
            m = "unknown";
            break;
    }

    printf(":: %s %4d %2d %2d %2d %2d %2d tx:%3d/%3d: rx:%3d/%3d:\n",
        m,
        status.baud,
        status.status,
        status.error_warning_limit,
        status.rx_errors,
        status.tx_errors,
        status.error_code,
        /* */
        status.tx_buffer_size,
        status.tx_buffer_used,
        status.rx_buffer_size,
        status.rx_buffer_used
        );
}

string ticksToString(int ticks){
    string hexstr;
    string revstr;

    int b0i = 255;
    int b1i = 65280;
    int b2i = 16711680;

    int a = abs(ticks);

    unsigned char b0 = a & b0i;
    unsigned char b1 = (a & b1i) >> 8;
    unsigned char b2 = (a & b2i) >> 16;

    unsigned char t = 0;
    if (ticks < 0)
        t = 128;

    char* hexbuf = new char[2];
    char* hexbuf2 = new char[2];
    char* hexbuf3 = new char[2];
    b2 = (b2 & 127) | t;
    sprintf(hexbuf, "%02X", b0);
    hexstr = hexbuf;
       
    sprintf(hexbuf2, "%02X", b1);
    hexstr += hexbuf2;
    
    sprintf(hexbuf3, "%02X", b2);
    hexstr += hexbuf3;

    revstr = hexbuf3;
    revstr += hexbuf2;
    revstr += hexbuf;

    revstr = revstr.substr(1);

    int hexToInt = 0;
    stringstream ss;
    ss << std::hex << revstr; 
    ss >> hexToInt;

    return hexstr; 
}

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
        val.push_back(f/5.0); 
    } 

    return val;
}

static int set_bitrate(int fd, int bitrate){
    //From can_send.c in the can4linux examples

    Config_par_t cfg;
    volatile Command_par_t cmd;
    int ret;

    cmd.cmd = CMD_STOP;
    ioctl(fd, CAN_IOCTL_COMMAND, &cmd);
    
    cfg.target = CONF_TIMING;
    cfg.val1 = (unsigned int)bitrate;
    ret = ioctl(fd, CAN_IOCTL_CONFIG, &cfg);
    
    cmd.cmd = CMD_START;
    ioctl(fd, CAN_IOCTL_COMMAND, &cmd);

    if (ret < 0) {
        perror("set_bitrate");
        exit(-1);
    } else {
        ret = 0;
    }
    return ret;
}

static int can_reset(int fd) {
    //From can_send.c
    volatile Command_par_t cmd;
    int ret;

    cmd.cmd = CMD_RESET;
    ret = ioctl(fd, CAN_IOCTL_COMMAND, &cmd);
    return ret;
}

static int can_start(int fd) {
    //From can_send.c
    volatile Command_par_t cmd;
    int ret;

    cmd.cmd = CMD_CLEARBUFFERS;
    ret = ioctl(fd, CAN_IOCTL_COMMAND, &cmd);
    cmd.cmd = CMD_START;
    ret = ioctl(fd, CAN_IOCTL_COMMAND, &cmd);
    return ret;
}

static canmsg_t message;

int main(){

    int channel;

    cout << "Attempting to connect to CAN hardware on /dev/can0..." << endl;

    channel = open("/dev/can0", O_RDWR | O_NONBLOCK);

    //Direction (0, -1)
    //Ratio (0, 10, 25)
    //Harmonic (0, 100)
    //EncoderSize (0, 4000)
    //Zero (0, -18100, 1)
    //Gains (SET_POS_GAIN_A, 200, 0, 500)
    //HIP
    //run
    cout << "Connected! Channel " << channel << endl;

    set_bitrate(channel, 1000);

    string name_info_str = "t0013000105";

    canMsg name_info = canMsg(BNO_R_HIP_YAW_ROLL, TX_MOTOR_CMD, CMD_SETREQ_BOARD_INFO,
                              0x05, 0, 0, 0, 0);

    string hip_off_str = "t0013000B00";
    string hip_str = "t0013000B01";

    canMsg hip_on = canMsg(BNO_R_HIP_YAW_ROLL, TX_MOTOR_CMD, CMD_HIP_ENABLE,
                           0x01, 0, 0, 0, 0);

    canMsg hip_off = canMsg(BNO_R_HIP_YAW_ROLL, TX_MOTOR_CMD, CMD_HIP_ENABLE,
                           0x00, 0, 0, 0, 0);

    string enc_zero = "t001300060F";

    canMsg run = canMsg(BNO_R_HIP_YAW_ROLL, TX_MOTOR_CMD, CMD_CONTROLLER_ON,
                        0, 0, 0, 0, 0);

    canMsg stop = canMsg(BNO_R_HIP_YAW_ROLL, TX_MOTOR_CMD, CMD_CONTROLLER_OFF,
                        0, 0, 0, 0, 0);

    canMsg encoder_zero = canMsg(BNO_R_HIP_YAW_ROLL, TX_MOTOR_CMD, CMD_RESET_ENC_ZERO,
                                 0x0F, 0, 0, 0, 0); 


    cout << "writing name info, fbc packet" << endl;
    
    can_start(channel);
 
    write(channel, name_info.toCAN(), 1);
    sleep(5);

    cout << "writing gains, hip, run packets" << endl;

    //write(channel, pos_gain, 22);

    hip_on.printme();
    run.printme();

    if (write(channel, hip_on.toCAN(), 1) < 1)
        cout << "hip_on error" << endl;
    if (write(channel, run.toCAN(), 1) < 1)
        cout << "run error" << endl;

    sleep(5);
    string set_start = "t0106";

    int ticks = 0;
    char* hexbuf = new char[8];
    string hexstr = "";
    string tickstring = "";
    vector<float> trajVal = trajectoryValues();

    canMsg ref_pos = canMsg(BNO_R_HIP_YAW_ROLL, TX_REF, (cmdType)2, 0, 0, 0, 0, 0);

    for (int i = 0; i < trajVal.size(); i++){
        
        ticks = (int)trajVal.at(i);

        ref_pos.setR1(ticks);
        ref_pos.setR2(ticks);
   
        //ref_pos.printme();
 
        if (write(channel, ref_pos.toCAN(), 1) < 1)
            cout << "write error" << endl;
  
        usleep(20000);
    }
  
    cout << "Writing hip off, stop packets" << endl;

    write(channel, hip_off.toCAN(), 1);
    write(channel, stop.toCAN(), 1);
    /*char* rx_buffer = new char[100];

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


    sleep(2);

    close(channel);

    cout << "Disconnected!" << endl;

    return 0;
}

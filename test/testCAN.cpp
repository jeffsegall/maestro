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
using std::cout;
using std::endl;
using namespace Hubo;

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

    open_packet[0] = 'O';
    open_packet[1] = (char)0x0D;

    close_packet[0] = 'C';
    close_packet[1] = (char)0x0D;

    speed_packet[0] = 's';
    speed_packet[1] = '8';
    speed_packet[2] = (char)0x0D;

    echo_packet[0] = 'E';
    echo_packet[1] = (char)0x0D;

    char* name_info = new char[12];
    name_info[0] = 't';
    name_info[1] = '0';
    name_info[2] = '0';
    name_info[3] = '1';
    name_info[4] = '3';
    name_info[5] = '0';
    name_info[6] = '0';
    name_info[7] = '0';
    name_info[8] = '1';
    name_info[9] = '0';
    name_info[10] = '5';
    name_info[11] = (char)0x0D;

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

    char* hip_off = new char[12];
    hip_off[0] = 't';
    hip_off[1] = '0';
    hip_off[2] = '0';
    hip_off[3] = '1';
    hip_off[4] = '3';
    hip_off[5] = '0';
    hip_off[6] = '0';
    hip_off[7] = '0';
    hip_off[8] = 'B';
    hip_off[9] = '0';
    hip_off[10] = '0';
    hip_off[11] = (char)0x0D;

    char* hip = new char[12];
    hip[0] = 't';
    hip[1] = '0';
    hip[2] = '0';
    hip[3] = '1';
    hip[4] = '3';
    hip[5] = '0';
    hip[6] = '0';
    hip[7] = '0';
    hip[8] = 'B';
    hip[9] = '0';
    hip[10] = '1';
    hip[11] = (char)0x0D;


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

    char* run = new char[10];
    run[0] = 't';
    run[1] = '0';
    run[2] = '0';
    run[3] = '1';
    run[4] = '2';
    run[5] = '0';
    run[6] = '0';
    run[7] = '0';
    run[8] = 'E';
    run[9] = (char)0x0D;    

    char* stop = new char[10];
    stop[0] = 't';
    stop[1] = '0';
    stop[2] = '0';
    stop[3] = '1';
    stop[4] = '2';
    stop[5] = '0';
    stop[6] = '0';
    stop[7] = '0';
    stop[8] = 'F';
    stop[9] = (char)0x0D;

    int dir = -1;
    int ppr = 25 * 100 * 4000;
    int setPoint = 500;
    float RAD2DEG = 3.14159 / 180.0;

    long data = (long)((setPoint * RAD2DEG) * dir * (ppr/360.));
    //long finalData = canMsg::bitStuff3byte(data);

    char* set = new char[22];
    set[0] = 't';
    set[1] = '0';
    set[2] = '1';
    set[3] = '0';
    set[4] = '8';
    set[5] = '0';
    set[6] = '1';
    set[7] = 'A';
    set[8] = '2';
    set[9] = '6';
    set[10] = '5';
    set[11] = '2';
    set[12] = '3';
    set[13] = '0';
    set[14] = '8';
    set[15] = 'B';
    set[16] = '2';
    set[17] = '3';
    set[18] = 'C';
    set[19] = '0';
    set[20] = '1';
    set[21] = (char)0x0D;
 
    char* set2 = new char[22];
    set2[0] = 't';
    set2[1] = '0';
    set2[2] = '1';
    set2[3] = '0';
    set2[4] = '8';
    set2[5] = '0';
    set2[6] = 'A';
    set2[7] = '2';
    set2[8] = 'B';
    set2[9] = '2';
    set2[10] = '6';
    set2[11] = 'B';
    set2[12] = '1';
    set2[13] = '0';
    set2[14] = '8';
    set2[15] = 'D';
    set2[16] = '2';
    set2[17] = 'E';
    set2[18] = '1';
    set2[19] = 'A';
    set2[20] = 'A';
    set2[21] = (char)0x0D;

    char* fbc = new char[12];
    fbc[0] = 't';
    fbc[1] = '0';
    fbc[2] = '0';
    fbc[3] = '1';
    fbc[4] = '3';
    fbc[5] = '0';
    fbc[6] = '0';
    fbc[7] = '1';
    fbc[8] = '0';
    fbc[9] = '0';
    fbc[10] = '0';
    fbc[11] = (char)0x0D; 
 
    write(channel, speed_packet, 3);
    write(channel, open_packet, 2);
//    write(channel, echo_packet, 2);
    sleep(2);

    cout << "writing name info, fbc packet" << endl;
 
    write(channel, name_info, 12);
    write(channel, fbc, 12);

    sleep(2);

    cout << "writing gains, hip, run packets" << endl;

//    write(channel, pos_gain, 22);
    write(channel, hip, 12);
    write(channel, run, 10);

    sleep(5);
/*
    cout << "writing set packet" << endl;

    for (int i = 0; i < 10; i++){
        if ((i % 2) == 0)
            if (write(channel, set, 22) < 0)
                cout << "write error" << endl;
        else
            if (write(channel, set2, 22) < 0)
                cout << "write error 2" << endl;
        sleep(2);

    }
*/
    cout << "Writing hip off, stop packets" << endl;

    write(channel, hip_off, 12);
    write(channel, stop, 10);

    cout << "writing close packet" << endl;

    sleep(2);

    write(channel, close_packet, 2);

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

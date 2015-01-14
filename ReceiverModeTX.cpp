/*
* Neato Controller Code
* Last edited: 4/26/2013
* Vijay Govindarajan and Alfredo Muniz
* Description: This code allows a controller mbed to communicate with the mbed connected to the Neato.
*/

#include "mbed.h"
#include "MRF24J40.h"
#include <string>

Serial pc(USBTX, USBRX);

//Controller uses potentiometers so voltage sense where joystick is
AnalogIn RightV(p17);
AnalogIn RightH(p18);
AnalogIn LeftH(p19);
AnalogIn LeftV(p20);

MRF24J40 mrf(p11, p12, p13, p14, p21);

// Used for sending and receiving
char txBuffer[128];
char rxBuffer[128];
int rxLen;

int L;
int R;
int S;
float calibrate_right;

// Timer
Timer timer;

/**
* Receive data from the MRF24J40.
*
* @param data A pointer to a char array to hold the data
* @param maxLength The max amount of data to read.
*/
int rf_receive(char *data, uint8_t maxLength)
{
    uint8_t len = mrf.Receive((uint8_t *)data, maxLength);
    uint8_t header[8]= {1, 8, 0, 0xA1, 0xB2, 0xC3, 0xD4, 0x00};

    if(len > 10) {
        //Remove the header and footer of the message
        for(uint8_t i = 0; i < len-2; i++) {
            if(i<8) {
                //Make sure our header is valid first
                if(data[i] != header[i])
                    return 0;
            } else {
                data[i-8] = data[i];
            }
        }
    }
    return ((int)len)-10;
}

/**
* Send data to another MRF24J40.
*
* @param data The string to send
* @param maxLength The length of the data to send.
*                  If you are sending a null-terminated string you can pass strlen(data)+1
*/
void rf_send(char *data, uint8_t len)
{
    //We need to prepend the message with a valid ZigBee header
    uint8_t header[8]= {1, 8, 0, 0xA1, 0xB2, 0xC3, 0xD4, 0x00};
    uint8_t *send_buf = (uint8_t *) malloc( sizeof(uint8_t) * (len+8) );

    for(uint8_t i = 0; i < len+8; i++) {
        //prepend the 8-byte header
        send_buf[i] = (i<8) ? header[i] : data[i-8];
    }
    //pc.printf("Sent: %s\r\n", send_buf+8);

    mrf.Send(send_buf, len+8);
    free(send_buf);
}
   
int main (void) {
    uint8_t channel = 2;
    //The wheels do not spin at same speed so calibration is required
    calibrate_right = 4/3;

    //Set the Channel. 0 is default, 15 is max
    mrf.SetChannel(channel);

    //Start the timer
    timer.start();

    while(true) {
        //Try to receive some data
        rxLen = rf_receive(rxBuffer, 128);
        if(rxLen > 0) {
            pc.printf("Received: %s\r\n", rxBuffer);
        }
        
        //Send some data every .1 second
        if(timer.read_ms() >= 100) {
            //Reset the timer to 0
            timer.reset();
            //Calibration to get the full range of joystick
            if(LeftV.read() <= 0.35 || LeftV.read() >= 0.65) {
                L = (0.5 - LeftV.read()) * 300;
                S = 300;
            }
            //If close to middle (.5+error) then do nothing
            else {
                L = 0;
                S = 0;
            }
            //More calibration
            if((1.0 - RightV.read()) <= 0.35 || (1.0 - RightV.read()) >= 0.65) {
                R = (0.5 - (1.0 - RightV.read())) * 300 * calibrate_right;
                S = 300;
            }
            else {
                R = 0;
            }
            
            
            //Add the setmotor command to the buffer
            sprintf(txBuffer, "setmotor %d %d %d", L, R, S);
                      
            //Send the buffer
            rf_send(txBuffer, strlen(txBuffer) + 1);
            pc.printf("Sent: %s\r\n", txBuffer);
        }
    }
}

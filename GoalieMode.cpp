/*
* Neato 2D Object Tracker Code
* Last edited: 4/26/2013
* Vijay Govindarajan and Alfredo Muniz
* Description: This code allows the Neato to move horizontally to block an object coming towards it.
*/

#include "mbed.h"
#include <string>

#define MODSERIAL_DEFAULT_RX_BUFFER_SIZE 1024   //One of these is not needed I think
#define MODSERIAL_DEFAULT_TX_BUFFER_SIZE 1024
#include "MODSERIAL.h"  //Normal Serial is limited to 16 byte buffers

MODSERIAL lidar(p9, p10);  // tx, rx
MODSERIAL pc(USBTX, USBRX); // tx, rx

/******************* Neato Communication *********************/
char c;
char s[5000];

int on;
int count;
int num1,num2,num3,num4;
int x1[359];
int x2[359];
int y1[359];
int y2[359];
int j;
int xball,yball;
int xsum,ysum;
int points;
int target;
int wallL,wallR,wallSum;
int ballFound;


// LEDs you can treat these as variables (led2 = 1 will turn led2 on!)
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

// Timer
Timer timer;

void getldsscan(void)
{
    /******** LIDAR DATA MANIPULATION ********/
    lidar.printf("getldsscan\n");
    //pc.printf("start\r\n");
    for (int i = 0; i < 363; i++) {
        //Scan the lidar serial line by line
        lidar.scanf("%[^\n]%*c",s); 
        //Take out the integers from the data
        count = sscanf(s,"%d,%d,%d,%d", &num1, &num2, &num3, &num4); 
        //If the range is over 5 meters than throw away data by pull down
        if(num2 >= 5000) {  
            num2 = 0;
        }
        //Throw away the first 4 data because part of lidar terminal
        if(num1 >= 0 && j == 5 && i >= 4) {
            //Convert polar coordinates to cartesian for more intuitive manipulation/filtering
            x1[num1] = num2*cos((double)num1*3.14159/180+3.14159);
            y1[num1] = num2*sin((double)num1*3.14159/180+3.14159);
        }
    }
    j++;
}

int main (void)
{
    //Neato communites using 115200 baud rate
    lidar.baud(115200);
    pc.baud(115200);

    on = 1;
    led4 = 1;
    j = 0;
    points = 0;
    xsum = 0;
    ysum = 0;
    target = 0;
    xball = 0;
    yball = 0;
    ballFound = 0;
    
    wait(1);
    //Neato needs testmode on to run commands
    lidar.printf("testmode on\n");
    pc.printf("testmode on\r\n");
    wait(1);
    //Rotate the lidar on the neato
    lidar.printf("setldsrotation on\n");
    pc.printf("setldsrotation on\r\n");


    //Start the timer
    timer.start();


    while(1) {
        
        //Send some data every 100ms
        if(timer.read_ms() >= 100) {
            //Reset the timer to 0
            timer.reset();
            // Toggle LED 2.
            led2 = led2^1;
            //Method for getting scan data
            getldsscan();

            //The lidar needs a couple of cycles before it can send usable data
            if(j==6) {
                //Detect right wall
                for (int i = 175; i<185; i++) {
                    wallSum = wallSum + x1[i];
                }
                wallR = wallSum/10;
                pc.printf("Right Wall = %d\r\n", wallR);
                wallSum = 0;
                
                //Detect Left Wall
                for (int i = 355; i<359; i++) {
                    wallSum = wallSum + x1[i];
                }
                for (int i = 0; i<6; i++) {
                    wallSum = wallSum + x1[i];
                }
                wallL = wallSum/10;
                pc.printf("Left Wall = %d\r\n", wallL);
                wallSum = 0;

                //Obstacle detection code
                for (int i=225; i<326; i++) {
                    //Checks variances, wall distances, and movement to decide if object
                    if (x1[i]<(wallR-450) &&
                            x1[i]>(wallL+450) &&
                            abs(x1[i+1]-x1[i])>20 &&
                            abs(x1[i+1]-x1[i])<100 &&
                            abs(y1[i+1]-y1[i])< 20 &&
                            x1[i]!=0) {
                        pc.printf("BALL: %d %d\r\n", x2[i], y2[i]);
                        xsum = xsum + x1[i];
                        ysum = ysum + y1[i];
                        points++;
                    }
                }
                xball = xsum/points;
                yball = ysum/points;
                pc.printf("Ball: %d %d\r\n", xball, yball);
                
                if (xball == 0 && yball == 0) {
                    ballFound = 0;
                } else {
                    ballFound = 1;
                }
                xsum = 0;
                ysum = 0;
                points = 0;
                //Because its really slow; we need to move to x as quick as possible
                target = -(xball + 100);
                pc.printf("Target = %d", target);
                
                //If the target is within the robot spacing then don't move
                if ((abs(target) <= 100) || (ballFound == 0)) {
                    target = 0;
                }
                //When found something, move. Notice wheels don't rotate at same speed
                if (ballFound) {
                    lidar.printf("setMotor %d %d %d\n", (int)target, (int)(target*4/3), 300);
                    pc.printf("setMotor %d %d %d\r\n", (int)target, (int)(target*4/3), 300);
                    wait(1);
                }
                
                //Stop the motors
                lidar.printf("setMotor %d %d %d\n", 0, 0, 0);                
                timer.reset();
                j = 5;
                ballFound = 0;
            }
        }
    }

}

/****** Data to Interpret ******
AngleInDegrees,DistInMM,Intensity,ErrorCodeHEX
0,221,1400,0
1,223,1396,0
2,228,1273,0
(. . .)
359,220,1421,0
*/

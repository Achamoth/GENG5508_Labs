//GENG5508, Semester 2, 2017
//Lab 7, Maze Navigation and Mapping
//Ammar Abu Shamleh & Zen Ly

#include "eyebot.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define size 180

//Converts degrees to radians
double degToRad(double deg) {
    return ((M_PI/180.0) * deg);
}

//Converts radians to degrees
double radToDeg(double rad) {
    return ((180.0/M_PI) * rad);
}

//With the robot in front of a wall, rotate the robot until it's parellel with the wall
void rotateUntilParallel() {

    //Start rotating robot left
    VWSetSpeed(0,-25);

    //Keep rotating robot until left sensor stops decreasing
    int last, cur;
    last = cur = PSDGet(1);
    while(true) {
        last = cur;
        cur = PSDGet(1);
        if(cur > last && cur < 320) {
            break;
        }
    }
    //Stop rotating
    VWSetSpeed(0,0);
}

void updateMap(int *dists, int x, int y, int phi, BYTE *img) {

    //Calculate robot's current position
    int sx, sy;
    sx = -y;
    sy = x;
    sy = 3600 - sy;
    sx /= 20;
    sy /= 20;

    // printf("x:%d\ty:%d\tsx:%d\tsy:%d\n",x,y,sx,sy);

    //Draw black point at sx, sy
    img[sy*size+sx] = BLACK;

    //Loop over all laser scan values
    for(int i=0; i<360; i++) {
        //Draw all points
        for(int j=0; j<dists[i]; j++) {
            //Work out dx and dy in robot coordinates
            int dx = (int) j * cos(degToRad(i+phi));
            int dy = (int) j * sin(degToRad(i+phi));

            //Hit position
            int hx = x + dx;
            int hy = y + dy;

            //Translate to image coordinates
            int ix, iy;
            ix = -hy;
            iy = hx;
            iy = 3600 - iy;
            ix /= 20;
            iy /= 20;

            // if(i==90)
                // printf("i:%d\thx:%d\thy:%d\tix:%d\tiy:%d\n",i,hx,hy,ix,iy);

            if(ix < 0 || iy < 0 || ix >= size || iy >= size) {
                continue;
            }

            //Draw black points at wall
            if(j == dists[i]-1) {
                img[iy*size+ix] = BLACK;
            }
            //Otherwise, draw white point (but don't overwrite any walls)
            else {
                if(img[iy*size+ix] == BLACK) {
                    ;
                }
                else {
                    img[iy*size+ix] = WHITE;
                }
            }
        }
    }
}

int main(int argc, char *argv[]) {
    //Map. Start everything off as gray.
    BYTE *img = (BYTE *) malloc(size*size*sizeof(BYTE));
    for(int i=0; i<size*size; i++) {
        img[i] = 0xAA;
    }

    //Get desired robot speed
    int linspeed;
    if(argc == 2) {
        linspeed = atoi(argv[1]);
    }
    else {
        linspeed = 250;
    }

    //Initialize LCD display and print first image
    LCDImageStart(0,0,size,size);
    LCDImageGray(img);

    //Set starting position
    VWSetPosition(290,-290,0);

    //Keep driving until reaching start position. Follow left wall at roughly 20cm distance
    VWSetSpeed(linspeed,0);
    while(true) {
        //Check PSD sensors
        int left = PSDGet(1);
        int front = PSDGet(2);
        // int right = PSDGet(3);
        // printf("Left:%d\tFront:%d\t\n", left, front);

        //Check distance
        if(front < 300) {
            //Turn right until left side of robot is roughly parallel with wall
            VWTurn(-60,-30);
            VWWait();
            rotateUntilParallel();
        }
        else if(left > 300 && left < 500) {
            //Move in towards wall. Turn left a bit
            VWSetSpeed(linspeed, 20);
        }
        else if(left < 280) {
            //Move away from wall. Turn right a bit
            VWSetSpeed(linspeed, -20);
        }
        else if(left > 500) {
            VWSetSpeed(linspeed, 40);
        }
        else if(front > 300) {
            //Keep driving straight
            VWSetSpeed(linspeed, 0);
        }

        //Get robot's current position
        int x, y, phi;
        VWGetPosition(&x, &y, &phi);

        //Perform laser scan
        int dists[360];
        SIMLaserScan(dists);

        //Overlay laser scan results on image
        updateMap(dists, x, y, phi, img);

        //Repaint image on LCD
        LCDImageGray(img);
    }
}

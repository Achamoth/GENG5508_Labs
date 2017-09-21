#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "eyebot.h"

//Set target position as globals
int TARG_X = 3500;
int TARG_Y = 3500;

//Converts degrees to radians
double toRadian(double deg) {
    return ((M_PI/180.0) * deg);
}

//Converts radians to degrees
double radToDeg(double rad) {
    return ((180.0/M_PI) * rad);
}

//Starting at a random position, drive straight until the robot is close to a wall. distFromWall specifies how close the robot should get to the wall
void reachWall(int distFromWall) {
    //Have robot drive straight to start off with
    VWSetSpeed(100,0);

    //Drive forward and continually check distance from wall in front
    while(true) {
        //Get distance from sensor in front
        int dist = PSDGet(2);
        //Check distance against threshold
        if(dist < distFromWall) {
            //Stop moving straight
            VWSetSpeed(0,0);
            break;
        }
    }
}

//With the robot in front of a wall, rotate the robot until it's parellel with the wall
void rotateUntilParallel() {

    //Start rotating robot left
    VWSetSpeed(0,15);

    //Keep rotating robot until front and right sensors are roughly same distance from the wall
    int last, cur;
    last = cur = PSDGet(3);
    while(true) {
        last = cur;
        cur = PSDGet(3);
        if(cur > last && cur < 320) {
            break;
        }
    }
    //Stop rotating
    VWSetSpeed(0,0);
}

//Find angle to target
int angleToTarget() {
    //Get robot's current position
    int x, y, phi;
    VWGetPosition(&x, &y, &phi);

    //Get diff between target and robot
    int dx = TARG_X - x;
    int dy = TARG_Y - y;

    //Get angle between robot position and target
    double pheta = radToDeg(atan2(dy, dx));

    //Calculate pheta prime (absolute angle betwee robot orientation and target)
    double phetaPrime = phi + pheta;

    return (int) phetaPrime;
}

//Starting parallel to a wall (with the wall on the right of the vehicle), keep driving straight and correcting course
//distToMaintain indicates the distance to try and maintain from the wall on the right (that the vehicle is driving along)
int driveAlongRightWall(int distToMaintain) {

    //Check angle to target
    int x, y, phi;
    VWGetPosition(&x, &y, &phi);
    int angle = angleToTarget();

    int startx, starty;

    //Check freespace distance in direction of target
    int dists[360];
    SIMLaserScan(dists);
    int F;
    if(angle <= 0)
        F = dists[abs(angle)];
    else
        F = dists[360-angle];
    int minF = F;

    //Check distance to target
    int distToTarget = (int)sqrt(pow(abs(TARG_X-x),2) + pow(abs(-1*TARG_Y-y),2));
    int minDist = distToTarget;

    //Start driving straight
    VWSetSpeed(100,0);
    while (true) {

        //Check angle to target
        VWGetPosition(&x, &y, &phi);
        int angle = angleToTarget();

        //Check freespace distance in direction of target
        SIMLaserScan(dists);
        int F;
        if(angle <= 0)
            F = dists[abs(angle)];
        else
            F = dists[360-angle];

        //Update minimum freespace distance in direction of target
        if(F < minF) {
            minF = F;
        }

        //Check distance to target and update minimum distance
        int distToTarget = (int)sqrt(pow(abs(TARG_X-x),2) + pow(abs(-1*TARG_Y-y),2));
        if(distToTarget < minDist) {
            minDist = distToTarget;
        }

        printf("x=%d\ty=%d\tangle=%d\tF=%d\n", x, y, angle, F);

        //Check distance from wall on right
        int dist = PSDGet(3);
        //Check against threshold
        if(dist < distToMaintain-15) {
            //Too close to wall: turn left
            VWSetSpeed(100,15);
        }
        else if(dist > distToMaintain+15) {
            //Too far from wall: turn right
            VWSetSpeed(100,-15);
        }
        else {
            //Drive straight
            VWSetSpeed(100,0);
        }

        //Check if the current obstacle should be departed from
        if(distToTarget <= F) {
            //Drive towards target. Exit loop
            printf("%d, %d\n", distToTarget, F);
            return 1;
        }
        else if(distToTarget-F <= minDist-1000) {
            //Drive towards target. Exit loop
            printf("%d, %d, %d\n", distToTarget, F, minDist);
            return 0;
        }
    }
}

int main() {
    //Set position
    VWSetPosition(0,0,0);

    while(true) {
        //Rotate to face goal (right hand corner) (-45 degrees)
        VWTurn(-angleToTarget(), 30);
        while(!VWDone()){};

        //Drive towards goal until 200mm away from obstacle
        reachWall(200);

        //Align with obstacle
        rotateUntilParallel();

        //Now drive along obstacle, and maintain distance from obstacle
        int res = driveAlongRightWall(200);

        //Drive straight for a short distance
        VWStraight(250,100);
        while(!VWDone()) {};

        if(res == 1) break;
    }

    //Turn towards target
    printf("Turning %d degrees to the right\n", angleToTarget());
    VWTurn(-angleToTarget(), 30);
    while(!VWDone()) {};

    //Drive towards target
    int x, y, phi;
    VWGetPosition(&x, &y, &phi);
    int distToTarget = (int)sqrt(pow(abs(TARG_X-x),2) + pow(abs(-1*TARG_Y-y),2));
    printf("Driving %d mm to target\n", distToTarget);
    VWStraight(distToTarget, 100);
    while(!VWDone()) {};

    //Return
    return 0;
}

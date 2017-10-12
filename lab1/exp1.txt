#include <stdio.h>
#include <stdlib.h>
#include "eyebot.h"

//Starting at a random position, drive straight until the robot is close to a wall. distFromWall specifies how close the robot should get to the wall
void reachWall(int distFromWall) {
    //Have robot drive straight to start off with
    VWSetSpeed(200,0);

    //Drive forward and continually check distance from wall in front
    while(true) {
        //Get distance from sensor in front
        int dist = PSDGet(2);
        //Check distance against threshold
        if(dist < distFromWall) {
            //Stop moving straight, and start turning robot
            VWSetSpeed(0,0);
            break;
        }
    }
}

//With the robot in front of a wall, rotate the robot until it's parellel with the wall
void rotateUntilParallel() {

    //Start rotating robot right
    VWSetSpeed(250,0);
    MOTORDrive(1,100);
    MOTORDrive(3,100);
    MOTORDrive(2,0);
    MOTORDrive(4,0);

    //Keep rotating robot until front and left sensors are roughly same distance from the wall
    while(abs(PSDGet(1) - PSDGet(2)) > 2) {
    }
    VWSetSpeed(0,0);
    MOTORDrive(1,100);
    MOTORDrive(2,100);
    MOTORDrive(3,100);
    MOTORDrive(4,100);

    //Now, turn right another 45-48 degrees (roughly)
    VWTurn(-48,22);
    while(!VWDone()) {
    }
}

void rotate90Right() {
    VWTurn(-90,30);
    while(!VWDone()) {

    }
}

void rotate90Left() {
    VWTurn(90,30);
    while(!VWDone()) {

    }
}

//Starting parallel to a wall (with the wall on the left of the vehicle), keep driving straight and correcting course, until hitting another wall
//distFromWall indicates how close to get to the next wall (i.e. when to stop)
//distToMaintain indicates the distance to try and maintain from the wall on the left (that the vehicle is driving along)
void driveAlongLeftWall(int distFromWall, int distToMaintain) {

    //Start driving straight
    VWSetSpeed(400,0);

    //Keep driving straight and correcting course until we reach the next wall
    while(PSDGet(2) > distFromWall) {
        //Check distance from wall on left
        int dist = PSDGet(1);
        //Check against threshold (should be roughly 200 from wall)
        if(dist < distToMaintain-5) {
            //Too close to wall: turn right
            MOTORDrive(1,100); //Right
            MOTORDrive(2,65); //Left
            MOTORDrive(3,100); //Right
            MOTORDrive(4,65); //Left
            MOTORSpeed(1,1000);
            MOTORSpeed(2,1000);
            MOTORSpeed(3,1000);
            MOTORSpeed(4,1000);
        }
        else if(dist > distToMaintain+5) {
            //Too far from wall: turn left
            MOTORDrive(1,65); //Right
            MOTORDrive(2,100); //Left
            MOTORDrive(3,65); //Right
            MOTORDrive(4,100); //Left
            MOTORSpeed(1,1000);
            MOTORSpeed(2,1000);
            MOTORSpeed(3,1000);
            MOTORSpeed(4,1000);
        }
    }
}

//Starting parallel to a wall (with the wall on the right of the vehicle), keep driving straight and correcting course, until hitting another wall
//distFromWall indicates how close to get to the next wall (i.e. when to stop)
//distToMaintain indicates the distance to try and maintain from the wall on the left (that the vehicle is driving along)
void driveAlongRightWall(int distFromWall, int distToMaintain) {

    //Start driving straight
    VWSetSpeed(400,0);

    //Keep driving straight and correcting course until we reach the next wall
    while(PSDGet(2) > distFromWall) {
        //Check distance from wall on left
        int dist = PSDGet(3);
        //Check against threshold (should be roughly 200 from wall)
        if(dist < distToMaintain-5) {
            //Too close to wall: turn left
            MOTORDrive(1,65); //Right
            MOTORDrive(2,100); //Left
            MOTORDrive(3,65); //Right
            MOTORDrive(4,100); //Left
            MOTORSpeed(1,1000);
            MOTORSpeed(2,1000);
            MOTORSpeed(3,1000);
            MOTORSpeed(4,1000);
        }
        else if(dist > distToMaintain+5) {
            //Too far from wall: turn right
            MOTORDrive(1,100); //Right
            MOTORDrive(2,65); //Left
            MOTORDrive(3,100); //Right
            MOTORDrive(4,65); //Left
            MOTORSpeed(1,1000);
            MOTORSpeed(2,1000);
            MOTORSpeed(3,1000);
            MOTORSpeed(4,1000);
        }
    }
}

//Experiments 1 and 2 combined. Find a wall, and then drive a rectangle continuously
void exp1and2() {
    //Start by driving towards a wall (until 250mm away from wall)
    reachWall(200);

    //Rotate robot to be parallel with wall
    rotateUntilParallel();

    //Now continue these steps in a loop to drive in a box
    while(true) {
        driveAlongLeftWall(200,200);
        rotate90Right();
    }

    //Now, stop robot
    VWSetSpeed(0,0);
}

//Experiments 1 and 3. Find a wall, drive to a corner, and then drive a lawnmower pattern
void exp1and3() {
    //Start by driving towards a wall (until 200mm away from wall)
    reachWall(200);

    //Rotate robot to be parallel with wall
    rotateUntilParallel();

    //Reach corner and turn right
    reachWall(200);
    rotate90Right();

    //Now, starting in this corner, drive a lawnmower pattern
    //Now continue these steps in a loop to drive in a lawnmower pattern
    bool right = true;
    int i=0;
    while(true) {
        if(right)
            //Drive along wall, maintaining specified distance (changes over time to produce lawnmower pattern)
            driveAlongLeftWall(200,200+(150*(i++)));
        else
            //Drive along wall, maintaining specified distance (changes over time to produce lawnmower pattern)
            driveAlongRightWall(200,200+(150*(i++)));

        //Rotate 90 degrees
        if(right)
            rotate90Right();
        else
            rotate90Left();

        //Drive straight a short distance (150mm)
        VWStraight(150,100);
        while(!VWDone()){
        }

        //Rotate 90 degrees again
        if(right)
            rotate90Right();
        else
            rotate90Left();

        //Rotate opposite direction next time
        right = !right;
    }

    //Now, stop robot
    VWSetSpeed(0,0);
}

int main() {

    //Experiments 1 and 2
    // exp1and2();

    //Experiments 1 and 3
    exp1and3();
}

//GENG5508 Lab 2, 2017 Sem 2
//Ammar Abu Shamleh

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "eyebot.h"

//Converts degrees to radians
double toRadian(double deg) {
    return ((M_PI/180.0) * deg);
}

//Converts radians to degrees
double radToDeg(double rad) {
    return ((180.0/M_PI) * rad);
}

//Given a desired position and pose, calculate and drive a spline curve between the robot's intitial position, and the desired position/orientation
void splineDrive(int x, int y, int alpha) {
    //Get robot's initial position and orientation
    int x0, y0, phi0;
    VWGetPosition(&x0, &y0, &phi0);

    //Write spline curve out to a csv file
    FILE *fout = fopen("curve.csv", "w+");

    //Loop to generate spline curve over timesteps (used 0.0625 as increment, since it's a power of 2, so there's no imprecision)
    for(float u=0.0; u<=1.00; u+=0.0625) {
        //Get current robot position
        int curx, cury, curphi;
        VWGetPosition(&curx, &cury, &curphi);

        //Polynomial equations to calculate next point on spline curve with current u value
        double H0 = 2*pow(u,3) - 3*pow(u,2) + 1;
        double H1 = -2*pow(u,3) + 3*pow(u,2);
        double H2 = pow(u,3) - 2*pow(u,2) + u;
        double H3 = pow(u,3) - pow(u,2);

        //Calculate magnitude
        double magnitude = sqrt(pow(x,2) + pow(y,2));

        //Calculate the next point on the spline curve
        double px = H0*x0 + H1*(x) + magnitude*H2*cos(toRadian(phi0)) + magnitude*H3*cos(toRadian(alpha));
        double py = H0*y0 + H1*(y) + magnitude*H2*sin(toRadian(phi0)) + magnitude*H3*sin(toRadian(alpha));

        //Find derivative with respect to u of spline curve (to find tangent at each point)
        double dH0 = 6*pow(u,2) - 6*u;
        double dH1 = -6*pow(u,2) + 6*u;
        double dH2 = 3*pow(u,2) - 4*u + 1;
        double dH3 = 3*pow(u,2) - 2*u;

        //Find x and y components of tangent
        double dpx = dH0*x0 + dH1*(x) + magnitude*dH2*cos(toRadian(phi0)) + magnitude*dH3*cos(toRadian(alpha));
        double dpy = dH0*y0 + dH1*(y) + magnitude*dH2*sin(toRadian(phi0)) + magnitude*dH3*sin(toRadian(alpha));

        //Calculate angle of tangent
        double nextPhi = radToDeg(atan2(dpy, dpx));

        //Calculate angle to turn (by adding angle of tangent, and current angle of vehicle)
        double angle = nextPhi - curphi;

        //Print data for debugging
        printf("u = %f\tpx = %.2f\tpy = %.2f\tdpx = %.2f\tdpy = %.2f\tnextphi = %.2f\t\tcurphi = %d\tangle = %.2f\n", u, px, py, dpx, dpy, nextPhi, curphi, angle);
        printf("u = %f\tx = %d\ty = %df\n\n",u, curx, cury);

        //Print current point to csv file
        fprintf(fout, "%f,%f,%f\n", px, py, nextPhi);

        //Find linear distance between two points (last point on curve and current point)
        double dist = sqrt(pow((px-curx),2) + pow((py-cury),2));

        //Drive along arc and shift rotation by desired angle using VWCurve
        if(u!=0.0) { //If u is 0, then dist and angle will be 0 (no movement), and the robot just disappears when you give these values to VWCurve (for some reason)
            //But I do want u = 0 to be captured in the loop, so the initial point shows up on the spline curve in the .csv file; hence the if statement
            VWCurve(dist, -1*angle, 30);
        }
        //Wait for drive to finish
        while(!VWDone()){
        }

        //Print final position and orientation (so it can be compared to desired position and orientation)
        if(u==1.0) {
            //Print final position and orientation
            VWGetPosition(&curx, &cury, &curphi);
            printf("u = %f\tx = %d\ty = %d\tangle = %d\n\n",u, curx, cury, curphi);
        }
    }

    //Close csv file
    fclose(fout);
}

//Drive between waypoints using spline curves
void waypointsSpline() {
    //Waypoints array
    int x[100];
    int y[100];
    int i=0;

    //Read waypoints file
    FILE *fp = fopen("way.txt","r");
    char str[100];
    while(fgets(str, 60, fp) != NULL) {
        int wpx, wpy;
        sscanf(str, "%d %d", &wpx, &wpy);
        x[i] = wpx;
        y[i++] = wpy;
    }
    //Close file
    fclose(fp);

    //Move between points
    while(true) {
        for(int j=0; j<=i; j++) {
            //Move to current waypoint using spline drive
            splineDrive(x[j],y[j],30);
        }
    }
}

//Drive between waypoints specified in "way.txt" text file. A bad drive method that just drives lines between each successive waypoint, rather than spline curves
void waypoints() {
    //Waypoints array
    int x[100];
    int y[100];
    int i=0;

    //Read file
    FILE *fp = fopen("way.txt","r");
    char str[100];
    while(fgets(str, 60, fp) != NULL) {
        int wpx, wpy;
        sscanf(str, "%d %d", &wpx, &wpy);
        x[i] = wpx;
        y[i++] = wpy;
    }
    fclose(fp);

    //Move between points
    int fin=0;
    while(true) {
        for(int j=0; j<=i; j++) {
            //Move to current waypoint
            int curx, cury, curphi;
            VWGetPosition(&curx, &cury, &curphi);

            //Calcualte angle and magnitude
            int angle = curphi - radToDeg(atan2(pow(y[j]-cury,2), pow(x[j]-curx,2)));
            int mag = sqrt(pow(x[j]-curx,2) + pow(y[j]-cury,2));
            if(j==i) {
                angle += 180;
            }
            //Turn
            VWTurn(angle, 30);
            printf("%d %d\n", curx, cury);
            printf("%d %d\n", x[j], y[j]);
            printf("%d\n\n", angle);
            while(!VWDone()) {
            }
            //Drive
            VWStraight(mag, 50);
            while(!VWDone()) {
            }
        }
    }
}

int main() {
    //Set initial position of vehicle to origin
    VWSetPosition(0,0,0);

    // splineDrive(100, 30, 90);
    // splineDrive(400,100,30);
    // splineDrive(100,100,30);
    // splineDrive(100,200,30);
    waypointsSpline();
}

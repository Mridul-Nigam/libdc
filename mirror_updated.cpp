#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include<chrono>
#include<vector>
#include "drdc.h"

struct Position {
    double x;
    double y;
    double z;
    int64_t sendTime;
    double dt;
};


int main(int  argc,
    char** argv)
{
    double tx, ty, tz;
    double time;
    int    done = 0;
    int    master, slave;

    // open and initialize 2 devices
for (int dev = 0; dev < 1; dev++) {

        // open device
        if (drdOpenID(dev) < 0) {
            printf("error: not enough devices found\n");
            dhdSleep(2.0);
            for (int j = 0; j <= dev; j++) drdClose(j);
            return -1;
        }

        // exclude some device types that have not been fully tested with 'mirror'
        bool incompatible = false;
        switch (dhdGetSystemType()) {
        case DHD_DEVICE_SIGMA331:
        case DHD_DEVICE_SIGMA331_LEFT:
            incompatible = true;
            break;
        }

        // check that device is supported
        if (incompatible || !drdIsSupported()) {
            printf("error: unsupported device (%s)\n", dhdGetSystemName(dev));
            dhdSleep(2.0);
            for (int j = 0; j <= dev; j++) drdClose(j);
            return -1;
        }

        // initialize Falcon by hand if necessary
        if (!drdIsInitialized() && dhdGetSystemType() == DHD_DEVICE_FALCON) {
            printf("please initialize Falcon device...\r"); fflush(stdout);
            while (!drdIsInitialized()) dhdSetForce(0.0, 0.0, 0.0);
            printf("                                  \r");
            dhdSleep(0.5);
        }

        // initialize if necessary
        if (!drdIsInitialized(dev) && (drdAutoInit(dev) < 0)) {
            printf("error: initialization failed (%s)\n", dhdErrorGetLastStr());
            dhdSleep(2.0);
            for (int j = 0; j <= dev; j++) drdClose(j);
            return -1;
        }

        // start robot control loop
        if (drdStart(dev) < 0) {
            printf("error: control loop failed to start properly (%s)\n", dhdErrorGetLastStr());
            dhdSleep(2.0);
            for (int j = 0; j <= dev; j++) drdClose(j);
            return -1;
        }
    }

    // default role assignment
    master = 1;
    slave = 0;

    // prefer Falcon as master
    if (dhdGetSystemType(0) != DHD_DEVICE_FALCON && dhdGetSystemType(1) == DHD_DEVICE_FALCON) {
        master = 0;
        slave = 1;
    }

    ushort mastersn, slavesn;
    dhdGetSerialNumber(&mastersn, master);
    dhdGetSerialNumber(&slavesn, slave);

    // center the devices
    drdMoveToPos(0.0, 0.0, 0.0, false, slave);

    // initialize slave target position to current position
    drdGetPositionAndOrientation(&tx, &ty, &tz, NULL, NULL, NULL, NULL, NULL, slave);

    // stop regulation on master, stop motion filters on slave
    drdStop(true, master);
    dhdSetForce(0.0, 0.0, 0.0, master);
    drdEnableFilter(true, slave);

    while (!done) {    
        Position receivedPosition;
        drdTrackPos(receivedPosition.x, receivedPosition.y, receivedPosition.z);
    }

// print stats and check for exit condition
    time = dhdGetTime();

    // report exit cause
    printf("                                                                           \r");
    if (done == -1) printf("\nregulation finished abnormally on slave device\n");
    else            printf("\nexiting on user request\n");

    // close the connection
    drdClose(slave);
    drdClose(master);

    // exit
    printf("\ndone.\n");
    return 0;
}

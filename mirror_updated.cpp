#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib") // Link with the Winsock library
#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include<chrono>
#include<vector>
#include "drdc.h"

#define DEFAULT_K_SLAVE   500.0
#define DEFAULT_K_BOX     500.0
#define SLAVE_BOX_SIZE      0.06
#define MIN_SCALE           0.2
#define MAX_SCALE           5.0

#define MIN(a,b) ((a)<(b))?(a):(b)
#define MAX(a,b) ((a)>(b))?(a):(b)
struct Position {
    double x;
    double y;
    double z;
    //int x;
    //int y;
    //int z;
    int64_t sendTime;
    double dt;
};

std::vector<double> receivedDataX;
std::vector<double> receivedDataY;
std::vector<double> receivedDataZ;

void recordDataToCSV(const std::string& filename) {
    std::ofstream outputFile(filename);
    if (!outputFile.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return;
    }



    // Write header to the CSV file
    outputFile << "X, Y, Z\n";

    // Write data to the CSV file
    for (size_t i = 0; i < receivedDataX.size(); ++i) {
        outputFile << receivedDataX[i] << ',' << receivedDataY[i] << ',' << receivedDataZ[i] << '\n';
    }

    std::cout << "Data recorded to " << filename << std::endl;
}


int main(int  argc,
    char** argv)
{
    double mx0, my0, mz0;
    double mx = 0.0, my = 0.0, mz = 0.0;
    double sx0, sy0, sz0 = 0.0;
    double sx, sy, sz;
    double tx, ty, tz;
    double fx, fy, fz;
    double time;
    double refTime = dhdGetTime();
    double Kslave = DEFAULT_K_SLAVE;
    double Kbox = DEFAULT_K_BOX;
    double scale = 1;
    bool   engaged = false;
    int    done = 0;
    int    master, slave;


    // message


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

    // give preference to omega.3 as slave
    /*if (dhdGetSystemType (0) == DHD_DEVICE_OMEGA3 && dhdGetSystemType (1) != DHD_DEVICE_OMEGA3) {
      master = 1;
      slave  = 0;
    }*/

    // if a device is virtual, make it the master
    /*if (dhdGetComMode (1) == DHD_COM_MODE_VIRTUAL) {
      master = 1;
      slave  = 0;
    }*/

    //dhdEmulateButton(DHD_ON, master);

    ushort mastersn, slavesn;
    dhdGetSerialNumber(&mastersn, master);
    dhdGetSerialNumber(&slavesn, slave);


    // center both devices
    //drdMoveToPos (0.0, 0.0, 0.0, false, master);//modified by MRIDUL - uncomment and comment next line to change
    //drdMoveToPos(0.0, 0.0, 0.0, false, master);
    drdMoveToPos(0.0, 0.0, 0.0, false, slave);
    //while (drdIsMoving (master) || drdIsMoving (slave)) drdWaitForTick (master); //modified by MRIDUL - uncomment and comment next line to change
    //while (drdIsMoving(master)) drdWaitForTick(master);

    // initialize slave target position to current position
    drdGetPositionAndOrientation(&tx, &ty, &tz, NULL, NULL, NULL, NULL, NULL, slave);

    // stop regulation on master, stop motion filters on slave
    drdStop(true, master);
    dhdSetForce(0.0, 0.0, 0.0, master);
    drdEnableFilter(true, slave);
    int flag = 0;
    // master slave loop

    // Initialize Winsock
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "Failed to initialize Winsock." << std::endl;
        return 1;
    }

    // Create a socket
    SOCKET serverSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (serverSocket == INVALID_SOCKET) {
        std::cerr << "Failed to create socket." << std::endl;
        WSACleanup();
        return 1;
    }

    // Bind the socket to a port
    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(65345); // Choose any available port
    serverAddress.sin_addr.s_addr = INADDR_ANY;

    if (bind(serverSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == SOCKET_ERROR) {
        std::cerr << "Failed to bind to port." << std::endl;
        closesocket(serverSocket);
        WSACleanup();
        return 1;
    }

    std::cout << "Server is listening on port 65435..." << std::endl;
    double time1 = 0.0;
    int fl = 0;
    int duration = 0;
    double ax, vx, jk;
    //Position old_position;
    //old_position.x = 0.0;
    //old_position.y = 0.0;
    //old_position.z = 0.0;
    while (!done) {
        //dhdGetPosition(&mx0, &my0, &mz0, master);
        //dhdGetPosition(&sx0, &sy0, &sz0, slave);
        ////std::cout << "we are in while loop";
        //// detect button press
        //if (!engaged && dhdGetButtonMask(master) != 0x00) {

        //    // store start position
        //    dhdGetPosition(&mx0, &my0, &mz0, master);
        //    dhdGetPosition(&sx0, &sy0, &sz0, slave);

        //    // enable slave control
        //    engaged = true;
        //}

        //// detect button release, disable slave control
        //else if (engaged && dhdGetButtonMask(master) == 0x00) engaged = false;

        //// if slave control is enabled, move the slave to match the master movement
        //if (5>3) {
        Position receivedPosition;
        // get master position
        //dhdGetPosition(&mx, &my, &mz, master);
        int clientAddressSize = sizeof(serverAddress);


        int bytesReceived = recvfrom(serverSocket, reinterpret_cast<char*>(&receivedPosition), sizeof(receivedPosition), 0,
            reinterpret_cast<struct sockaddr*>(&serverAddress), &clientAddressSize);
        //std::cout << receivedPosition.x;
        // compute target slave position
        //drdEnableFilter(true);
        drdTrackPos(receivedPosition.x, receivedPosition.y, receivedPosition.z);


        /*  if (old_position.x != receivedPosition.x && old_position.y != receivedPosition.y && old_position.z != receivedPosition.z)
          {
              drdMoveToPos(receivedPosition.x, receivedPosition.y, receivedPosition.z);
          }
          else {
              old_position.x = receivedPosition.x;
              old_position.y = receivedPosition.y;
              old_position.z = receivedPosition.z;
          }*/
          /*std::cout << drdIsFiltering();*/
        duration++;
        if (fl < 1) {
            time1 = drdGetTime();
            fl = 1;
        }
        //mx = receivedPosition.x;
        //my = receivedPosition.y;
        //mz = receivedPosition.z;
        // mx0 = receivedPosition.a;
        // my0 = receivedPosition.b;
        // mz0 = receivedPosition.c;
        // if (!receivedPosition.eng)
        // {
        //     dhdGetPosition(&sx0, &sy0, &sz0, slave);
        //     //printf("engaged = False \n");
        // }
        ///* if (receivedPosition.eng == 3) {
        //     printf("\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@22engaged= 3\n");
        // }*/
        //


        //printf("tracking position: %f %f %f",dt, tx, ty, tz );
        //printf("\ndata#: %d| cur: %f| recv: %f| tracking: %f %f %f| latency: %f s", duration,drdGetTime()-time1, receivedPosition.dt, receivedPosition.x, receivedPosition.y, receivedPosition.z, (drdGetTime() - time1)-receivedPosition.dt);
        // send slave to target position


    }
    //drdTrackPos(tx,ty,tz,false);
//}

//// apply forces if slave control is enabled
//if (engaged) {

//    // get current actual slave position
//    drdGetPositionAndOrientation(&sx, &sy, &sz, NULL, NULL, NULL, NULL, NULL, slave);

//    // compute force from slave regulation
//    fx = -Kslave * (tx - sx);
//    fy = -Kslave * (ty - sy);
//    fz = -Kslave * (tz - sz);

//    // add force from virtual authorized workspace
//    if (sx > SLAVE_BOX_SIZE / 2.0) fx += -Kbox * (sx - SLAVE_BOX_SIZE / 2.0) / scale;
//    if (sx < -SLAVE_BOX_SIZE / 2.0) fx += -Kbox * (sx - -SLAVE_BOX_SIZE / 2.0) / scale;
//    if (sy > SLAVE_BOX_SIZE / 2.0) fy += -Kbox * (sy - SLAVE_BOX_SIZE / 2.0) / scale;
//    if (sy < -SLAVE_BOX_SIZE / 2.0) fy += -Kbox * (sy - -SLAVE_BOX_SIZE / 2.0) / scale;
//    if (sz > SLAVE_BOX_SIZE / 2.0) fz += -Kbox * (sz - SLAVE_BOX_SIZE / 2.0) / scale;
//    if (sz < -SLAVE_BOX_SIZE / 2.0) fz += -Kbox * (sz - -SLAVE_BOX_SIZE / 2.0) / scale;
//}

//// otherwise, only apply gravity compensation
//else fx = fy = fz = 0.0;

//// apply force to master
//dhdSetForce(fx, fy, fz, master);

// print stats and check for exit condition
    time = dhdGetTime();
    if (time - refTime > 0.04)
    {
        /*printf("[%c] scale = %0.03f | K = %04d | master %0.02f kHz | slave %0.02f kHz            \r",
            (engaged ? '*' : ' '), scale, (int)Kslave, dhdGetComFreq(master), drdGetCtrlFreq(slave));
        refTime = time;*/
        //if (!drdIsRunning(slave)) done = -1;
        if (dhdKbHit())
        {
            switch (dhdKbGet())
            {
            case 'q': done = 1;   break;
            case 'k': Kslave -= 100.0; break;
            case 'K': Kslave += 100.0; break;
            case 's': if (!engaged) scale = MAX(MIN_SCALE, scale - 0.1); break;
            case 'S': if (!engaged) scale = MIN(MAX_SCALE, scale + 0.1); break;
            }
        }
    }


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


//#include <stdio.h>
//#include <stdlib.h>
//#define _USE_MATH_DEFINES
//#include <math.h>
//
//#include "drdc.h"
//
//#define DEFAULT_K_SLAVE   500.0
//#define DEFAULT_K_BOX     500.0
//#define SLAVE_BOX_SIZE      0.06
//#define MIN_SCALE           0.2
//#define MAX_SCALE           5.0
//
//#define MIN(a,b) ((a)<(b))?(a):(b)
//#define MAX(a,b) ((a)>(b))?(a):(b)
//
//
//
//int
//main (int  argc,
//      char **argv)
//{
//  double mx0, my0, mz0;
//  double mx, my, mz;
//  double sx0, sy0, sz0;
//  double sx, sy, sz;
//  double tx, ty, tz;
//  double fx, fy, fz;
//  double time;
//  double refTime = dhdGetTime ();
//  double Kslave = DEFAULT_K_SLAVE;
//  double Kbox   = DEFAULT_K_BOX;
//  double scale   = 1.0;
//  bool   engaged = false;
//  int    done    = 0;
//  int    master, slave;
//
//
//  // message
//  printf ("Force Dimension - Master Slave Example %s\n", dhdGetSDKVersionStr());
//  printf ("Copyright (C) 2001-2022 Force Dimension\n");
//  printf ("All Rights Reserved.\n\n");
//
//  // open and initialize 2 devices
//  for (int dev=0; dev<2; dev++) {
//
//    // open device
//    if (drdOpenID (dev) < 0) {
//      printf ("error: not enough devices found\n");
//      dhdSleep (2.0);
//      for (int j=0; j<=dev; j++) drdClose (j);
//      return -1;
//    }
//
//    // exclude some device types that have not been fully tested with 'mirror'
//    bool incompatible = false;
//    switch (dhdGetSystemType ()) {
//    case DHD_DEVICE_SIGMA331:
//    case DHD_DEVICE_SIGMA331_LEFT:
//      incompatible = true;
//      break;
//    }
//
//    // check that device is supported
//    if (incompatible || !drdIsSupported()) {
//      printf ("error: unsupported device (%s)\n", dhdGetSystemName(dev));
//      dhdSleep (2.0);
//      for (int j=0; j<=dev; j++) drdClose (j);
//      return -1;
//    }
//
//    // initialize Falcon by hand if necessary
//    if (!drdIsInitialized() && dhdGetSystemType() == DHD_DEVICE_FALCON) {
//      printf ("please initialize Falcon device...\r"); fflush(stdout);
//      while (!drdIsInitialized()) dhdSetForce (0.0, 0.0, 0.0);
//      printf ("                                  \r");
//      dhdSleep (0.5);
//    }
//
//    // initialize if necessary
//    if (!drdIsInitialized (dev) && (drdAutoInit (dev) < 0)) {
//      printf ("error: initialization failed (%s)\n", dhdErrorGetLastStr ());
//      dhdSleep (2.0);
//      for (int j=0; j<=dev; j++) drdClose (j);
//      return -1;
//    }
//
//    // start robot control loop
//    if (drdStart (dev) < 0) {
//      printf ("error: control loop failed to start properly (%s)\n", dhdErrorGetLastStr ());
//      dhdSleep (2.0);
//      for (int j=0; j<=dev; j++) drdClose (j);
//      return -1;
//    }
//  }
//
//  // default role assignment
//  master = 0;
//  slave  = 1;
//
//  // prefer Falcon as master
//  if (dhdGetSystemType (0) != DHD_DEVICE_FALCON && dhdGetSystemType (1) == DHD_DEVICE_FALCON) {
//    master = 1;
//    slave  = 0;
//  }
//
//  // give preference to omega.3 as slave
//  /*if (dhdGetSystemType (0) == DHD_DEVICE_OMEGA3 && dhdGetSystemType (1) != DHD_DEVICE_OMEGA3) {
//    master = 1;
//    slave  = 0;
//  }*/
//
//  // if a device is virtual, make it the master
//  /*if (dhdGetComMode (1) == DHD_COM_MODE_VIRTUAL) {
//    master = 1;
//    slave  = 0;
//  }*/
//
//  dhdEmulateButton(DHD_ON, master);
//
//  ushort mastersn, slavesn;
//  dhdGetSerialNumber (&mastersn, master);
//  dhdGetSerialNumber (&slavesn, slave);
//  printf ("%s haptic device [sn: %04d] as master\n", dhdGetSystemName(master), mastersn);
//  printf ("%s haptic device [sn: %04d] as slave\n", dhdGetSystemName(slave), slavesn);
//
//  // display instructions
//  printf ("\n");
//  printf ("press 's' to decrease scaling factor\n");
//  printf ("      'S' to increase scaling factor\n");
//  printf ("      'k' to decrease virtual stiffness\n");
//  printf ("      'K' to increase virtual stiffness\n");
//  printf ("      'q' to quit\n\n");
//
//  // center both devices
//  //drdMoveToPos (0.0, 0.0, 0.0, false, master);//modified by MRIDUL - uncomment and comment next line to change
//  drdMoveToPos (0.0, 0.0, 0.0, false, master);
//  drdMoveToPos (0.0, 0.0, 0.0, false,  slave);
//  //while (drdIsMoving (master) || drdIsMoving (slave)) drdWaitForTick (master); //modified by MRIDUL - uncomment and comment next line to change
//  while (drdIsMoving(master) ) drdWaitForTick(master);
//
//  // initialize slave target position to current position
//  drdGetPositionAndOrientation (&tx, &ty, &tz, NULL, NULL, NULL, NULL, NULL, slave);
//
//  // stop regulation on master, stop motion filters on slave
//  drdStop (true, master);
//  dhdSetForce (0.0, 0.0, 0.0, master);
//  drdEnableFilter (false, slave);
//  int flag = 0;
//  // master slave loop
//  while (!done) {
//    
//
//    // detect button press
//    if (!engaged && dhdGetButtonMask(master) != 0x00) {
//
//      // store start position
//      dhdGetPosition (&mx0, &my0, &mz0, master);
//      dhdGetPosition (&sx0, &sy0, &sz0, slave);
//
//      // enable slave control
//      engaged = true;
//    }
//
//    // detect button release, disable slave control
//    else if (engaged && dhdGetButtonMask(master) == 0x00) engaged = false;
//
//    // if slave control is enabled, move the slave to match the master movement
//    if (engaged) {
//
//      // get master position
//      dhdGetPosition (&mx, &my, &mz, master);
//      printf("sx sy sz %f %f %f", sx0, sy0, sz0);
//      // compute target slave position
//      tx = sx0 + scale*(mx - mx0);
//      ty = sy0 + scale*(my - my0);
//      tz = sz0 + scale*(mz - mz0);
//      printf("tracking position: %f %f %f\n", tx, ty, tz);
//      // send slave to target position
//      drdTrackPos (tx, ty, tz, slave);
//    }
//
//    // apply forces if slave control is enabled
//    if (engaged) {
//
//      // get current actual slave position
//      drdGetPositionAndOrientation (&sx, &sy, &sz, NULL, NULL, NULL, NULL, NULL, slave);
//
//      // compute force from slave regulation
//      fx = -Kslave * (tx-sx);
//      fy = -Kslave * (ty-sy);
//      fz = -Kslave * (tz-sz);
//
//      // add force from virtual authorized workspace
//      if (sx >  SLAVE_BOX_SIZE/2.0) fx += -Kbox * (sx -  SLAVE_BOX_SIZE/2.0) / scale;
//      if (sx < -SLAVE_BOX_SIZE/2.0) fx += -Kbox * (sx - -SLAVE_BOX_SIZE/2.0) / scale;
//      if (sy >  SLAVE_BOX_SIZE/2.0) fy += -Kbox * (sy -  SLAVE_BOX_SIZE/2.0) / scale;
//      if (sy < -SLAVE_BOX_SIZE/2.0) fy += -Kbox * (sy - -SLAVE_BOX_SIZE/2.0) / scale;
//      if (sz >  SLAVE_BOX_SIZE/2.0) fz += -Kbox * (sz -  SLAVE_BOX_SIZE/2.0) / scale;
//      if (sz < -SLAVE_BOX_SIZE/2.0) fz += -Kbox * (sz - -SLAVE_BOX_SIZE/2.0) / scale;
//    }
//
//    // otherwise, only apply gravity compensation
//    else fx = fy = fz = 0.0;
//
//    // apply force to master
//    dhdSetForce (fx, fy, fz, master);
//
//    // print stats and check for exit condition
//    time = dhdGetTime ();
//    if (time-refTime > 0.04) {
//      printf ("[%c] scale = %0.03f | K = %04d | master %0.02f kHz | slave %0.02f kHz            \r",
//          (engaged ? '*' : ' '), scale, (int)Kslave, dhdGetComFreq (master), drdGetCtrlFreq (slave));
//      refTime = time;
//      if (!drdIsRunning (slave)) done = -1;
//      if (dhdKbHit ()){
//        switch (dhdKbGet ()) {
//        case 'q': done = 1;   break;
//        case 'k': Kslave -= 100.0; break;
//        case 'K': Kslave += 100.0; break;
//        case 's': if (!engaged) scale = MAX(MIN_SCALE, scale-0.1); break;
//        case 'S': if (!engaged) scale = MIN(MAX_SCALE, scale+0.1); break;
//        }
//      }
//    }
//  }
//
//  // report exit cause
//  printf ("                                                                           \r");
//  if (done == -1) printf ("\nregulation finished abnormally on slave device\n");
//  else            printf ("\nexiting on user request\n");
//
//  // close the connection
//  drdClose (slave);
//  drdClose (master);
//
//  // exit
//  printf ("\ndone.\n");
//  return 0;
//}

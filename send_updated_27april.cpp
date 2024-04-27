#include "rtc/rtc.hpp"
#include "rtc/rtcpreceivingsession.hpp"
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

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
#include "drdc.h"
#include <iostream>
#include <mutex>

std::string libanswer;
std::string iceCandidate;
rtc::Configuration config;
int startDataTfr = 0;
int msgRecd = 1;
auto peer1 = rtc::PeerConnection(config);
int i = 1;
typedef websocketpp::server<websocketpp::config::asio> server;
std::string string;
char comma = ',';
double array[4] = {1.0,1.0,1.0,1.0};
int k = 1;
double mx, my, mz;


void on_open(server* s, websocketpp::connection_hdl hdl) {
	std::cout << "on_open string: " << string << std::endl;
	s->send(hdl, string, websocketpp::frame::opcode::text);
}

void on_close(server* s, websocketpp::connection_hdl hdl) {
	std::cout << "Connection closed" << std::endl;
}

void on_message(server* s, websocketpp::connection_hdl hdl, server::message_ptr msg) {
	if (msgRecd == 1) {
		libanswer = msg->get_payload();
		std::cout << "Received Message libanswer: " << libanswer << std::endl;
		peer1.setRemoteDescription(rtc::Description(libanswer));
		std::cout << "libanswer received" << std::endl;
		s->send(hdl, iceCandidate, websocketpp::frame::opcode::text);
		msgRecd++;
	}
	else if (msgRecd == 2) {
		startDataTfr = std::stoi(msg->get_payload());
		msgRecd++;
	}
	else {
		startDataTfr = std::stoi(msg->get_payload());

	}
}


#define MIN(a,b) ((a)<(b))?(a):(b)
#define MAX(a,b) ((a)>(b))?(a):(b)


int main()
{
    double mx0, my0, mz0;
    double sx0, sy0, sz0;
    double sx, sy, sz;
    double tx, ty, tz;
    double fx, fy, fz;
    double time;
    double refTime = dhdGetTime();
    double scale = 1.0;
    bool   engaged = false;
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
    master = 0;
    slave = 1;

    // prefer Falcon as master
    if (dhdGetSystemType(0) != DHD_DEVICE_FALCON && dhdGetSystemType(1) == DHD_DEVICE_FALCON) {
        master = 1;
        slave = 0;
    }

    dhdEmulateButton(DHD_ON, master);

    ushort mastersn, slavesn;
    dhdGetSerialNumber(&mastersn, master);
    dhdGetSerialNumber(&slavesn, slave);
    printf("%s haptic device [sn: %04d] as master\n", dhdGetSystemName(master), mastersn);
    printf("%s haptic device [sn: %04d] as slave\n", dhdGetSystemName(slave), slavesn);

    // display instructions
    printf("\n");
    printf("press 's' to decrease scaling factor\n");
    printf("      'S' to increase scaling factor\n");
    printf("      'k' to decrease virtual stiffness\n");
    printf("      'K' to increase virtual stiffness\n");
    printf("      'q' to quit\n\n");

    // center both devices
    drdMoveToPos(0.0, 0.0, 0.0, false, master);
    //drdMoveToPos(0.0, 0.0, 0.0, true, slave);
    while (drdIsMoving(master) || drdIsMoving(slave)) drdWaitForTick(master);

    // initialize slave target position to current position
    drdGetPositionAndOrientation(&tx, &ty, &tz, NULL, NULL, NULL, NULL, NULL, slave);

    // stop regulation on master, stop motion filters on slave
    drdStop(true, master);
    dhdSetForce(0.0, 0.0, 0.0, master);
    //drdEnableFilter(false, slave);

    struct forceData {
        double fX;
        double fY;
        double fZ;
        int64_t sendTime;
        double dt;
    };
    int encoders[DHD_MAX_DOF] = {};

    // master slave loop
    double time1 = drdGetTime();
    int duration = 0;
    dhdEnableExpertMode();


    peer1.onStateChange([](rtc::PeerConnection::State state) {
        std::cout << "State: " << state << std::endl;
        });


    peer1.onLocalDescription([](rtc::Description description) {

        string = std::string(description);
        std::cout << "string: " << string << "\n";
        //peer2.setRemoteDescription(description);
        });

    peer1.onLocalCandidate([](auto candidate) {
        std::cout << "candidate: " << candidate << "\n";
        iceCandidate = std::string(candidate);
        //peer2.addRemoteCandidate(candidate);
        });

    auto channel1 = peer1.createDataChannel("test");

    std::thread([&encoders, &scale, &done, &engaged, &master, &mx0, &my0, &mz0, &sx0, &sy0, &sz0, &slave, &tx, &ty, &tz, &fx, &fy, &fz, &time1]() {

        while (startDataTfr == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        while (!done && startDataTfr == 1) {

            // detect button press
            if (!engaged && dhdGetButtonMask(master) != 0x00) {
                printf("I m here");
                // store start position
                dhdGetPosition(&mx0, &my0, &mz0, master);
                dhdGetPosition(&sx0, &sy0, &sz0, slave);
                engaged = true;
            }

            // detect button release, disable slave control
            else if (engaged && dhdGetButtonMask(master) == 0x00)
            {
                engaged = false;
            }

            // if slave control is enabled, move the slave to match the master movement
            if (engaged) {
                printf("%f  %f  %f\n", mx0, my0, mz0);
                // get master position
                dhdGetPosition(&mx, &my, &mz, master);
                tx = sx0 + scale * (mx - mx0);
                ty = sy0 + scale * (my - my0);
                tz = sz0 + scale * (mz - mz0);
            }
            if (5 > 3) {
                dhdGetPosition(&mx, &my, &mz, master);
                std::cout << "k: " << k << "__" << std::endl;
                    //<<array[1] << "," << array[2] << "," << array[3] << std::endl;
                ++k;

            }



            fx = fy = fz = 0.0;
            // apply force to master
            dhdSetForce(fx, fy, fz, master);

        } }).detach();

    channel1->onOpen([&channel1]() {
        std::thread([&channel1]() {
            while (startDataTfr == 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            while (startDataTfr == 1) {

                array[0] = i;
                array[1] = mx;
                array[2] = my;
                array[3] = mz;
                ++i;
                    // Convert the array to a comma-separated string
                    std::stringstream ss;
                    
                        ss << array[0]<<comma << array[1] << comma << array[2] << comma << array[3];
                        //ss <<i << comma << mx << comma << my << comma << mz;
                        //std::cout<< array[0] << comma << mx << comma << my << comma << mz<<std::endl;
                        //std::this_thread::sleep_for(std::chrono::microseconds(1));
                    channel1->send(rtc::message_variant(ss.str()));
                    std::cout << ss.str()<<std::endl;
            }
            std::vector<double> array1;
            array1.push_back(0-i);
            array1.push_back(0);
            array1.push_back(0);
            array1.push_back(0);

            std::cout << "The array no. " << i << " sending is: ";
            for (const auto& element : array1) {
                std::cout << element << " ";
            }
            std::cout << std::endl;
            ++i;

            // Convert the array to a comma-separated string
            std::stringstream ss;
            for (size_t i = 0; i < array1.size(); ++i) {
                ss << array1[i];
                if (i < array1.size() - 1) {
                    ss << ",";
                }
            }
            channel1->send(rtc::message_variant(ss.str()));
            }).detach();

        });

	server ws_server;

	// Register message handler
	ws_server.set_message_handler(std::bind(&on_message, &ws_server, std::placeholders::_1, std::placeholders::_2));
	ws_server.set_close_handler(std::bind(&on_close, &ws_server, std::placeholders::_1));
	ws_server.set_open_handler(std::bind(&on_open, &ws_server, std::placeholders::_1));
	// Initialize the server
	ws_server.init_asio();
	ws_server.listen(9004);
	ws_server.start_accept();

	// Run the server
	ws_server.run();
	std::cin.ignore();

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
//#include <iostream>
//#include <fstream>
//#include <cstdlib>
//#include <cstring>
//#include <winsock2.h>
//#pragma comment(lib, "ws2_32.lib") // Link with the Winsock library
//#include <stdio.h>
//#include <stdlib.h>
//#define _USE_MATH_DEFINES
//#include <math.h>
//#include<chrono>
//#include<vector>
//#include "httplib.h"
//#include <nlohmann/json.hpp>
//#include "drdc.h"
//
//using json = nlohmann::json;
//
//#define DEFAULT_K_SLAVE   500.0
//#define DEFAULT_K_BOX     500.0
//#define SLAVE_BOX_SIZE      0.06
//#define MIN_SCALE           0.2
//#define MAX_SCALE           5.0
//
//#define MIN(a,b) ((a)<(b))?(a):(b)
//#define MAX(a,b) ((a)>(b))?(a):(b)
//struct Position {
//    double x;
//    double y;
//    double z;
//    //int x;
//    //int y;
//    //int z;
//    int64_t sendTime;
//    double dt;
//};
//
//std::vector<double> receivedDataX;
//std::vector<double> receivedDataY;
//std::vector<double> receivedDataZ;
//
//void recordDataToCSV(const std::string& filename) {
//    std::ofstream outputFile(filename);
//    if (!outputFile.is_open()) {
//        std::cerr << "Error: Unable to open file " << filename << std::endl;
//        return;
//    }
//
//
//
//    // Write header to the CSV file
//    outputFile << "X, Y, Z\n";
//
//    // Write data to the CSV file
//    for (size_t i = 0; i < receivedDataX.size(); ++i) {
//        outputFile << receivedDataX[i] << ',' << receivedDataY[i] << ',' << receivedDataZ[i] << '\n';
//    }
//
//    std::cout << "Data recorded to " << filename << std::endl;
//}
//
//
//int main(int  argc,
//    char** argv)
//{
//    double mx0, my0, mz0;
//    double mx = 0.0, my = 0.0, mz = 0.0;
//    double sx0, sy0, sz0 = 0.0;
//    double sx, sy, sz;
//    double tx, ty, tz;
//    double fx, fy, fz;
//    double time;
//    double refTime = dhdGetTime();
//    double Kslave = DEFAULT_K_SLAVE;
//    double Kbox = DEFAULT_K_BOX;
//    double scale = 1;
//    bool   engaged = false;
//    int    done = 0;
//    int    master, slave;
//
//
//    // message
//
//
//    // open and initialize 2 devices
//    for (int dev = 0; dev < 1; dev++) {
//
//        // open device
//        if (drdOpenID(dev) < 0) {
//            printf("error: not enough devices found\n");
//            dhdSleep(2.0);
//            for (int j = 0; j <= dev; j++) drdClose(j);
//            return -1;
//        }
//
//        // exclude some device types that have not been fully tested with 'mirror'
//        bool incompatible = false;
//        switch (dhdGetSystemType()) {
//        case DHD_DEVICE_SIGMA331:
//        case DHD_DEVICE_SIGMA331_LEFT:
//            incompatible = true;
//            break;
//        }
//
//        // check that device is supported
//        if (incompatible || !drdIsSupported()) {
//            printf("error: unsupported device (%s)\n", dhdGetSystemName(dev));
//            dhdSleep(2.0);
//            for (int j = 0; j <= dev; j++) drdClose(j);
//            return -1;
//        }
//
//        // initialize Falcon by hand if necessary
//        if (!drdIsInitialized() && dhdGetSystemType() == DHD_DEVICE_FALCON) {
//            printf("please initialize Falcon device...\r"); fflush(stdout);
//            while (!drdIsInitialized()) dhdSetForce(0.0, 0.0, 0.0);
//            printf("                                  \r");
//            dhdSleep(0.5);
//        }
//
//        // initialize if necessary
//        if (!drdIsInitialized(dev) && (drdAutoInit(dev) < 0)) {
//            printf("error: initialization failed (%s)\n", dhdErrorGetLastStr());
//            dhdSleep(2.0);
//            for (int j = 0; j <= dev; j++) drdClose(j);
//            return -1;
//        }
//
//        // start robot control loop
//        if (drdStart(dev) < 0) {
//            printf("error: control loop failed to start properly (%s)\n", dhdErrorGetLastStr());
//            dhdSleep(2.0);
//            for (int j = 0; j <= dev; j++) drdClose(j);
//            return -1;
//        }
//    }
//
//    // default role assignment
//    master = 1;
//    slave = 0;
//
//    // prefer Falcon as master
//    if (dhdGetSystemType(0) != DHD_DEVICE_FALCON && dhdGetSystemType(1) == DHD_DEVICE_FALCON) {
//        master = 0;
//        slave = 1;
//    }
//
//    ushort mastersn, slavesn;
//    dhdGetSerialNumber(&mastersn, master);
//    dhdGetSerialNumber(&slavesn, slave);
//    drdMoveToPos(0.0, 0.0, 0.0, false, slave);
//
//    drdGetPositionAndOrientation(&tx, &ty, &tz, NULL, NULL, NULL, NULL, NULL, slave);
//    drdStop(true, master);
//    dhdSetForce(0.0, 0.0, 0.0, master);
//    drdEnableFilter(true, slave);
//    int flag = 0;
//    double time1 = 0.0;
//    int fl = 0;
//    int duration = 0;
//    double ax, vx, jk;
//    httplib::Server svr;
//
//    // Define the route for POST requests to /receive_data
//    svr.Post("/receive_data", [](const httplib::Request& req, httplib::Response& res) {
//        // CORS headers
//        res.set_header("Access-Control-Allow-Origin", "*");
//        res.set_header("Access-Control-Allow-Methods", "POST, OPTIONS");
//        res.set_header("Access-Control-Allow-Headers", "Content-Type");
//
//    
//    try {
//        json requestData = json::parse(req.body);
//        json innerJson = json::parse(requestData.get<std::string>());
//
//        // Extract values from the inner JSON object
//        double x = innerJson["x"].get<double>();
//        double y = innerJson["y"].get<double>();
//        double z = innerJson["z"].get<double>();
//
//        // Perform your logic with the extracted values
//        // drdTrackPos(x, y, z);
//
//        res.status = 200;
//        res.set_content("Received POST data", "text/plain");
//    }
//    catch (const json::exception& e) {
//        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
//        res.status = 400;  // Bad Request
//        res.set_content("Error parsing JSON", "text/plain");
//    }
//        });
//
//    // Set CORS headers for all routes (optional)
//    svr.set_base_dir("C:/Users/91962/Desktop/Force Dimension/sdk-3.15.0/examples/CLI/mirror/");
//    svr.set_file_extension_and_mimetype_mapping("html", "text/html");
//    svr.set_mount_point("/mirror", "C:/Users/91962/Desktop/Force Dimension/sdk-3.15.0/examples/CLI/mirror/");
//
//        duration++;
//        if (fl < 1) {
//            time1 = drdGetTime();
//            fl = 1;
//        }
//        svr.listen("localhost", 8082);
//
//    
//
//    time = dhdGetTime();
//    if (time - refTime > 0.04)
//    {
//        if (dhdKbHit())
//        {
//            switch (dhdKbGet())
//            {
//            case 'q': done = 1;   break;
//            case 'k': Kslave -= 100.0; break;
//            case 'K': Kslave += 100.0; break;
//            case 's': if (!engaged) scale = MAX(MIN_SCALE, scale - 0.1); break;
//            case 'S': if (!engaged) scale = MIN(MAX_SCALE, scale + 0.1); break;
//            }
//        }
//    }
//
//
//    // report exit cause
//    printf("                                                                           \r");
//    if (done == -1) printf("\nregulation finished abnormally on slave device\n");
//    else            printf("\nexiting on user request\n");
//
//    // close the connection
//    drdClose(slave);
//    drdClose(master);
//
//    // exit
//    printf("\ndone.\n");
//    return 0;
//}


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

//#include <stdio.h>
//#include "dhdc.h"
//#define REFRESH_INTERVAL  0.1   // sec
//int
//main (int  argc,
//      char **argv)
//{
//  double px, py, pz;
//  double fx, fy, fz;
//  double t1,t0  = dhdGetTime ();
//  int    done   = 0;
//  // message
//  printf ("Force Dimension - Gravity Compensation Example %s\n", dhdGetSDKVersionStr());
//  printf ("Copyright (C) 2001-2022 Force Dimension\n");
//  printf ("All Rights Reserved.\n\n");
//  // open the first available device
//  if (dhdOpen () < 0) {
//    printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
//    dhdSleep (2.0);
//    return -1;
//  }
//  // identify device
//  printf ("%s device detected\n\n", dhdGetSystemName());
//  // display instructions
//  printf ("press 'q' to quit\n\n");
//  // enable force
//  dhdEnableForce (DHD_ON);
//  // haptic loop
//  while (!done) {
//      // apply zero force
//      if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR) {
//          printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
//          done = 1;
//      }
//      // display refresh rate and position at 10Hz
//      t1 = dhdGetTime();
//      if ((t1 - t0) > REFRESH_INTERVAL) {
//
//          // update timestamp
//          t0 = t1;
//          // retrieve position
//          if (dhdGetPosition(&px, &py, &pz) < DHD_NO_ERROR) {
//              printf("error: cannot read position (%s)\n", dhdErrorGetLastStr());
//              done = 1;
//          }
//          // retrieve force
//          if (dhdGetForce(&fx, &fy, &fz) < DHD_NO_ERROR) {
//              printf("error: cannot read force (%s)\n", dhdErrorGetLastStr());
//              done = 1;
//          }
//          // display status
//          printf("p (%+0.03f %+0.03f %+0.03f) m  |  f (%+0.01f %+0.01f %+0.01f) N  |  freq %0.02f kHz\r", px, py, pz, fx, fy, fz, dhdGetComFreq());
//          // user input
//          if (dhdKbHit() && dhdKbGet() == 'q') done = 1;
//      }
//  }
//  // close the connection
//  dhdClose ();
//  // happily exit
//  printf ("\ndone.\n");
//  return 0;
//}

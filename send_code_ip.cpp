///////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C) 2001-2022 Force Dimension, Switzerland.
//  All Rights Reserved.
//
//  Force Dimension SDK 3.15.0
//
///////////////////////////////////////////////////////////////////////////////
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <winsock2.h>
using namespace std;
#pragma comment(lib, "ws2_32.lib") // Link with the Winsock library
#include <chrono>
//modification done

// C++ library headers
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <iostream>

//#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "drdc.h"
#include "dhdc.h"

#define DEFAULT_K_SLAVE   500.0
#define DEFAULT_K_BOX     500.0
#define SLAVE_BOX_SIZE      0.06
#define MIN_SCALE           0.2
#define MAX_SCALE           5.0

#include "rtc/rtc.hpp"
#include "rtc/rtcpreceivingsession.hpp"
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <iostream>
#include <mutex>
#include <thread>

std::string libanswer;
std::string iceCandidate;
rtc::Configuration config;
int startDataTfr = 0;
int msgRecd = 1;
auto peer1 = rtc::PeerConnection(config);
int i = 1;
typedef websocketpp::server<websocketpp::config::asio> server;
std::string string1;

void on_open(server* s, websocketpp::connection_hdl hdl) {
    std::cout << "on_open string: " << string1 << std::endl;
    s->send(hdl, string1, websocketpp::frame::opcode::text);
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

void handle_http(server* s, websocketpp::connection_hdl hdl) {
    try {
        server::connection_ptr con = s->get_con_from_hdl(hdl);
        con->set_body("Server is running");
        con->set_status(websocketpp::http::status_code::ok);
    }
    catch (const std::exception& e) {
        std::cerr << "Exception in handle_http: " << e.what() << std::endl;
    }
}


#define MIN(a,b) ((a)<(b))?(a):(b)
#define MAX(a,b) ((a)>(b))?(a):(b)



int
main(int  argc,
    char** argv)
{
    double mx0, my0, mz0;
    double mx, my, mz;
    double sx0, sy0, sz0;
    double sx, sy, sz;

    double tx, ty, tz;
    double fx, fy, fz;
    double time;
    double refTime = dhdGetTime();
    double Kslave = DEFAULT_K_SLAVE;
    double Kbox = DEFAULT_K_BOX;
    double scale = 1.0;
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
        /*int fX;
        int fY;
        int fZ;*/
        double fX;
        double fY;
        double fZ;
        int64_t sendTime;
        double dt;
        //double fX0;
        //double fY0;
        //double fZ0;
        //bool eng;
    };
    int encoders[DHD_MAX_DOF] = {};
    // Initialize Winsock
    //WSADATA wsData;
    //if (WSAStartup(MAKEWORD(2, 2), &wsData) != 0) {
    //    std::cerr << "Failed to initialize Winsock." << std::endl;
    //    //return 1;
    //}

    //// Create a socket
    //SOCKET clientSocket = socket(AF_INET, SOCK_DGRAM, 0);
    //if (clientSocket == INVALID_SOCKET) {
    //    std::cerr << "Error creating socket: " << WSAGetLastError() << std::endl;
    //    WSACleanup();
    //    //return 1;
    //}

    //// Set up the server address structure
    //sockaddr_in serverAddress;
    //serverAddress.sin_family = AF_INET;
    //serverAddress.sin_port = htons(65345); // Choose a port number
    //serverAddress.sin_addr.s_addr = inet_addr("10.194.171.129");
    // master slave loop
    double time1 = drdGetTime();
    int duration = 0;
    dhdEnableExpertMode();

    peer1.onStateChange([](rtc::PeerConnection::State state) {
        //std::cout << "State: " << state << std::endl;
        });


    peer1.onLocalDescription([](rtc::Description description) {

        //string1 = std::string(description);
        //std::cout << "string: " << string1 << "\n";
        //peer2.setRemoteDescription(description);
        });

    peer1.onLocalCandidate([](auto candidate) {
        //std::cout << "candidate: " << candidate << "\n";
         iceCandidate = std::string(candidate);
        //peer2.addRemoteCandidate(candidate);
        });

    auto channel1 = peer1.createDataChannel("test");
    

    //createServer();
    server websocket_server;

    try {
        // Set logging level
        websocket_server.set_access_channels(websocketpp::log::alevel::all);
        websocket_server.clear_access_channels(websocketpp::log::alevel::frame_payload);
        std::cout << "creating server" << std::endl;
        // Register message handler
        websocket_server.set_message_handler(std::bind(&on_message, &websocket_server, std::placeholders::_1, std::placeholders::_2));

        // Initialize the server
        websocket_server.init_asio();

        // Bind the port
        websocket_server.listen(9002);

        websocket_server.set_http_handler(std::bind(&handle_http, &websocket_server, std::placeholders::_1));

        // Start the server accept loop in a separate thread
        std::thread server_thread([&]() {
            websocket_server.start_accept();
            std::cout << "running server" << std::endl;
            websocket_server.run();
            std::cout << "server running" << std::endl;
            });

        // Detach the server thread to allow it to run independently
        server_thread.detach();

        // Now you can execute code here while the server is running in its own thread


    }
    catch (const std::exception& e) {
        std::cout << e.what() << std::endl;
    }


   
    while (!done) {

        // detect button press
        if (!engaged && dhdGetButtonMask(master) != 0x00) {
            printf("I m here");
            // store start position
            dhdGetPosition(&mx0, &my0, &mz0, master);

            dhdGetPosition(&sx0, &sy0, &sz0, slave);
            //forceData fd = { mx0,my0,mz0,mx0,my0,mz0,3 };
            //sendto(clientSocket, reinterpret_cast<const char*>(&fd), sizeof(forceData), 0,
            //    reinterpret_cast<struct sockaddr*>(&serverAddress), sizeof(serverAddress));
            // enable slave control
            engaged = true;
        }

        // detect button release, disable slave control
        else if (engaged && dhdGetButtonMask(master) == 0x00) {
            engaged = false;

            //forceData fd = { mx,my,mz,mx0, my0, mz0, engaged };
            //sendto(clientSocket, reinterpret_cast<const char*>(&fd), sizeof(forceData), 0,
            //    reinterpret_cast<struct sockaddr*>(&serverAddress), sizeof(serverAddress));
        }

        // if slave control is enabled, move the slave to match the master movement
        if (engaged) {
            printf("%f  %f  %f\n", mx0, my0, mz0);
            // get master position
            dhdGetPosition(&mx, &my, &mz, master);
            //forceData fd = { mx,my,mz,mx0, my0, mz0, engaged};
            /*sendto(clientSocket, reinterpret_cast<const char*>(&fd), sizeof(forceData), 0,
                reinterpret_cast<struct sockaddr*>(&serverAddress), sizeof(serverAddress));*/
                // compute target slave position
            tx = sx0 + scale * (mx - mx0);
            ty = sy0 + scale * (my - my0);
            tz = sz0 + scale * (mz - mz0);

            /*printf("tracked position: %f %f %f", tx, ty, tz);*/
            // send slave to target position
            //drdTrackPos(tx, ty, tz, slave);
        }
        if (5 > 3) {
            dhdGetPosition(&mx, &my, &mz, master);
            //double dt = drdGetTime();
            //forceData fd = { mx,my,mz,dt-time1};


            dhdGetForce(&fx, &fy, &fz);
            if (dhdGetEnc(encoders) < 0)
            {
                std::cout << "error: failed to read encoders (" << dhdErrorGetLastStr() << ")" << std::endl;
                break;
            }
            auto sendTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            dhdGetPosition(&mx, &my, &mz);
            forceData fd = { mx,my,mz, sendTime, drdGetTime() - time1 };
            //forceData fd = { encoders[0], encoders[1], encoders[2], sendTime, drdGetTime() - time1};
            //forceData fd = { fx, fy, fz, dt-time1 };
            /*sendto(clientSocket, reinterpret_cast<const char*>(&fd), sizeof(forceData), 0,
                reinterpret_cast<struct sockaddr*>(&serverAddress), sizeof(serverAddress));*/
            duration++;
            //if (duration < 2) {
                printf("\ndata# %d curr: %f sent position: %d % d % d ", duration, drdGetTime() - time1, encoders[0], encoders[1], encoders[2]);
            dhdSleep(0.02);
            //printf("%f sent position: % f % f % f \n", dt - time1, fx, fy,fz);
            //printf("%f sent position: % f % f % f \n", dt-time1 ,mx, my, mz);
        }
        /*if (!engaged) {
            dhdGetPosition(&mx0, &my0, &mz0, master);
            forceData fd = { mx0, my0, mz0,engaged };
            sendto(clientSocket, reinterpret_cast<const char*>(&fd), sizeof(forceData), 0,
                reinterpret_cast<struct sockaddr*>(&serverAddress), sizeof(serverAddress));
        }*/

        // apply forces if slave control is enabled
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

        // otherwise, only apply gravity compensation
        /*else fx = fy = fz = 0.0;*/
        fx = fy = fz = 0.0;
        // apply force to master
        dhdSetForce(fx, fy, fz, master);

        // print stats and check for exit condition
        time = dhdGetTime();
        if (time - refTime > 0.04) {
            printf("[%c] scale = %0.03f | K = %04d | master %0.02f kHz | slave %0.02f kHz            \r",
                (engaged ? '*' : ' '), scale, (int)Kslave, dhdGetComFreq(master), drdGetCtrlFreq(slave));
            refTime = time;
            //if (!drdIsRunning(slave)) done = -1;
            if (dhdKbHit()) {
                switch (dhdKbGet()) {
                case 'q': done = 1;   break;
                case 'k': Kslave -= 100.0; break;
                case 'K': Kslave += 100.0; break;
                case 's': if (!engaged) scale = MAX(MIN_SCALE, scale - 0.1); break;
                case 'S': if (!engaged) scale = MIN(MAX_SCALE, scale + 0.1); break;
                }
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

//#include <websocketpp/config/asio_no_tls.hpp>
//#include <websocketpp/server.hpp>
//#include <iostream>
//#include <sstream>
//#include <thread>
//#include <chrono>
//#include <json/json.h>
//#include <set>
//
//using namespace std;
//
//typedef websocketpp::server<websocketpp::config::asio> server;
//typedef server::connection_ptr connection_ptr;
//
//bool streaming = false; // Flag to control streaming
//
//struct Position {
//    double x;
//    double y;
//    double z;
//};
//
//void dataGenerationThread(server& echo_server, std::set<connection_ptr>& connections) {
//    double sequenceNumber = 0.005;
//
//    while (true) {
//
//        if (streaming) {
//            // Generate integer data
//            sequenceNumber++;
//            Position position;
//            position.x = sequenceNumber;
//            position.y = sequenceNumber + 1;
//            position.z = sequenceNumber + 2;
//
//            // Convert Position to JSON
//            Json::Value jsonData;
//            jsonData["x"] = position.x;
//            jsonData["y"] = position.y;
//            jsonData["z"] = position.z;
//            // Convert the integer to a string for display
//            std::ostringstream oss;
//            //oss << sequenceNumber;
//            oss << jsonData;
//
//            // Broadcast the data to all connected clients
//            for (auto it : connections) {
//                try {
//                    echo_server.send(it, oss.str(), websocketpp::frame::opcode::text);
//                }
//                catch (const websocketpp::lib::error_code& e) {
//                    cout << "Error sending data to client: " << e.message() << endl;
//                }
//            }
//        }
//
//        // Sleep to control the data generation rate
//        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 100 seq/second
//    }
//}
//
//int main() {
//    server echo_server;
//    std::set<connection_ptr> connections;
//
//    // Initialize the server
//    echo_server.init_asio();
//
//    // Register the message handler
//    echo_server.set_message_handler([&](websocketpp::connection_hdl hdl, server::message_ptr msg) {
//        // Handle incoming messages 
//        if (msg->get_payload() == "start_streaming") {
//            streaming = true;
//        }
//        else if (msg->get_payload() == "stop_streaming") {
//            streaming = false;
//        }
//        });
//
//    // Register the open handler
//    echo_server.set_open_handler([&](websocketpp::connection_hdl hdl) {
//        connections.insert(echo_server.get_con_from_hdl(hdl));
//        });
//
//    // Register the close handler
//    echo_server.set_close_handler([&](websocketpp::connection_hdl hdl) {
//        connections.erase(echo_server.get_con_from_hdl(hdl));
//        });
//
//    // Listen on port 9000
//    echo_server.listen(9000);
//
//    // Start the server accept loop
//    echo_server.start_accept();
//
//    // Run the ASIO io_service in a separate thread
//    std::thread ioThread([&]() {
//        echo_server.run();
//        });
//
//    // Start the data generation thread
//    std::thread dataThread(dataGenerationThread, std::ref(echo_server), std::ref(connections));
//
//    // Wait for the user to press Enter to stop the server
//    std::cout << "Press Enter to stop the server." << std::endl;
//    std::cin.ignore();
//
//    // Stop the streaming thread and join the data thread
//    streaming = false;
//    dataThread.join();
//
//    // Stop the server and join the io_thread
//    echo_server.stop();
//    ioThread.join();
//
//    return 0;
//}

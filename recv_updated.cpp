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

#include "rtc/rtc.hpp"
#include "rtc/rtcpreceivingsession.hpp"
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <iostream>
#include <mutex>
#include <thread>

std::string liboffer;
std::string libanswer;
std::string libIceCandidate;
rtc::Configuration config;
auto peer2 = rtc::PeerConnection(config);

typedef websocketpp::server<websocketpp::config::asio> server;
server ws_server;

struct Position {
    int i;
    double x;
    double y;
    double z;
};

int sendAnswer = 0;
int recvMsg = 1;
char comma;
Position receivedPosition;

void on_open(server* s, websocketpp::connection_hdl hdl) {
    std::cout << "librecvsocket opened" << std::endl;
    std::thread([s, hdl]() {
        while (sendAnswer != 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        }
        std::cout << "libanswer in thread" << libanswer << std::endl;
        s->send(hdl, libanswer, websocketpp::frame::opcode::text);
        }).detach();


}

void on_close(server* s, websocketpp::connection_hdl hdl) {
    std::cout << "Connection closed" << std::endl;
}

void on_message(server* s, websocketpp::connection_hdl hdl, server::message_ptr msg) {
    if (recvMsg == 1) {
        liboffer = msg->get_payload();
        std::cout << "Received Liboffer: " << liboffer << std::endl;
        recvMsg++;
        peer2.setRemoteDescription(rtc::Description(liboffer));
    }
    else if (recvMsg == 2) {
        libIceCandidate = msg->get_payload();
        std::cout << "Received LibiceCandidate: " << libIceCandidate << std::endl;
        peer2.addRemoteCandidate(rtc::Candidate(libIceCandidate));
    }
}

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

    //--------------------------------------------------------------------------------------------------
    peer2.onStateChange([](rtc::PeerConnection::State state) {
        std::cout << "State: " << state << std::endl;
        });

    peer2.onLocalCandidate([](auto candidate) {
        std::cout << "peer2 candidate: " << candidate << "\n";
        });

    std::shared_ptr<rtc::DataChannel> channel2;

    std::mutex printMutex;

    peer2.onDataChannel([&channel2, &printMutex](auto dc) {
        std::cout << "peer 2 got data channel\n";
        channel2 = dc;
        dc->onMessage([&printMutex](auto message) {
            std::unique_lock<std::mutex>(printMutex);

            //std::cout << "message from peer1 : " << std::get<std::string>(message) << "\n";

            std::stringstream ss(std::get<std::string>(message));
            ss >> receivedPosition.i >> receivedPosition.x >> receivedPosition.y >> receivedPosition.z;

            });
        });

    peer2.onLocalDescription([](rtc::Description liboffer) {
        libanswer = std::string(liboffer);
        std::cout << "libanswer: " << libanswer << "\n";
        sendAnswer = 1;
        });

    // Register message handler
    ws_server.set_message_handler(std::bind(&on_message, &ws_server, std::placeholders::_1, std::placeholders::_2));
    ws_server.set_close_handler(std::bind(&on_close, &ws_server, std::placeholders::_1));
    ws_server.set_open_handler(std::bind(&on_open, &ws_server, std::placeholders::_1));
    //Initialize the server
    ws_server.init_asio();
    ws_server.listen(9005);
    ws_server.start_accept();

    // Run the server
    ws_server.run();
    //---------------------------------------------------------------------------------------------------

    while (!done) {
        std::cout << receivedPosition.i << std::endl;
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

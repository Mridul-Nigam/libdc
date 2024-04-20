#include <iostream>
#include <cstring>
#include <cstdlib>
using namespace std;
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include "drdc.h"
#include "dhdc.h"

#include "rtc/rtc.hpp"
#include "rtc/rtcpreceivingsession.hpp"
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
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

#define MIN(a,b) ((a)<(b))?(a):(b)
#define MAX(a,b) ((a)>(b))?(a):(b)

int main(int  argc, char** argv)
{
    double mx0, my0, mz0;
    double mx, my, mz;
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

        string1 = std::string(description);
        std::cout << "string: " << string1 << "\n";
        //peer2.setRemoteDescription(description);
        });

    peer1.onLocalCandidate([](auto candidate) {
        std::cout << "candidate: " << candidate << "\n";
        iceCandidate = std::string(candidate);
        //peer2.addRemoteCandidate(candidate);
        });

    auto channel1 = peer1.createDataChannel("test");

    channel1->onOpen([&channel1, &encoders, &scale, &done, &engaged, &master, &mx0, &my0, &mz0, &sx0, &sy0, &sz0, &slave, &tx, &ty, &tz, &mx, &my, &mz, &fx, &fy, &fz, &time1]() {
        std::thread([&channel1, &encoders, &scale, &done, &engaged, &master, &mx0, &my0, &mz0, &sx0, &sy0, &sz0, &slave, &tx, &ty, &tz, &mx, &my, &mz, &fx, &fy, &fz, &time1]() {
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
                    dhdGetForce(&fx, &fy, &fz);
                    if (dhdGetEnc(encoders) < 0)
                    {
                        std::cout << "error: failed to read encoders (" << dhdErrorGetLastStr() << ")" << std::endl;
                        break;
                    }
                    auto sendTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                    dhdGetPosition(&mx, &my, &mz);
                    forceData fd = { mx,my,mz, sendTime, drdGetTime() - time1 };
                    //duration++;
                    //printf("\ndata# %d curr: %f sent position: %d % d % d ",duration, drdGetTime() - time1, encoders[0], encoders[1], encoders[2]);
                    dhdSleep(0.02);
                    std::vector<int> array;
                    array.push_back(i);
                    array.push_back(mx);
                    array.push_back(my);
                    array.push_back(mz);

                    std::cout << "The array no. " << i << " sending is: ";
                    for (const auto& element : array) {
                        std::cout << element << " ";
                    }
                    std::cout << std::endl;
                    ++i;

                    // Convert the array to a comma-separated string
                    std::stringstream ss;
                    for (size_t i = 0; i < array.size(); ++i) {
                        ss << array[i];
                        if (i < array.size() - 1) {
                            ss << ",";
                        }
                    }
                    channel1->send(rtc::message_variant(ss.str()));
                }

                fx = fy = fz = 0.0;
                // apply force to master
                dhdSetForce(fx, fy, fz, master);

            }
            }).detach();

        }); 

    server ws_server;
    std::cout << "creating server" << std::endl;
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
    std::cout << "server running"<<std::endl;
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

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

int sendAnswer = 0;
int recvMsg = 1;

void on_open(server* s, websocketpp::connection_hdl hdl) {
	std::cout << "librecvsocket opened" << std::endl;
	std::thread([s, hdl]() {
		while (sendAnswer!=1) {
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



int main()
{
	
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

	std::cout << "message from peer1 : " << std::get<std::string>(message) << "\n";
	
	

	
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
		
		
	

	std::cin.ignore();

	return 0;
}

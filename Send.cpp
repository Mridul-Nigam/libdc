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
std::string string;
void on_open(server* s, websocketpp::connection_hdl hdl) {
	std::cout << "on_open string: " << string<<std::endl;
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

int main()
{
	

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
	
	channel1->onOpen([&channel1]() {
		std::thread([&channel1]() {
		while(startDataTfr==0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	while (startDataTfr == 1) {
		std::vector<int> array;
		array.push_back(i);
		array.push_back(2);
		array.push_back(3);
		array.push_back(4);
		array.push_back(5);
		array.push_back(6);

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

	return 0;
}

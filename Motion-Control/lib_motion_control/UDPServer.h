// Header guard
#ifndef UDP_SERVER_H
#define UDP_SERVER_H

// Include directives for Wi-Fi transmission
#include "ESP8266WiFi.h"
#include "WiFiUdp.h"


//==============================================================================================================
//=                                        INTERFACE FOR WI-FI TRANSMISSION                                    =
//==============================================================================================================
class UDPServer {
public:
    // Enumeration describing server states
    enum State {
        INACTIVE,
        WAITING_FOR_REQUEST,
        READY_TO_SEND
    };

    // Constructor
    UDPServer(unsigned int local_udp_port);

    // Accesors
    State getState();
    void setState(State state);

    // Initialization method
    bool initialize();

    // Communication methods
    void listen();
    void sendPacket(char* packet, int packet_size);


private:
    const unsigned int LOCAL_UDP_PORT; // Port number by which the server can be found
    WiFiUDP udp;                       // Object for UDP communication
    char incoming_packet[1];           // UDP packet to store requests
    State state;                       // Current state of the server
};


// Header guard
#endif

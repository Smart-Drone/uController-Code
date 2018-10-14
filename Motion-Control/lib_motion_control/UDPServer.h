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
        SENDING
    };

    // Constructor
    UDPServer(const char* wifi_ssid, const char* wifi_pwd, unsigned int local_udp_port);

    // Accesors
    State getState();

    // Initialization method
    bool initialize();

    // Communication methods
    void listen();
    void sendPacket(char* packet, int packet_size);


private:
    const char* WIFI_SSID;             // The server will try to connect to this network
    const char* WIFI_PWD;              // This password will be used for connecting to the network
    const unsigned int LOCAL_UDP_PORT; // Port number by which the server can be found
    WiFiUDP udp;                       // Object for UDP communication
    char incoming_packet[1];           // UDP packet to store requests
    State state;                       // Current state of the server
};


// Header guard
#endif

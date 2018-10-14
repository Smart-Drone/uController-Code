#include "UDPServer.h"


//==============================================================================================================
//=                                        CONSTRUCTOR                                                         =
//==============================================================================================================
UDPServer::UDPServer(
    const char* wifi_ssid,
    const char* wifi_pwd,
    unsigned int local_udp_port
)
: WIFI_SSID(wifi_ssid)
, WIFI_PWD(wifi_pwd)
, LOCAL_UDP_PORT(local_udp_port)
, udp()
, state(INACTIVE) {

}


//==============================================================================================================
//=                                        ACCESSORS                                                           =
//==============================================================================================================
UDPServer::State UDPServer::getState() {
    return this->state;
}


//==============================================================================================================
//=                                        INITIALIZATION                                                      =
//==============================================================================================================
bool UDPServer::initialize() {
    // Connect to Wi-Fi
    int time_passed = 0;
    int time_to_wait = 10000; // 10 seconds
    WiFi.begin(WIFI_SSID, WIFI_PWD);
    while (time_passed <= time_to_wait) {
        time_passed += 500;
        delay(500);
    }

    // Report failure if Wi-Fi connection failed
    if(WiFi.status() != WL_CONNECTED) {
        return false;
    }
    // Otherwise try to start UDP communication
    else {
        int success = udp.begin(LOCAL_UDP_PORT);
        // Report failure if UDP communication could not be started
        if(success != 1) {
            return false;
        }
        // Otherwise report success and start waiting for requests
        else {
            state = WAITING_FOR_REQUEST;
            return true;
        }
    }
}


//==============================================================================================================
//=                                        LISTENING FOR REQUESTS                                              =
//==============================================================================================================
void UDPServer::listen(){
    int packet_size = udp.parsePacket();
    if(packet_size) {
        int len = udp.read(incoming_packet, 1);
        if(len > 0) {
            incoming_packet[len] = 0;
        }
        state = SENDING;
    }
}


//==============================================================================================================
//=                                        PACKET TRANSMISSION                                                 =
//==============================================================================================================
void UDPServer::sendPacket(char* packet, int packet_size) {
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write(packet, packet_size);
    udp.endPacket();
}

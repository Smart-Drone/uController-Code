#include "UDPServer.h"


//==============================================================================================================
//=                                        CONSTRUCTOR                                                         =
//==============================================================================================================
UDPServer::UDPServer(
    unsigned int local_udp_port
)
: LOCAL_UDP_PORT(local_udp_port)
, udp()
, state(INACTIVE) {

}


//==============================================================================================================
//=                                        ACCESSORS                                                           =
//==============================================================================================================
UDPServer::State UDPServer::getState() {
    return this->state;
}


void UDPServer::setState(UDPServer::State state) {
    this->state = state;
}

//==============================================================================================================
//=                                        INITIALIZATION                                                      =
//==============================================================================================================
bool UDPServer::initialize() {
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
        state = READY_TO_SEND;
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

/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#ifndef ETHERNETSERVER_H
#define ETHERNETSERVER_H

//#include "Server.h" Only creates class which includes print function from print class...?

// Class ethernetserver
typedef struct {
    uint16_t _port;
} EthernetServer;


// Implementation should be moved to .c file
void EthernetServer_new(EthernetServer *ethServ, uint16_t port) {
    ethServ->_port = port;
}



#endif


/*

#ifndef ethernetserver_h
#define ethernetserver_h
#include "Server.h"
class EthernetClient;
class EthernetServer :
public Server {
private:
uint16_t _port;
void accept();
public:
EthernetServer(uint16_t);
EthernetClient available();
virtual void begin();
virtual size_t write(uint8_t);
virtual size_t write(const uint8_t *buf, size_t size);
using Print::write;
};
#endif
*/

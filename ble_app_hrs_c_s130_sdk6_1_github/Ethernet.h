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

#ifndef ETHERNET_H
#define ETHERNET_H

#include "IPAddress.h"





#endif


/*
#ifndef ethernet_h
#define ethernet_h
#include <inttypes.h>
//#include "w5100.h"
#include "IPAddress.h"
#include "EthernetClient.h"
#include "EthernetServer.h"
#include "Dhcp.h"
#define MAX_SOCK_NUM 4
class EthernetClass {
private:
IPAddress _dnsServerAddress;
DhcpClass* _dhcp;
public:
static uint8_t _state[MAX_SOCK_NUM];
static uint16_t _server_port[MAX_SOCK_NUM];
// Initialise the Ethernet shield to use the provided MAC address and gain the rest of the
// configuration through DHCP.
// Returns 0 if the DHCP configuration failed, and 1 if it succeeded
int begin(uint8_t *mac_address);
void begin(uint8_t *mac_address, IPAddress local_ip);
void begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server);
void begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server, IPAddress gateway);
void begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet);
int maintain();
IPAddress localIP();
IPAddress subnetMask();
IPAddress gatewayIP();
IPAddress dnsServerIP();
friend class EthernetClient;
friend class EthernetServer;
};
extern EthernetClass Ethernet;
#endif
*/

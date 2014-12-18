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
#include "EthernetServer.h"
#include "Dhcp.h"
#include "w5100.h"

#define MAX_SOCK_NUM 4

typedef struct {
    IPAddress _dnsServerAddress;
    DhcpClass* _dhcp; // Not implemented yet
    
    uint8_t _state[MAX_SOCK_NUM];
    uint16_t _server_port[MAX_SOCK_NUM];
} EthernetClass;


void EthernetClass_begin_with_localIP_and_dnsServer_and_gateway_and_subnet(
            EthernetClass *ethernet, 
            uint8_t *mac_address, 
            IPAddress local_ip, 
            IPAddress dns_server, 
            IPAddress gateway, 
            IPAddress subnet
            ) {
    /*
                 W5100.init();
  SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
  W5100.setMACAddress(mac);
  W5100.setIPAddress(local_ip.raw_address());
  W5100.setGatewayIp(gateway.raw_address());
  W5100.setSubnetMask(subnet.raw_address());
  SPI.endTransaction();
  _dnsServerAddress = dns_server;
                */
}

void EthernetClass_begin_with_localIP_and_dnsServer_and_gateway(EthernetClass *ethernet, uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server, IPAddress gateway) {
    IPAddress subnet;
    IPAddress_new_with_octets(&subnet, 255, 255, 255, 0);
    EthernetClass_begin_with_localIP_and_dnsServer_and_gateway_and_subnet(ethernet, mac_address, local_ip, dns_server, gateway, subnet);
}

void EthernetClass_begin_with_localIP_and_dnsServer(EthernetClass *ethernet, uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server) {
    // Assumes the gateway will be the machine on the same network as the local IP but with last octet being '1'
    IPAddress gateway = local_ip;
    gateway._address[3] = 1;
    EthernetClass_begin_with_localIP_and_dnsServer_and_gateway(ethernet, mac_address, local_ip, dns_server, gateway);
}

void EthernetClass_begin_with_localIP(EthernetClass *ethernet, uint8_t *mac_address, IPAddress local_ip) {
    // Assumes the DNS server will be the machine on the same network as the local IP but with last octet being '1'
    IPAddress dns_server = local_ip;
    dns_server._address[3] = 1;
    EthernetClass_begin_with_localIP_and_dnsServer(ethernet, mac_address, local_ip, dns_server);
}


// Declare global ethernet class
EthernetClass Ethernet;

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

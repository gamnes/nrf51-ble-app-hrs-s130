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

#ifndef IPADDRESS_H
#define IPADDRESS_H

typedef struct {
    uint8_t _address[4];
} IPAddress;

// Implementation should be moved to .c file
void IPAddress_new(IPAddress *ipaddr) {
    memset(ipaddr->_address, 0, sizeof(ipaddr->_address)); // memset: copies the charachter 0 to the first sizeof chars of _address
}

void IPAddress_new_with_octets(IPAddress *ipaddr, uint8_t first_octet, uint8_t second_octet, uint8_t third_octet, uint8_t fourth_octet) {
    ipaddr->_address[0] = first_octet;
    ipaddr->_address[1] = second_octet;
    ipaddr->_address[2] = third_octet;
    ipaddr->_address[3] = fourth_octet;
}



#endif



/*
IPAddress.h - Base class that provides IPAddress
Copyright (c) 2011 Adrian McEwen. All right reserved.
This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.
This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/
/*
#ifndef IPAddress_h
#define IPAddress_h
#include <Printable.h>
// A class to make it easier to handle and pass around IP addresses
class IPAddress : public Printable {
private:
uint8_t _address[4]; // IPv4 address
// Access the raw byte array containing the address. Because this returns a pointer
// to the internal structure rather than a copy of the address this function should only
// be used when you know that the usage of the returned uint8_t* will be transient and not
// stored.
uint8_t* raw_address() { return _address; };
public:
// Constructors
IPAddress();
IPAddress(uint8_t first_octet, uint8_t second_octet, uint8_t third_octet, uint8_t fourth_octet);
IPAddress(uint32_t address);
IPAddress(const uint8_t *address);
// Overloaded cast operator to allow IPAddress objects to be used where a pointer
// to a four-byte uint8_t array is expected
operator uint32_t() { return *((uint32_t*)_address); };
bool operator==(const IPAddress& addr) { return (*((uint32_t*)_address)) == (*((uint32_t*)addr._address)); };
bool operator==(const uint8_t* addr);
// Overloaded index operator to allow getting and setting individual octets of the address
uint8_t operator[](int index) const { return _address[index]; };
uint8_t& operator[](int index) { return _address[index]; };
// Overloaded copy operators to allow initialisation of IPAddress objects from other types
IPAddress& operator=(const uint8_t *address);
IPAddress& operator=(uint32_t address);
virtual size_t printTo(Print& p) const;
friend class EthernetClass;
friend class UDP;
friend class Client;
friend class Server;
friend class DhcpClass;
friend class DNSClient;
};
const IPAddress INADDR_NONE(0,0,0,0);
#endif
*/

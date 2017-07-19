/*
 *  Udp.cpp: Library to send/receive UDP packets with the Arduino ethernet shield.
 *  This version only offers minimal wrapping of socket.c/socket.h
 *  Drop Udp.h/.cpp into the Ethernet library directory at hardware/libraries/Ethernet/
 *
 * NOTE: UDP is fast, but has some important limitations (thanks to Warren Gray for mentioning these)
 * 1) UDP does not guarantee the order in which assembled UDP packets are received. This
 * might not happen often in practice, but in larger network topologies, a UDP
 * packet can be received out of sequence.
 * 2) UDP does not guard against lost packets - so packets *can* disappear without the sender being
 * aware of it. Again, this may not be a concern in practice on small local networks.
 * For more information, see http://www.cafeaulait.org/course/week12/35.html
 *
 * MIT License:
 * Copyright (c) 2008 Bjoern Hartmann
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * bjoern@cs.stanford.edu 12/30/2008
 */

#ifndef ethernetudp_h
#define ethernetudp_h

//#include <Udp.h>

#define UDP_TX_PACKET_MAX_SIZE 24


  uint8_t _sock;  // socket ID for Wiz5100
  uint16_t _port; // local port to listen on
  uint8_t _remoteIP[4]; // remote IP address for the incoming packet whilst it's being processed
  uint16_t _remotePort; // remote port for the incoming packet whilst it's being processed
  uint16_t _offset; // offset into the packet being sent
  uint16_t _remaining; // remaining bytes of incoming packet yet to be processed


  uint8_t EthernetUDP_begin(SOCKET sock, uint16_t port);	// initialize, start listening on specified port. Returns 1 if successful, 0 if there are no sockets available to use
  void EthernetUDP_stop(SOCKET sock);  // Finish with the UDP socket

  // Sending UDP packets

  // Start building up a packet to send to the remote host specific in ip and port
  // Returns 1 if successful, 0 if there was a problem with the supplied IP address or port
  int EthernetUDP_beginPacket(SOCKET sock, uint8_t *ip, uint16_t port);
  // Start building up a packet to send to the remote host specific in host and port
  // Returns 1 if successful, 0 if there was a problem resolving the hostname or port
  //int EthernetUDP_beginPacket(const char *host, uint16_t port);
  // Finish off this packet and send it
  // Returns 1 if the packet was sent successfully, 0 if there was an error
  int EthernetUDP_endPacket(SOCKET sock);
  // Write a single byte into the packet
  //size_t write(uint8_t);
  // Write size bytes from buffer into the packet
  size_t EthernetUDP_write(SOCKET sock, const uint8_t *buffer, size_t size);

  // Start processing the next available incoming packet
  // Returns the size of the packet in bytes, or 0 if no packets are available
  int EthernetUDP_parsePacket(SOCKET sock);
  // Number of bytes remaining in the current packet
  int EthernetUDP_available();
  // Read a single byte from the current packet
  int EthernetUDP_read_byte(SOCKET sock);
  // Read up to len bytes from the current packet and place them into buffer
  // Returns the number of bytes read, or 0 if none are available
  int EthernetUDP_read(SOCKET sock, unsigned char* buffer, size_t len);
  // Read up to len characters from the current packet and place them into buffer
  // Returns the number of characters read, or 0 if none are available
  //int EthernetUDP_read(char* buffer, size_t len) { return read((unsigned char*)buffer, len); };
  // Return the next byte from the current packet without moving on to the next byte
  int EthernetUDP_peek(SOCKET sock);
  void EthernetUDP_flush(SOCKET sock);	// Finish reading the current packet

  // Return the IP address of the host who sent the current incoming packet
  void EthernetUDP_remoteIP(uint8_t *ip);
  // Return the port of the host who sent the current incoming packet
  uint16_t EthernetUDP_remotePort();


#endif

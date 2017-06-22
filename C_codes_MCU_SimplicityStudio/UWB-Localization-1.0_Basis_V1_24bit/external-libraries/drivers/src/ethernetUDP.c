/*
 *  Udp.cpp: Library to send/receive UDP packets with the Arduino ethernet shield.
 *  This version only offers minimal wrapping of socket.c/socket.h
 *  Drop Udp.h/.cpp into the Ethernet library directory at hardware/libraries/Ethernet/
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

#include "w5100.h"
#include "socket.h"
#include "ethernetUDP.h"
//#include "Ethernet.h"
//#include "Udp.h"
//#include "Dns.h"


/* Start EthernetUDP socket, listening at local port PORT */
uint8_t EthernetUDP_begin(SOCKET sock, uint16_t port) {

  _remaining = 0;
  socket(sock, SnMR_UDP, port, 0);

  return 1;
}

/* return number of bytes available in the current packet,
   will return zero if parsePacket hasn't been called yet */
int EthernetUDP_available() {
  return _remaining;
}

/* Release any resources being used by this EthernetUDP instance */
void EthernetUDP_stop(SOCKET sock)
{
   close(sock);
}

/*
int EthernetUDP_beginPacket(SOCKET sock, const char *host, uint16_t port)
{
  // Look up the host first
  int ret = 0;
  uint8_t remote_addr[4] = { 0 };

  dns.begin(Ethernet.dnsServerIP());
  ret = dns.getHostByName(host, remote_addr);
  if (ret == 1) {
    return beginPacket(sock, remote_addr, port);
  } else {
    return ret;
  }
}*/

int EthernetUDP_beginPacket(SOCKET sock, uint8_t *ip, uint16_t port)
{
  _offset = 0;
  return startUDP(sock, ip, port);
}

int EthernetUDP_endPacket(SOCKET sock)
{
  return sendUDP(sock);
}

size_t EthernetUDP_write(SOCKET sock, const uint8_t *buffer, size_t size)
{
  uint16_t bytes_written = bufferData(sock, _offset, buffer, size);
  _offset += bytes_written;
  return bytes_written;
}

int EthernetUDP_parsePacket(SOCKET sock)
{
  // discard any remaining bytes in the last packet
	EthernetUDP_flush(sock);

  if (recvAvailable(sock) > 0)
  {
    //HACK - hand-parse the UDP packet using TCP recv method
    uint8_t tmpBuf[8];
    int ret =0;
    //read 8 header bytes and get IP and port from it
    ret = recv(sock,tmpBuf,8);
    if (ret > 0)
    {
      _remoteIP[0] = tmpBuf[0];
      _remoteIP[1] = tmpBuf[1];
      _remoteIP[2] = tmpBuf[2];
      _remoteIP[3] = tmpBuf[3];
      _remotePort = tmpBuf[4];
      _remotePort = (_remotePort << 8) + tmpBuf[5];
      _remaining = tmpBuf[6];
      _remaining = (_remaining << 8) + tmpBuf[7];

      // When we get here, any remaining bytes are the data
      ret = _remaining;
    }
    return ret;
  }
  // There aren't any packets available
  return 0;
}

/*int EthernetUDP_read(SOCKET sock)
{
  uint8_t byte;

  if ((_remaining > 0) && (recv(sock, &byte, 1) > 0))
  {
    // We read things without any problems
    _remaining--;
    return byte;
  }

  // If we get here, there's no data available
  return -1;
}*/

int EthernetUDP_read(SOCKET sock, unsigned char* buffer, size_t len)
{

  if (_remaining > 0)
  {

    int got;

    if (_remaining <= len)
    {
      // data should fit in the buffer
      got = recv(sock, buffer, _remaining);
    }
    else
    {
      // too much data for the buffer,
      // grab as much as will fit
      got = recv(sock, buffer, len);
    }

    if (got > 0)
    {
      _remaining -= got;
      return got;
    }

  }

  // If we get here, there's no data available or recv failed
  return -1;

}

int EthernetUDP_read_byte(SOCKET sock)
{
  uint8_t byte;

  if ((_remaining > 0) && (recv(sock, &byte, 1) > 0))
  {
    // We read things without any problems
    _remaining--;
    return byte;
  }

  // If we get here, there's no data available
  return -1;
}

int EthernetUDP_peek(SOCKET sock)
{
  uint8_t b;
  // Unlike recv, peek doesn't check to see if there's any data available, so we must.
  // If the user hasn't called parsePacket yet then return nothing otherwise they
  // may get the UDP header
  if (!_remaining)
    return -1;
  peek(sock, &b);
  return b;
}

void EthernetUDP_flush(SOCKET sock)
{
  // could this fail (loop endlessly) if _remaining > 0 and recv in read fails?
  // should only occur if recv fails after telling us the data is there, lets
  // hope the w5100 always behaves :)

  while (_remaining)
  {
	  EthernetUDP_read_byte(sock);
  }
}

void EthernetUDP_remoteIP(uint8_t *ip) {
	ip = _remoteIP;
}

uint16_t EthernetUDP_remotePort() {
	return _remotePort;
}


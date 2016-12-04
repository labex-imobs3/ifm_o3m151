/* -*- mode: C++ -*-
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  O3M151 3D LIDAR data input classes
 *
 *    These classes provide raw O3M151 LIDAR input packets from
 *    either a live socket interface or a previously-saved PCAP dump
 *    file.
 *
 *  Classes:
 *
 *     o3m151::Input -- pure virtual base class to access the data
 *                      independently of its source
 *
 *     o3m151::InputSocket -- derived class reads live data from the
 *                      device via a UDP socket
 *
 *     o3m151::InputPCAP -- derived class provides a similar interface
 *                      from a PCAP dump file
 */

#ifndef __O3M151_INPUT_H
#define __O3M151_INPUT_H

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

namespace o3m151_driver
{
  class PacketHeader;
  static uint16_t UDP_PORT_NUMBER = 42000;

  /** @brief Pure virtual O3M151 input base class */
  class Input
  {
  public:
    Input();

    /** @brief Read one O3M151 packet.
     *
     * @param pkt points to O3M151Packet message
     *
     * @returns 0 if successful,
     *          -1 if end of file
     *          > 0 if incomplete packet (is this possible?)
     */
    virtual int getPacket(pcl::PointCloud<pcl::PointXYZI> &pc) = 0;

  protected:
    int processPacket( int8_t* currentPacketData,  // payload of the udp packet (without ethernet/IP/UDP header)
                        uint32_t currentPacketSize, // size of the udp packet payload
                        int8_t* channelBuffer,      // buffer for a complete channel
                        uint32_t channelBufferSize, // size of the buffer for the complete channel
                        uint32_t* pos);              // the current pos in the channel buffer

    double slope(const std::vector<double>& x, const std::vector<double>& y);
    void processChannel8(int8_t* buf, uint32_t size, pcl::PointCloud<pcl::PointXYZI> &pc);
    int process(int8_t *udpPacketBuf, const ssize_t rc, pcl::PointCloud<pcl::PointXYZI> & pc);

    // the size of the channel may change so the size will be taken from the packet
    uint32_t channel_buf_size_;
    int8_t* channelBuf;

    // As there is no offset in the packet header we have to remember where the next part should go
    uint32_t pos_in_channel_;

    // remember the counter of the previous packet so we know when we loose packets
    uint32_t previous_packet_counter_;
    bool previous_packet_counter_valid_;
    // the receiption of the data may start at any time. So we wait til we find the beginning of our channel
    bool startOfChannelFound_;
  };

  /** @brief Live O3M151 input from socket. */
  class InputSocket: public Input
  {
  public:
    InputSocket(ros::NodeHandle private_nh,
                uint16_t udp_port = UDP_PORT_NUMBER);
    ~InputSocket();
    virtual int getPacket(pcl::PointCloud<pcl::PointXYZI> &pc);

  private:

    int sockfd_;
  };


  /** @brief O3M151 input from PCAP dump file.
   *
   * Dump files can be grabbed by libpcap,
   * ethereal, wireshark, tcpdump, or the \ref vdump_command.
   */
  class InputPCAP: public Input
  {
  public:
    InputPCAP(ros::NodeHandle private_nh,
              double packet_rate,
              std::string filename="",
              bool read_once=false,
              bool read_fast=false,
              double repeat_delay=0.0);
    ~InputPCAP();

    virtual int getPacket(pcl::PointCloud<pcl::PointXYZI> &pc);

  private:

    std::string filename_;
    FILE *fp_;
    pcap_t *pcap_;
    char errbuf_[PCAP_ERRBUF_SIZE];
    bool empty_;
    bool read_once_;
    bool read_fast_;
    double repeat_delay_;
    ros::Rate packet_rate_;
  };

} // o3m151_driver namespace

#endif // __O3M151_INPUT_H

/*
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Input classes for the O3M151 HDL-64E 3D LIDAR:
 *
 *     Input -- virtual base class than can be used to access the data
 *              independently of its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */

#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <o3m151_driver/input.h>

#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>

#define RESULT_OK (0)
#define RESULT_ERROR (-1)

namespace o3m151_driver
{
  static const size_t packet_size = 1024; //sizeof(o3m151_msgs::O3M151Packet().data);
  // The data is in the channel 8
  static const uint32_t customerDataChannel = 8;

  typedef struct PacketHeader
  {
    uint16_t Version;
    uint16_t Device;
    uint32_t PacketCounter;
    uint32_t CycleCounter;
    uint16_t NumberOfPacketsInCycle;
    uint16_t IndexOfPacketInCycle;
    uint16_t NumberOfPacketsInChannel;
    uint16_t IndexOfPacketInChannel;
    uint32_t ChannelID;
    uint32_t TotalLengthOfChannel;
    uint32_t LengthPayload;
  } PacketHeader;


  typedef struct ChannelHeader
  {
    uint32_t StartDelimiter;
    uint8_t reserved[24];
  } ChannelHeader;

  typedef struct ChannelEnd
  {
    uint32_t EndDelimiter;
  } ChannelEnd;

  Input::Input() :
      channel_buf_size_(0),
      previous_packet_counter_(0),
      previous_packet_counter_valid_(false),
      startOfChannelFound_(false),
      pos_in_channel_(0)
  {
      channelBuf = NULL;
  }

  // Extracts the data in the payload of the udp packet und puts it into the channel buffer
  int Input::processPacket( int8_t* currentPacketData,  // payload of the udp packet (without ethernet/IP/UDP header)
                      uint32_t currentPacketSize, // size of the udp packet payload
                      int8_t* channelBuffer,      // buffer for a complete channel
                      uint32_t channelBufferSize, // size of the buffer for the complete channel
                      uint32_t* pos)              // the current pos in the channel buffer
  {

    // There is always a PacketHeader structure at the beginning
    PacketHeader* ph = (PacketHeader*)currentPacketData;
    int Start = sizeof(PacketHeader);
    int Length = currentPacketSize - sizeof(PacketHeader);

    // Only the first packet of a channel contains a ChannelHeader
    if (ph->IndexOfPacketInChannel == 0)
    {
      Start += sizeof(ChannelHeader);
    }

      // Only the last packet of a channel contains an EndDelimiter (at the end, after the data)
    if (ph->IndexOfPacketInChannel == ph->NumberOfPacketsInChannel - 1)
    {
      Length -= sizeof(ChannelEnd);
    }

    // Is the buffer big enough?
    if ((*pos) + Length > channelBufferSize)
    {
        // Too small means either an error in the program logic or a corrupt packet
        ROS_DEBUG("Channel buffer is too small.\n");
        return RESULT_ERROR;
    }
    else
    {
      memcpy(channelBuffer+(*pos), currentPacketData+Start, Length);
    }

    (*pos) += Length;

    return RESULT_OK;
  }


  double Input::slope(const std::vector<double>& x, const std::vector<double>& y) {
      const double n    = x.size();
      const double s_x  = std::accumulate(x.begin(), x.end(), 0.0);
      const double s_y  = std::accumulate(y.begin(), y.end(), 0.0);
      const double s_xx = std::inner_product(x.begin(), x.end(), x.begin(), 0.0);
      const double s_xy = std::inner_product(x.begin(), x.end(), y.begin(), 0.0);
      const double a    = (n * s_xy - s_x * s_y) / (n * s_xx - s_x * s_x);
      return a;
  }

  // Gets the data from the channel and prints them
  void Input::processChannel8(int8_t* buf, uint32_t size, pcl::PointCloud<pcl::PointXYZI> & pc)
  {
    // you may find the offset of the data in the documentation
    const uint32_t distanceX_offset = 2052;
    const uint32_t distanceY_offset = 6148;
    const uint32_t distanceZ_offset = 10244;
    const uint32_t confidence_offset = 14340;
    const uint32_t amplitude_offset = 16388;
    const uint32_t amplitude_normalization_offset = 18436;
    const uint32_t camera_calibration_offset = 18464;

    // As we are working on raw buffers we have to check if the buffer is as big as we think
    if (size < 18488)
    {
      ROS_DEBUG("processChannel8: buf too small\n");
      return;
    }

    // These are arrays with 16*64=1024 elements
    float* distanceX = (float*)(&buf[distanceX_offset]);
    float* distanceY = (float*)(&buf[distanceY_offset]);
    float* distanceZ = (float*)(&buf[distanceZ_offset]);
    uint16_t* confidence = (uint16_t*)(&buf[confidence_offset]);
    uint16_t* amplitude = (uint16_t*)(&buf[amplitude_offset]);
    float* amplitude_normalization = (float*)(&buf[amplitude_normalization_offset]);
    float* camera_calibration = (float*)(&buf[camera_calibration_offset]);

    // line 8 column 32 is in the middle of the sensor
    //uint32_t i = 64*7 + 32;

    std::vector<double> vx, vz;
    for( uint32_t m =0; m<16; ++m)
    {
      for( uint32_t j =0; j<64; ++j)
      {
        uint32_t i = 64*m +j;
        // The lowest bit of the confidence contains the information if the pixel is valid
        uint32_t pixelValid = confidence[i] & 1;

        // 0=valid, 1=invalid
        if (distanceX[i] > 0.2)// && j > 20 && j < 50 && m >0 && m <12)
        {
            // \r is carriage return without newline. This way the output is always written in the same line
  //        ROS_DEBUG("X: %5.2f Y: %5.2f Z:%5.2f Amplitude:%5d                                               \r",
  //            distanceX[i],
  //            distanceY[i],
  //            distanceZ[i],
  //            amplitude[i]);
          pcl::PointXYZI point;
          point.x = distanceX[i] ;//- camera_calibration[0];
          point.y = distanceY[i] ;//- camera_calibration[1];
          point.z = distanceZ[i] - 1;// camera_calibration[2];
//          bool exposure = (confidence[i] >> 1) & 1;
//          bool gain = (confidence[i] >> 2) & 1;
//          int norm_index;
//          if(gain == false)
//            if(exposure == false)
//              norm_index = 0;
//            else
//              norm_index = 1;
//          else
//            if(exposure == false)
//              norm_index = 2;
//            else
//              norm_index = 3;
          float normalization_factor = 1;
//          float normalization_factor = amplitude_normalization[norm_index];
//          ROS_INFO_STREAM("gain "<<gain<<" exposure "<<exposure<<
//                          " amplitude[i] "<<amplitude[i]<<" normalization_factor "<<normalization_factor);
          point.intensity = amplitude[i]*normalization_factor;
          pc.points.push_back(point);
//          if(j == 40)
//          {
//            vx.push_back(point.x);
//            vz.push_back(point.z);
//          }
        }
      }
    }
    for(int n = 0; n <4; n++)
      ROS_DEBUG_STREAM("amplitude_normalization "<<n<<" "<<amplitude_normalization[n]);
//    static int k = 0;
//    static int l = 0;
//    l++;
//    static double s=0, s_mean=0;
//    s += slope(vz, vx);
//    if(l%10 == 0)
//    {
//      k++;
//      if(k > 63)
//        k =0;

//      s_mean = s/10;
//      s=0;
//    }
//    ROS_INFO_STREAM("vx "<<vx.size()<<" vz "<<vz.size());
//    ROS_INFO_STREAM("s "<<s_mean);
  }

  int Input::process(int8_t* udpPacketBuf, const ssize_t rc, pcl::PointCloud<pcl::PointXYZI> & pc)
  {
    // As the alignment was forced to 1 we can work with the struct on the buffer.
    // This assumes the byte order is little endian which it is on a PC.
    PacketHeader* ph = (PacketHeader*)udpPacketBuf;

    // Check the packet counter for missing packets
    if (previous_packet_counter_valid_)
    {
      // if the type of the variables is ui32, it will also work when the wrap around happens.
      if ((ph->PacketCounter - previous_packet_counter_) != 1)
      {
        ROS_ERROR("Packet Counter jumped from %ul to %ul", previous_packet_counter_, ph->PacketCounter);

        // With this it will ignore the already received parts and resynchronize at
        // the beginning of the next cycle.
        startOfChannelFound_ = false;
      }
    }

    previous_packet_counter_ = ph->PacketCounter;
    previous_packet_counter_valid_ = true;

    // is this the channel with our data?
    if (ph->ChannelID == customerDataChannel)
    {
      // are we at the beginning of the channel?
      if (ph->IndexOfPacketInChannel == 0)
      {
        startOfChannelFound_ = true;

        // If we haven't allocated memory for channel do it now.
        if (channel_buf_size_ == 0)
        {
            channel_buf_size_ = ph->TotalLengthOfChannel;
            channelBuf = new int8_t[channel_buf_size_];
        }

        // as we reuse the buffer we clear it at the beginning of a transmission
        memset(channelBuf, 0, channel_buf_size_);
        pos_in_channel_ = 0;
      }

      // if we have found the start of the channel at least once, we are ready to process the packet
      if (startOfChannelFound_)
      {
        processPacket(udpPacketBuf, rc, channelBuf, channel_buf_size_, &pos_in_channel_);

        // Have we found the last packet in this channel? Then we are able to process it
        // The index is zero based so a channel with n parts will have indices from 0 to n-1
        if (ph->IndexOfPacketInChannel == ph->NumberOfPacketsInChannel -1)
        {
            processChannel8(channelBuf, pos_in_channel_, pc);
            return RESULT_OK;
        }
      }
    }
  }
  ////////////////////////////////////////////////////////////////////////
  // InputSocket class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh private node handle for driver
   *  @param udp_port UDP port number to connect
   */
  InputSocket::InputSocket(ros::NodeHandle private_nh, uint16_t udp_port):
    Input()
  {
    sockfd_ = -1;

    // connect to O3M151 UDP port
    ROS_INFO_STREAM("Opening UDP socket: port " << udp_port);
    sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
    if (sockfd_ == -1)
      {
        perror("socket");               // TODO: ROS_ERROR errno
        return;
      }
  
    sockaddr_in my_addr;                     // my address information
    memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
    my_addr.sin_family = AF_INET;            // host byte order
    my_addr.sin_port = htons(udp_port);      // short, in network byte order
    my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP
  
    if (bind(sockfd_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
      {
        perror("bind");                 // TODO: ROS_ERROR errno
        return;
      }
  
    if (fcntl(sockfd_,F_SETFL, O_NONBLOCK|FASYNC) < 0)
      {
        perror("non-block");
        return;
      }

    ROS_DEBUG("O3M151 socket fd is %d\n", sockfd_);
  }

  /** @brief destructor */
  InputSocket::~InputSocket(void)
  {
    (void) close(sockfd_);
  }

  int InputSocket::getPacket(pcl::PointCloud<pcl::PointXYZI> &pc)
  {

    // buffer for a single UDP packet
    const uint32_t udpPacketBufLen = 2000;
    int8_t udpPacketBuf[udpPacketBufLen];

    struct pollfd fds[1];
    fds[0].fd = sockfd_;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 1000; // one second (in msec)

    // run for all eternity as long as no error occurs
    while (true)
    {
      // Unfortunately, the Linux kernel recvfrom() implementation
      // uses a non-interruptible sleep() when waiting for data,
      // which would cause this method to hang if the device is not
      // providing data.  We poll() the device first to make sure
      // the recvfrom() will not block.
      //
      // Note, however, that there is a known Linux kernel bug:
      //
      //   Under Linux, select() may report a socket file descriptor
      //   as "ready for reading", while nevertheless a subsequent
      //   read blocks.  This could for example happen when data has
      //   arrived but upon examination has wrong checksum and is
      //   discarded.  There may be other circumstances in which a
      //   file descriptor is spuriously reported as ready.  Thus it
      //   may be safer to use O_NONBLOCK on sockets that should not
      //   block.

      // poll() until input available
      do
      {
        int retval = poll(fds, 1, POLL_TIMEOUT);
        if (retval < 0)             // poll() error?
        {
          if (errno != EINTR)
            ROS_ERROR("poll() error: %s", strerror(errno));
          return 1;
        }
        if (retval == 0)            // poll() timeout?
        {
          ROS_WARN("O3M151 poll() timeout");
          return 1;
        }
        if ((fds[0].revents & POLLERR)
            || (fds[0].revents & POLLHUP)
            || (fds[0].revents & POLLNVAL)) // device error?
        {
          ROS_ERROR("poll() reports O3M151 error");
          return 1;
        }
      } while ((fds[0].revents & POLLIN) == 0);
      // receive the data. rc contains the number of received bytes and also the error code
      // IMPORTANT: This is a blocking call. If it doesn't receive anything it will wait forever
      ssize_t rc=recvfrom(sockfd_,(char*)udpPacketBuf,udpPacketBufLen,0,NULL,NULL);
      if(rc < 0)
        return RESULT_ERROR;
      else
      {
        int result = process(udpPacketBuf, rc, pc);
        ROS_DEBUG("result process %d", result);
        if(result == RESULT_OK)
          return result;
      }
    }
  }

  ////////////////////////////////////////////////////////////////////////
  // InputPCAP class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh private node handle for driver
   *  @param packet_rate expected device packet frequency (Hz)
   *  @param filename PCAP dump file name
   *  @param read_once read PCAP in a loop, unless false
   *  @param read_fast read PCAP at device rate, unless false
   *  @param repeat_delay time to wait before repeating PCAP data
   */
  InputPCAP::InputPCAP(ros::NodeHandle private_nh,
                       double packet_rate,
                       std::string filename,
                       bool read_once,
                       bool read_fast,
                       double repeat_delay):
    Input(),
    packet_rate_(packet_rate)
  {
    filename_ = filename;
    fp_ = NULL;  
    pcap_ = NULL;  
    empty_ = true;

    // get parameters using private node handle
    private_nh.param("read_once", read_once_, read_once);
    private_nh.param("read_fast", read_fast_, read_fast);
    private_nh.param("repeat_delay", repeat_delay_, repeat_delay);

    if (read_once_)
      ROS_INFO("Read input file only once.");
    if (read_fast_)
      ROS_INFO("Read input file as quickly as possible.");
    if (repeat_delay_ > 0.0)
      ROS_INFO("Delay %.3f seconds before repeating input file.",
               repeat_delay_);

    // Open the PCAP dump file
    ROS_INFO("Opening PCAP file \"%s\"", filename_.c_str());
    if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_) ) == NULL)
    {
      ROS_FATAL("Error opening O3M151 socket dump file.");
      return;
    }
  }


  /** destructor */
  InputPCAP::~InputPCAP(void)
  {
    pcap_close(pcap_);
  }


  /** @brief Get one o3m151 packet. */
  int InputPCAP::getPacket(pcl::PointCloud<pcl::PointXYZI> &pc)
  {
    struct pcap_pkthdr *header;
    const u_char *pkt_data;
    // buffer for a single UDP packet
    const uint32_t udpPacketBufLen = 2000;
    int8_t udpPacketBuf[udpPacketBufLen];

    while (true)
    {
      bool keep_reading = false;
      int res;
      if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
      {
        // Keep the reader from blowing through the file.
        if (read_fast_ == false)
          packet_rate_.sleep();

        memcpy(udpPacketBuf, pkt_data+42, header->len);
        int result = RESULT_ERROR;
        result = process(udpPacketBuf, header->len-42, pc);
        ROS_DEBUG("result process %d", header->len);
        empty_ = false;
        if(result == RESULT_OK)
          return RESULT_OK;                   // success
        else
          keep_reading = true;
      }

      if (empty_)                 // no data in file?
      {
        ROS_WARN("Error %d reading O3M151 packet: %s",
                 res, pcap_geterr(pcap_));
        return -1;
      }

      if (read_once_)
      {
        ROS_INFO("end of file reached -- done reading.");
        return -1;
      }

      if (repeat_delay_ > 0.0)
      {
        ROS_INFO("end of file reached -- delaying %.3f seconds.",
                 repeat_delay_);
        usleep(rint(repeat_delay_ * 1000000.0));
      }

      if(keep_reading == false)
      {
        ROS_DEBUG("replaying O3M151 dump file");
        // I can't figure out how to rewind the file, because it
        // starts with some kind of header.  So, close the file
        // and reopen it with pcap.
        pcap_close(pcap_);
        pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
        empty_ = true;              // maybe the file disappeared?
      }
    } // loop back and try again
  }

} // o3m151 namespace

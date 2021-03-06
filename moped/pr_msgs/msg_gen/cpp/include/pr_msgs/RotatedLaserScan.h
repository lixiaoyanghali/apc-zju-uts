/* Auto-generated by genmsg_cpp for file /home/wu/ros_f_ws/sandbox/pr_msgs/msg/RotatedLaserScan.msg */
#ifndef PR_MSGS_MESSAGE_ROTATEDLASERSCAN_H
#define PR_MSGS_MESSAGE_ROTATEDLASERSCAN_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "sensor_msgs/LaserScan.h"

namespace pr_msgs
{
template <class ContainerAllocator>
struct RotatedLaserScan_ {
  typedef RotatedLaserScan_<ContainerAllocator> Type;

  RotatedLaserScan_()
  : angle(0.0)
  , velocity(0.0)
  , laser_scan()
  {
  }

  RotatedLaserScan_(const ContainerAllocator& _alloc)
  : angle(0.0)
  , velocity(0.0)
  , laser_scan(_alloc)
  {
  }

  typedef double _angle_type;
  double angle;

  typedef double _velocity_type;
  double velocity;

  typedef  ::sensor_msgs::LaserScan_<ContainerAllocator>  _laser_scan_type;
   ::sensor_msgs::LaserScan_<ContainerAllocator>  laser_scan;


  typedef boost::shared_ptr< ::pr_msgs::RotatedLaserScan_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr_msgs::RotatedLaserScan_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct RotatedLaserScan
typedef  ::pr_msgs::RotatedLaserScan_<std::allocator<void> > RotatedLaserScan;

typedef boost::shared_ptr< ::pr_msgs::RotatedLaserScan> RotatedLaserScanPtr;
typedef boost::shared_ptr< ::pr_msgs::RotatedLaserScan const> RotatedLaserScanConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::pr_msgs::RotatedLaserScan_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::pr_msgs::RotatedLaserScan_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace pr_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::pr_msgs::RotatedLaserScan_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::pr_msgs::RotatedLaserScan_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::pr_msgs::RotatedLaserScan_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3c54b452087e2e0b6fb2114d826dfdb4";
  }

  static const char* value(const  ::pr_msgs::RotatedLaserScan_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x3c54b452087e2e0bULL;
  static const uint64_t static_value2 = 0x6fb2114d826dfdb4ULL;
};

template<class ContainerAllocator>
struct DataType< ::pr_msgs::RotatedLaserScan_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pr_msgs/RotatedLaserScan";
  }

  static const char* value(const  ::pr_msgs::RotatedLaserScan_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::pr_msgs::RotatedLaserScan_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 angle\n\
float64 velocity\n\
sensor_msgs/LaserScan laser_scan\n\
\n\
================================================================================\n\
MSG: sensor_msgs/LaserScan\n\
# Single scan from a planar laser range-finder\n\
#\n\
# If you have another ranging device with different behavior (e.g. a sonar\n\
# array), please find or create a different message, since applications\n\
# will make fairly laser-specific assumptions about this data\n\
\n\
Header header            # timestamp in the header is the acquisition time of \n\
                         # the first ray in the scan.\n\
                         #\n\
                         # in frame frame_id, angles are measured around \n\
                         # the positive Z axis (counterclockwise, if Z is up)\n\
                         # with zero angle being forward along the x axis\n\
                         \n\
float32 angle_min        # start angle of the scan [rad]\n\
float32 angle_max        # end angle of the scan [rad]\n\
float32 angle_increment  # angular distance between measurements [rad]\n\
\n\
float32 time_increment   # time between measurements [seconds] - if your scanner\n\
                         # is moving, this will be used in interpolating position\n\
                         # of 3d points\n\
float32 scan_time        # time between scans [seconds]\n\
\n\
float32 range_min        # minimum range value [m]\n\
float32 range_max        # maximum range value [m]\n\
\n\
float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)\n\
float32[] intensities    # intensity data [device-specific units].  If your\n\
                         # device does not provide intensities, please leave\n\
                         # the array empty.\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::pr_msgs::RotatedLaserScan_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::pr_msgs::RotatedLaserScan_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.angle);
    stream.next(m.velocity);
    stream.next(m.laser_scan);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct RotatedLaserScan_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr_msgs::RotatedLaserScan_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::pr_msgs::RotatedLaserScan_<ContainerAllocator> & v) 
  {
    s << indent << "angle: ";
    Printer<double>::stream(s, indent + "  ", v.angle);
    s << indent << "velocity: ";
    Printer<double>::stream(s, indent + "  ", v.velocity);
    s << indent << "laser_scan: ";
s << std::endl;
    Printer< ::sensor_msgs::LaserScan_<ContainerAllocator> >::stream(s, indent + "  ", v.laser_scan);
  }
};


} // namespace message_operations
} // namespace ros

#endif // PR_MSGS_MESSAGE_ROTATEDLASERSCAN_H


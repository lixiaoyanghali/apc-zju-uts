/* Auto-generated by genmsg_cpp for file /home/wu/ros_f_ws/sandbox/pr_msgs/msg/ObjectPoseList.msg */
#ifndef PR_MSGS_MESSAGE_OBJECTPOSELIST_H
#define PR_MSGS_MESSAGE_OBJECTPOSELIST_H
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

#include "std_msgs/Header.h"
#include "pr_msgs/ObjectPose.h"

namespace pr_msgs
{
template <class ContainerAllocator>
struct ObjectPoseList_ {
  typedef ObjectPoseList_<ContainerAllocator> Type;

  ObjectPoseList_()
  : header()
  , object_list()
  , originalTimeStamp()
  , requestTimeStamp()
  {
  }

  ObjectPoseList_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , object_list(_alloc)
  , originalTimeStamp()
  , requestTimeStamp()
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::vector< ::pr_msgs::ObjectPose_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::pr_msgs::ObjectPose_<ContainerAllocator> >::other >  _object_list_type;
  std::vector< ::pr_msgs::ObjectPose_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::pr_msgs::ObjectPose_<ContainerAllocator> >::other >  object_list;

  typedef ros::Time _originalTimeStamp_type;
  ros::Time originalTimeStamp;

  typedef ros::Time _requestTimeStamp_type;
  ros::Time requestTimeStamp;


  typedef boost::shared_ptr< ::pr_msgs::ObjectPoseList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr_msgs::ObjectPoseList_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ObjectPoseList
typedef  ::pr_msgs::ObjectPoseList_<std::allocator<void> > ObjectPoseList;

typedef boost::shared_ptr< ::pr_msgs::ObjectPoseList> ObjectPoseListPtr;
typedef boost::shared_ptr< ::pr_msgs::ObjectPoseList const> ObjectPoseListConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::pr_msgs::ObjectPoseList_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::pr_msgs::ObjectPoseList_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace pr_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::pr_msgs::ObjectPoseList_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::pr_msgs::ObjectPoseList_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::pr_msgs::ObjectPoseList_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8374d419f4435fc3eaa2652d85ff89d7";
  }

  static const char* value(const  ::pr_msgs::ObjectPoseList_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8374d419f4435fc3ULL;
  static const uint64_t static_value2 = 0xeaa2652d85ff89d7ULL;
};

template<class ContainerAllocator>
struct DataType< ::pr_msgs::ObjectPoseList_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pr_msgs/ObjectPoseList";
  }

  static const char* value(const  ::pr_msgs::ObjectPoseList_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::pr_msgs::ObjectPoseList_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
\n\
ObjectPose[] object_list\n\
\n\
time originalTimeStamp\n\
\n\
time requestTimeStamp\n\
\n\
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
================================================================================\n\
MSG: pr_msgs/ObjectPose\n\
string name\n\
\n\
geometry_msgs/Pose pose\n\
\n\
int16[] convex_hull_x\n\
int16[] convex_hull_y\n\
\n\
float32 mean_quality\n\
int16 used_points\n\
\n\
NameTypeValue[] properties\n\
\n\
geometry_msgs/Pose[] pose_uncertainty_list\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: pr_msgs/NameTypeValue\n\
string name\n\
string type\n\
string value\n\
\n\
";
  }

  static const char* value(const  ::pr_msgs::ObjectPoseList_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::pr_msgs::ObjectPoseList_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::pr_msgs::ObjectPoseList_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::pr_msgs::ObjectPoseList_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.object_list);
    stream.next(m.originalTimeStamp);
    stream.next(m.requestTimeStamp);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ObjectPoseList_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr_msgs::ObjectPoseList_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::pr_msgs::ObjectPoseList_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "object_list[]" << std::endl;
    for (size_t i = 0; i < v.object_list.size(); ++i)
    {
      s << indent << "  object_list[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::pr_msgs::ObjectPose_<ContainerAllocator> >::stream(s, indent + "    ", v.object_list[i]);
    }
    s << indent << "originalTimeStamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.originalTimeStamp);
    s << indent << "requestTimeStamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.requestTimeStamp);
  }
};


} // namespace message_operations
} // namespace ros

#endif // PR_MSGS_MESSAGE_OBJECTPOSELIST_H

/* Auto-generated by genmsg_cpp for file /home/wu/ros_f_ws/sandbox/pr_msgs/msg/SignalFeedback.msg */
#ifndef PR_MSGS_MESSAGE_SIGNALFEEDBACK_H
#define PR_MSGS_MESSAGE_SIGNALFEEDBACK_H
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


namespace pr_msgs
{
template <class ContainerAllocator>
struct SignalFeedback_ {
  typedef SignalFeedback_<ContainerAllocator> Type;

  SignalFeedback_()
  {
  }

  SignalFeedback_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::pr_msgs::SignalFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr_msgs::SignalFeedback_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SignalFeedback
typedef  ::pr_msgs::SignalFeedback_<std::allocator<void> > SignalFeedback;

typedef boost::shared_ptr< ::pr_msgs::SignalFeedback> SignalFeedbackPtr;
typedef boost::shared_ptr< ::pr_msgs::SignalFeedback const> SignalFeedbackConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::pr_msgs::SignalFeedback_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::pr_msgs::SignalFeedback_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace pr_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::pr_msgs::SignalFeedback_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::pr_msgs::SignalFeedback_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::pr_msgs::SignalFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::pr_msgs::SignalFeedback_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::pr_msgs::SignalFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pr_msgs/SignalFeedback";
  }

  static const char* value(const  ::pr_msgs::SignalFeedback_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::pr_msgs::SignalFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
\n\
";
  }

  static const char* value(const  ::pr_msgs::SignalFeedback_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::pr_msgs::SignalFeedback_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::pr_msgs::SignalFeedback_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SignalFeedback_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr_msgs::SignalFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::pr_msgs::SignalFeedback_<ContainerAllocator> & v) 
  {
  }
};


} // namespace message_operations
} // namespace ros

#endif // PR_MSGS_MESSAGE_SIGNALFEEDBACK_H


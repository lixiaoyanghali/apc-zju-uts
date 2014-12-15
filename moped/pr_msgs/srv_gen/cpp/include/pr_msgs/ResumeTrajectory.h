/* Auto-generated by genmsg_cpp for file /home/wu/ros_f_ws/sandbox/pr_msgs/srv/ResumeTrajectory.srv */
#ifndef PR_MSGS_SERVICE_RESUMETRAJECTORY_H
#define PR_MSGS_SERVICE_RESUMETRAJECTORY_H
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

#include "ros/service_traits.h"




namespace pr_msgs
{
template <class ContainerAllocator>
struct ResumeTrajectoryRequest_ {
  typedef ResumeTrajectoryRequest_<ContainerAllocator> Type;

  ResumeTrajectoryRequest_()
  {
  }

  ResumeTrajectoryRequest_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::pr_msgs::ResumeTrajectoryRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr_msgs::ResumeTrajectoryRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ResumeTrajectoryRequest
typedef  ::pr_msgs::ResumeTrajectoryRequest_<std::allocator<void> > ResumeTrajectoryRequest;

typedef boost::shared_ptr< ::pr_msgs::ResumeTrajectoryRequest> ResumeTrajectoryRequestPtr;
typedef boost::shared_ptr< ::pr_msgs::ResumeTrajectoryRequest const> ResumeTrajectoryRequestConstPtr;


template <class ContainerAllocator>
struct ResumeTrajectoryResponse_ {
  typedef ResumeTrajectoryResponse_<ContainerAllocator> Type;

  ResumeTrajectoryResponse_()
  : ok(false)
  , reason()
  {
  }

  ResumeTrajectoryResponse_(const ContainerAllocator& _alloc)
  : ok(false)
  , reason(_alloc)
  {
  }

  typedef uint8_t _ok_type;
  uint8_t ok;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _reason_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  reason;


  typedef boost::shared_ptr< ::pr_msgs::ResumeTrajectoryResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr_msgs::ResumeTrajectoryResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ResumeTrajectoryResponse
typedef  ::pr_msgs::ResumeTrajectoryResponse_<std::allocator<void> > ResumeTrajectoryResponse;

typedef boost::shared_ptr< ::pr_msgs::ResumeTrajectoryResponse> ResumeTrajectoryResponsePtr;
typedef boost::shared_ptr< ::pr_msgs::ResumeTrajectoryResponse const> ResumeTrajectoryResponseConstPtr;

struct ResumeTrajectory
{

typedef ResumeTrajectoryRequest Request;
typedef ResumeTrajectoryResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct ResumeTrajectory
} // namespace pr_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::pr_msgs::ResumeTrajectoryRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::pr_msgs::ResumeTrajectoryRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::pr_msgs::ResumeTrajectoryRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::pr_msgs::ResumeTrajectoryRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::pr_msgs::ResumeTrajectoryRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pr_msgs/ResumeTrajectoryRequest";
  }

  static const char* value(const  ::pr_msgs::ResumeTrajectoryRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::pr_msgs::ResumeTrajectoryRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
";
  }

  static const char* value(const  ::pr_msgs::ResumeTrajectoryRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::pr_msgs::ResumeTrajectoryRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::pr_msgs::ResumeTrajectoryResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::pr_msgs::ResumeTrajectoryResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::pr_msgs::ResumeTrajectoryResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4679398f882e7cbdea165980d3ec2888";
  }

  static const char* value(const  ::pr_msgs::ResumeTrajectoryResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x4679398f882e7cbdULL;
  static const uint64_t static_value2 = 0xea165980d3ec2888ULL;
};

template<class ContainerAllocator>
struct DataType< ::pr_msgs::ResumeTrajectoryResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pr_msgs/ResumeTrajectoryResponse";
  }

  static const char* value(const  ::pr_msgs::ResumeTrajectoryResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::pr_msgs::ResumeTrajectoryResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool ok\n\
string reason\n\
\n\
\n\
";
  }

  static const char* value(const  ::pr_msgs::ResumeTrajectoryResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::pr_msgs::ResumeTrajectoryRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ResumeTrajectoryRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::pr_msgs::ResumeTrajectoryResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ok);
    stream.next(m.reason);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ResumeTrajectoryResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<pr_msgs::ResumeTrajectory> {
  static const char* value() 
  {
    return "4679398f882e7cbdea165980d3ec2888";
  }

  static const char* value(const pr_msgs::ResumeTrajectory&) { return value(); } 
};

template<>
struct DataType<pr_msgs::ResumeTrajectory> {
  static const char* value() 
  {
    return "pr_msgs/ResumeTrajectory";
  }

  static const char* value(const pr_msgs::ResumeTrajectory&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<pr_msgs::ResumeTrajectoryRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4679398f882e7cbdea165980d3ec2888";
  }

  static const char* value(const pr_msgs::ResumeTrajectoryRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<pr_msgs::ResumeTrajectoryRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pr_msgs/ResumeTrajectory";
  }

  static const char* value(const pr_msgs::ResumeTrajectoryRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<pr_msgs::ResumeTrajectoryResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4679398f882e7cbdea165980d3ec2888";
  }

  static const char* value(const pr_msgs::ResumeTrajectoryResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<pr_msgs::ResumeTrajectoryResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pr_msgs/ResumeTrajectory";
  }

  static const char* value(const pr_msgs::ResumeTrajectoryResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // PR_MSGS_SERVICE_RESUMETRAJECTORY_H

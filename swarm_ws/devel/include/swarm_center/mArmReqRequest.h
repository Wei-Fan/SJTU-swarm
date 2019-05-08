// Generated by gencpp from file swarm_center/mArmReqRequest.msg
// DO NOT EDIT!


#ifndef SWARM_CENTER_MESSAGE_MARMREQREQUEST_H
#define SWARM_CENTER_MESSAGE_MARMREQREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace swarm_center
{
template <class ContainerAllocator>
struct mArmReqRequest_
{
  typedef mArmReqRequest_<ContainerAllocator> Type;

  mArmReqRequest_()
    : a(0)  {
    }
  mArmReqRequest_(const ContainerAllocator& _alloc)
    : a(0)  {
  (void)_alloc;
    }



   typedef int32_t _a_type;
  _a_type a;





  typedef boost::shared_ptr< ::swarm_center::mArmReqRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::swarm_center::mArmReqRequest_<ContainerAllocator> const> ConstPtr;

}; // struct mArmReqRequest_

typedef ::swarm_center::mArmReqRequest_<std::allocator<void> > mArmReqRequest;

typedef boost::shared_ptr< ::swarm_center::mArmReqRequest > mArmReqRequestPtr;
typedef boost::shared_ptr< ::swarm_center::mArmReqRequest const> mArmReqRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::swarm_center::mArmReqRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::swarm_center::mArmReqRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace swarm_center

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'swarm_center': ['/home/wade/SJTU-swarm/swarm_ws/src/swarm_center/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::swarm_center::mArmReqRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::swarm_center::mArmReqRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::swarm_center::mArmReqRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::swarm_center::mArmReqRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::swarm_center::mArmReqRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::swarm_center::mArmReqRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::swarm_center::mArmReqRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5c9fb1a886e81e3162a5c87bf55c072b";
  }

  static const char* value(const ::swarm_center::mArmReqRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5c9fb1a886e81e31ULL;
  static const uint64_t static_value2 = 0x62a5c87bf55c072bULL;
};

template<class ContainerAllocator>
struct DataType< ::swarm_center::mArmReqRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "swarm_center/mArmReqRequest";
  }

  static const char* value(const ::swarm_center::mArmReqRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::swarm_center::mArmReqRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 a\n\
";
  }

  static const char* value(const ::swarm_center::mArmReqRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::swarm_center::mArmReqRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.a);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct mArmReqRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::swarm_center::mArmReqRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::swarm_center::mArmReqRequest_<ContainerAllocator>& v)
  {
    s << indent << "a: ";
    Printer<int32_t>::stream(s, indent + "  ", v.a);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SWARM_CENTER_MESSAGE_MARMREQREQUEST_H

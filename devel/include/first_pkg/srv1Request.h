// Generated by gencpp from file first_pkg/srv1Request.msg
// DO NOT EDIT!


#ifndef FIRST_PKG_MESSAGE_SRV1REQUEST_H
#define FIRST_PKG_MESSAGE_SRV1REQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace first_pkg
{
template <class ContainerAllocator>
struct srv1Request_
{
  typedef srv1Request_<ContainerAllocator> Type;

  srv1Request_()
    : num1(0)
    , num2(0)  {
    }
  srv1Request_(const ContainerAllocator& _alloc)
    : num1(0)
    , num2(0)  {
  (void)_alloc;
    }



   typedef int32_t _num1_type;
  _num1_type num1;

   typedef int32_t _num2_type;
  _num2_type num2;





  typedef boost::shared_ptr< ::first_pkg::srv1Request_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::first_pkg::srv1Request_<ContainerAllocator> const> ConstPtr;

}; // struct srv1Request_

typedef ::first_pkg::srv1Request_<std::allocator<void> > srv1Request;

typedef boost::shared_ptr< ::first_pkg::srv1Request > srv1RequestPtr;
typedef boost::shared_ptr< ::first_pkg::srv1Request const> srv1RequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::first_pkg::srv1Request_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::first_pkg::srv1Request_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace first_pkg

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::first_pkg::srv1Request_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::first_pkg::srv1Request_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::first_pkg::srv1Request_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::first_pkg::srv1Request_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::first_pkg::srv1Request_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::first_pkg::srv1Request_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::first_pkg::srv1Request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c68f3979e1a90aac7d1c75a1096d1e60";
  }

  static const char* value(const ::first_pkg::srv1Request_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc68f3979e1a90aacULL;
  static const uint64_t static_value2 = 0x7d1c75a1096d1e60ULL;
};

template<class ContainerAllocator>
struct DataType< ::first_pkg::srv1Request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "first_pkg/srv1Request";
  }

  static const char* value(const ::first_pkg::srv1Request_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::first_pkg::srv1Request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 num1\n\
int32 num2\n\
";
  }

  static const char* value(const ::first_pkg::srv1Request_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::first_pkg::srv1Request_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.num1);
      stream.next(m.num2);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct srv1Request_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::first_pkg::srv1Request_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::first_pkg::srv1Request_<ContainerAllocator>& v)
  {
    s << indent << "num1: ";
    Printer<int32_t>::stream(s, indent + "  ", v.num1);
    s << indent << "num2: ";
    Printer<int32_t>::stream(s, indent + "  ", v.num2);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FIRST_PKG_MESSAGE_SRV1REQUEST_H

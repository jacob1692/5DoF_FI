// Generated by gencpp from file robot_shared_control/FootInputMsg.msg
// DO NOT EDIT!


#ifndef ROBOT_SHARED_CONTROL_MESSAGE_FOOTINPUTMSG_H
#define ROBOT_SHARED_CONTROL_MESSAGE_FOOTINPUTMSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robot_shared_control
{
template <class ContainerAllocator>
struct FootInputMsg_
{
  typedef FootInputMsg_<ContainerAllocator> Type;

  FootInputMsg_()
    : FxDes(0.0)
    , FyDes(0.0)
    , TphiDes(0.0)
    , TthetaDes(0.0)
    , TpsiDes(0.0)
    , stateDes(0)  {
    }
  FootInputMsg_(const ContainerAllocator& _alloc)
    : FxDes(0.0)
    , FyDes(0.0)
    , TphiDes(0.0)
    , TthetaDes(0.0)
    , TpsiDes(0.0)
    , stateDes(0)  {
  (void)_alloc;
    }



   typedef float _FxDes_type;
  _FxDes_type FxDes;

   typedef float _FyDes_type;
  _FyDes_type FyDes;

   typedef float _TphiDes_type;
  _TphiDes_type TphiDes;

   typedef float _TthetaDes_type;
  _TthetaDes_type TthetaDes;

   typedef float _TpsiDes_type;
  _TpsiDes_type TpsiDes;

   typedef int16_t _stateDes_type;
  _stateDes_type stateDes;




  typedef boost::shared_ptr< ::robot_shared_control::FootInputMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_shared_control::FootInputMsg_<ContainerAllocator> const> ConstPtr;

}; // struct FootInputMsg_

typedef ::robot_shared_control::FootInputMsg_<std::allocator<void> > FootInputMsg;

typedef boost::shared_ptr< ::robot_shared_control::FootInputMsg > FootInputMsgPtr;
typedef boost::shared_ptr< ::robot_shared_control::FootInputMsg const> FootInputMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robot_shared_control::FootInputMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robot_shared_control::FootInputMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace robot_shared_control

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'robot_shared_control': ['/home/walid/catkin_ws/src/robot_shared_control/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::robot_shared_control::FootInputMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_shared_control::FootInputMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_shared_control::FootInputMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_shared_control::FootInputMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_shared_control::FootInputMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_shared_control::FootInputMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robot_shared_control::FootInputMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a39283cd0a6414e571a661f8b3a644bc";
  }

  static const char* value(const ::robot_shared_control::FootInputMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa39283cd0a6414e5ULL;
  static const uint64_t static_value2 = 0x71a661f8b3a644bcULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_shared_control::FootInputMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robot_shared_control/FootInputMsg";
  }

  static const char* value(const ::robot_shared_control::FootInputMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robot_shared_control::FootInputMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# FootInputMsg.msg\n\
\n\
float32 FxDes\n\
float32 FyDes\n\
float32 TphiDes\n\
float32 TthetaDes\n\
float32 TpsiDes\n\
\n\
int16 stateDes\n\
";
  }

  static const char* value(const ::robot_shared_control::FootInputMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robot_shared_control::FootInputMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.FxDes);
      stream.next(m.FyDes);
      stream.next(m.TphiDes);
      stream.next(m.TthetaDes);
      stream.next(m.TpsiDes);
      stream.next(m.stateDes);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FootInputMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_shared_control::FootInputMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robot_shared_control::FootInputMsg_<ContainerAllocator>& v)
  {
    s << indent << "FxDes: ";
    Printer<float>::stream(s, indent + "  ", v.FxDes);
    s << indent << "FyDes: ";
    Printer<float>::stream(s, indent + "  ", v.FyDes);
    s << indent << "TphiDes: ";
    Printer<float>::stream(s, indent + "  ", v.TphiDes);
    s << indent << "TthetaDes: ";
    Printer<float>::stream(s, indent + "  ", v.TthetaDes);
    s << indent << "TpsiDes: ";
    Printer<float>::stream(s, indent + "  ", v.TpsiDes);
    s << indent << "stateDes: ";
    Printer<int16_t>::stream(s, indent + "  ", v.stateDes);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOT_SHARED_CONTROL_MESSAGE_FOOTINPUTMSG_H

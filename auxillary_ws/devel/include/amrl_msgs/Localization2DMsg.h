// Generated by gencpp from file amrl_msgs/Localization2DMsg.msg
// DO NOT EDIT!


#ifndef AMRL_MSGS_MESSAGE_LOCALIZATION2DMSG_H
#define AMRL_MSGS_MESSAGE_LOCALIZATION2DMSG_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <amrl_msgs/Pose2Df.h>

namespace amrl_msgs
{
template <class ContainerAllocator>
struct Localization2DMsg_
{
  typedef Localization2DMsg_<ContainerAllocator> Type;

  Localization2DMsg_()
    : header()
    , pose()
    , map()  {
    }
  Localization2DMsg_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , pose(_alloc)
    , map(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::amrl_msgs::Pose2Df_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _map_type;
  _map_type map;





  typedef boost::shared_ptr< ::amrl_msgs::Localization2DMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::amrl_msgs::Localization2DMsg_<ContainerAllocator> const> ConstPtr;

}; // struct Localization2DMsg_

typedef ::amrl_msgs::Localization2DMsg_<std::allocator<void> > Localization2DMsg;

typedef boost::shared_ptr< ::amrl_msgs::Localization2DMsg > Localization2DMsgPtr;
typedef boost::shared_ptr< ::amrl_msgs::Localization2DMsg const> Localization2DMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::amrl_msgs::Localization2DMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::amrl_msgs::Localization2DMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::amrl_msgs::Localization2DMsg_<ContainerAllocator1> & lhs, const ::amrl_msgs::Localization2DMsg_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.pose == rhs.pose &&
    lhs.map == rhs.map;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::amrl_msgs::Localization2DMsg_<ContainerAllocator1> & lhs, const ::amrl_msgs::Localization2DMsg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace amrl_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::amrl_msgs::Localization2DMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::amrl_msgs::Localization2DMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::amrl_msgs::Localization2DMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::amrl_msgs::Localization2DMsg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::amrl_msgs::Localization2DMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::amrl_msgs::Localization2DMsg_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::amrl_msgs::Localization2DMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4ce450daa8564e2fb59b919aebbbe26e";
  }

  static const char* value(const ::amrl_msgs::Localization2DMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4ce450daa8564e2fULL;
  static const uint64_t static_value2 = 0xb59b919aebbbe26eULL;
};

template<class ContainerAllocator>
struct DataType< ::amrl_msgs::Localization2DMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "amrl_msgs/Localization2DMsg";
  }

  static const char* value(const ::amrl_msgs::Localization2DMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::amrl_msgs::Localization2DMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"Pose2Df pose\n"
"string map\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: amrl_msgs/Pose2Df\n"
"float32 x\n"
"float32 y\n"
"float32 theta\n"
;
  }

  static const char* value(const ::amrl_msgs::Localization2DMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::amrl_msgs::Localization2DMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.pose);
      stream.next(m.map);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Localization2DMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::amrl_msgs::Localization2DMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::amrl_msgs::Localization2DMsg_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::amrl_msgs::Pose2Df_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "map: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.map);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AMRL_MSGS_MESSAGE_LOCALIZATION2DMSG_H

// Generated by gencpp from file ti_mmwave_rospkg/RadarScan.msg
// DO NOT EDIT!


#ifndef TI_MMWAVE_ROSPKG_MESSAGE_RADARSCAN_H
#define TI_MMWAVE_ROSPKG_MESSAGE_RADARSCAN_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace ti_mmwave_rospkg
{
template <class ContainerAllocator>
struct RadarScan_
{
  typedef RadarScan_<ContainerAllocator> Type;

  RadarScan_()
    : header()
    , point_id(0)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , range(0.0)
    , velocity(0.0)
    , doppler_bin(0)
    , bearing(0.0)
    , intensity(0.0)  {
    }
  RadarScan_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , point_id(0)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , range(0.0)
    , velocity(0.0)
    , doppler_bin(0)
    , bearing(0.0)
    , intensity(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint16_t _point_id_type;
  _point_id_type point_id;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;

   typedef float _range_type;
  _range_type range;

   typedef float _velocity_type;
  _velocity_type velocity;

   typedef uint16_t _doppler_bin_type;
  _doppler_bin_type doppler_bin;

   typedef float _bearing_type;
  _bearing_type bearing;

   typedef float _intensity_type;
  _intensity_type intensity;





  typedef boost::shared_ptr< ::ti_mmwave_rospkg::RadarScan_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ti_mmwave_rospkg::RadarScan_<ContainerAllocator> const> ConstPtr;

}; // struct RadarScan_

typedef ::ti_mmwave_rospkg::RadarScan_<std::allocator<void> > RadarScan;

typedef boost::shared_ptr< ::ti_mmwave_rospkg::RadarScan > RadarScanPtr;
typedef boost::shared_ptr< ::ti_mmwave_rospkg::RadarScan const> RadarScanConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ti_mmwave_rospkg::RadarScan_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ti_mmwave_rospkg::RadarScan_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ti_mmwave_rospkg

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'ti_mmwave_rospkg': ['/home/ccpang/Desktop/Git/ros_mmradar/src/ti_mmwave_rospkg/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ti_mmwave_rospkg::RadarScan_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ti_mmwave_rospkg::RadarScan_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ti_mmwave_rospkg::RadarScan_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ti_mmwave_rospkg::RadarScan_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ti_mmwave_rospkg::RadarScan_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ti_mmwave_rospkg::RadarScan_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ti_mmwave_rospkg::RadarScan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7a726cbc7d2934bb55d96dada9040f86";
  }

  static const char* value(const ::ti_mmwave_rospkg::RadarScan_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7a726cbc7d2934bbULL;
  static const uint64_t static_value2 = 0x55d96dada9040f86ULL;
};

template<class ContainerAllocator>
struct DataType< ::ti_mmwave_rospkg::RadarScan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ti_mmwave_rospkg/RadarScan";
  }

  static const char* value(const ::ti_mmwave_rospkg::RadarScan_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ti_mmwave_rospkg::RadarScan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"uint16 point_id\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
"float32 range\n"
"float32 velocity\n"
"uint16 doppler_bin\n"
"float32 bearing\n"
"float32 intensity\n"
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
;
  }

  static const char* value(const ::ti_mmwave_rospkg::RadarScan_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ti_mmwave_rospkg::RadarScan_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.point_id);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.range);
      stream.next(m.velocity);
      stream.next(m.doppler_bin);
      stream.next(m.bearing);
      stream.next(m.intensity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RadarScan_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ti_mmwave_rospkg::RadarScan_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ti_mmwave_rospkg::RadarScan_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "point_id: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.point_id);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
    s << indent << "range: ";
    Printer<float>::stream(s, indent + "  ", v.range);
    s << indent << "velocity: ";
    Printer<float>::stream(s, indent + "  ", v.velocity);
    s << indent << "doppler_bin: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.doppler_bin);
    s << indent << "bearing: ";
    Printer<float>::stream(s, indent + "  ", v.bearing);
    s << indent << "intensity: ";
    Printer<float>::stream(s, indent + "  ", v.intensity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TI_MMWAVE_ROSPKG_MESSAGE_RADARSCAN_H

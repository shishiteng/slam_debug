// Generated by gencpp from file msckf/FeatureMeasurement.msg
// DO NOT EDIT!


#ifndef MSCKF_MESSAGE_FEATUREMEASUREMENT_H
#define MSCKF_MESSAGE_FEATUREMEASUREMENT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace msckf
{
template <class ContainerAllocator>
struct FeatureMeasurement_
{
  typedef FeatureMeasurement_<ContainerAllocator> Type;

  FeatureMeasurement_()
    : id(0)
    , u0(0.0)
    , v0(0.0)
    , u1(0.0)
    , v1(0.0)
    , du0(0.0)
    , dv0(0.0)
    , du1(0.0)
    , dv1(0.0)  {
    }
  FeatureMeasurement_(const ContainerAllocator& _alloc)
    : id(0)
    , u0(0.0)
    , v0(0.0)
    , u1(0.0)
    , v1(0.0)
    , du0(0.0)
    , dv0(0.0)
    , du1(0.0)
    , dv1(0.0)  {
  (void)_alloc;
    }



   typedef uint64_t _id_type;
  _id_type id;

   typedef double _u0_type;
  _u0_type u0;

   typedef double _v0_type;
  _v0_type v0;

   typedef double _u1_type;
  _u1_type u1;

   typedef double _v1_type;
  _v1_type v1;

   typedef double _du0_type;
  _du0_type du0;

   typedef double _dv0_type;
  _dv0_type dv0;

   typedef double _du1_type;
  _du1_type du1;

   typedef double _dv1_type;
  _dv1_type dv1;




  typedef boost::shared_ptr< ::msckf::FeatureMeasurement_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msckf::FeatureMeasurement_<ContainerAllocator> const> ConstPtr;

}; // struct FeatureMeasurement_

typedef ::msckf::FeatureMeasurement_<std::allocator<void> > FeatureMeasurement;

typedef boost::shared_ptr< ::msckf::FeatureMeasurement > FeatureMeasurementPtr;
typedef boost::shared_ptr< ::msckf::FeatureMeasurement const> FeatureMeasurementConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msckf::FeatureMeasurement_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msckf::FeatureMeasurement_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace msckf

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'msckf': ['/home/sst/catkin_ws2/src/msckf_7251live/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::msckf::FeatureMeasurement_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msckf::FeatureMeasurement_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msckf::FeatureMeasurement_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msckf::FeatureMeasurement_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msckf::FeatureMeasurement_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msckf::FeatureMeasurement_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msckf::FeatureMeasurement_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6df0ff53917feff7df8fe51bbe161c37";
  }

  static const char* value(const ::msckf::FeatureMeasurement_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6df0ff53917feff7ULL;
  static const uint64_t static_value2 = 0xdf8fe51bbe161c37ULL;
};

template<class ContainerAllocator>
struct DataType< ::msckf::FeatureMeasurement_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msckf/FeatureMeasurement";
  }

  static const char* value(const ::msckf::FeatureMeasurement_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msckf::FeatureMeasurement_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint64 id\n\
# Normalized feature coordinates (with identity intrinsic matrix)\n\
float64 u0 # horizontal coordinate in cam0\n\
float64 v0 # vertical coordinate in cam0\n\
float64 u1 # horizontal coordinate in cam0\n\
float64 v1 # vertical coordinate in cam0\n\
float64 du0 #distorted\n\
float64 dv0\n\
float64 du1\n\
float64 dv1\n\
";
  }

  static const char* value(const ::msckf::FeatureMeasurement_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msckf::FeatureMeasurement_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.u0);
      stream.next(m.v0);
      stream.next(m.u1);
      stream.next(m.v1);
      stream.next(m.du0);
      stream.next(m.dv0);
      stream.next(m.du1);
      stream.next(m.dv1);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FeatureMeasurement_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msckf::FeatureMeasurement_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msckf::FeatureMeasurement_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.id);
    s << indent << "u0: ";
    Printer<double>::stream(s, indent + "  ", v.u0);
    s << indent << "v0: ";
    Printer<double>::stream(s, indent + "  ", v.v0);
    s << indent << "u1: ";
    Printer<double>::stream(s, indent + "  ", v.u1);
    s << indent << "v1: ";
    Printer<double>::stream(s, indent + "  ", v.v1);
    s << indent << "du0: ";
    Printer<double>::stream(s, indent + "  ", v.du0);
    s << indent << "dv0: ";
    Printer<double>::stream(s, indent + "  ", v.dv0);
    s << indent << "du1: ";
    Printer<double>::stream(s, indent + "  ", v.du1);
    s << indent << "dv1: ";
    Printer<double>::stream(s, indent + "  ", v.dv1);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSCKF_MESSAGE_FEATUREMEASUREMENT_H

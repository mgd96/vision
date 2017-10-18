// This is an automatically generated file.
// Generated from this sensor_msgs_CompressedImage.msg definition:
//   # This message contains a compressed image
//   
//   Header header        # Header timestamp should be acquisition time of image
//                        # Header frame_id should be optical frame of camera
//                        # origin of frame should be optical center of cameara
//                        # +x should point to the right in the image
//                        # +y should point down in the image
//                        # +z should point into to plane of the image
//   
//   string format        # Specifies the format of the data
//                        #   Acceptable values:
//                        #     jpeg, png
//   uint8[] data         # Compressed image buffer
//   
// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARPMSG_TYPE_sensor_msgs_CompressedImage
#define YARPMSG_TYPE_sensor_msgs_CompressedImage

#include <string>
#include <vector>
#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include "TickTime.h"
#include "std_msgs_Header.h"

class sensor_msgs_CompressedImage : public yarp::os::idl::WirePortable {
public:
  std_msgs_Header header;
  std::string format;
  std::vector<unsigned char> data;

  sensor_msgs_CompressedImage() {
  }

  void clear() {
    // *** header ***
    header.clear();

    // *** format ***
    format = "";

    // *** data ***
    data.clear();
  }

  bool readBare(yarp::os::ConnectionReader& connection) YARP_OVERRIDE {
    // *** header ***
    if (!header.read(connection)) return false;

    // *** format ***
    int len = connection.expectInt();
    format.resize(len);
    if (!connection.expectBlock((char*)format.c_str(),len)) return false;

    // *** data ***
    len = connection.expectInt();
    data.resize(len);
    if (len > 0 && !connection.expectBlock((char*)&data[0],sizeof(unsigned char)*len)) return false;
    return !connection.isError();
  }

  bool readBottle(yarp::os::ConnectionReader& connection) YARP_OVERRIDE {
    connection.convertTextMode();
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(3)) return false;

    // *** header ***
    if (!header.read(connection)) return false;

    // *** format ***
    if (!reader.readString(format)) return false;

    // *** data ***
    if (connection.expectInt()!=(BOTTLE_TAG_LIST|BOTTLE_TAG_INT)) return false;
    int len = connection.expectInt();
    data.resize(len);
    for (int i=0; i<len; i++) {
      data[i] = (unsigned char)connection.expectInt();
    }
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::read;
  bool read(yarp::os::ConnectionReader& connection) YARP_OVERRIDE {
    if (connection.isBareMode()) return readBare(connection);
    return readBottle(connection);
  }

  bool writeBare(yarp::os::ConnectionWriter& connection) YARP_OVERRIDE {
    // *** header ***
    if (!header.write(connection)) return false;

    // *** format ***
    connection.appendInt(format.length());
    connection.appendExternalBlock((char*)format.c_str(),format.length());

    // *** data ***
    connection.appendInt(data.size());
    if (data.size()>0) {connection.appendExternalBlock((char*)&data[0],sizeof(unsigned char)*data.size());}
    return !connection.isError();
  }

  bool writeBottle(yarp::os::ConnectionWriter& connection) YARP_OVERRIDE {
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(3);

    // *** header ***
    if (!header.write(connection)) return false;

    // *** format ***
    connection.appendInt(BOTTLE_TAG_STRING);
    connection.appendInt(format.length());
    connection.appendExternalBlock((char*)format.c_str(),format.length());

    // *** data ***
    connection.appendInt(BOTTLE_TAG_LIST|BOTTLE_TAG_INT);
    connection.appendInt(data.size());
    for (size_t i=0; i<data.size(); i++) {
      connection.appendInt((int)data[i]);
    }
    connection.convertTextMode();
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::write;
  bool write(yarp::os::ConnectionWriter& connection) YARP_OVERRIDE {
    if (connection.isBareMode()) return writeBare(connection);
    return writeBottle(connection);
  }

  // This class will serialize ROS style or YARP style depending on protocol.
  // If you need to force a serialization style, use one of these classes:
  typedef yarp::os::idl::BareStyle<sensor_msgs_CompressedImage> rosStyle;
  typedef yarp::os::idl::BottleStyle<sensor_msgs_CompressedImage> bottleStyle;

  // Give source text for class, ROS will need this
  yarp::os::ConstString getTypeText() {
    return "# This message contains a compressed image\n\
\n\
Header header        # Header timestamp should be acquisition time of image\n\
                     # Header frame_id should be optical frame of camera\n\
                     # origin of frame should be optical center of cameara\n\
                     # +x should point to the right in the image\n\
                     # +y should point down in the image\n\
                     # +z should point into to plane of the image\n\
\n\
string format        # Specifies the format of the data\n\
                     #   Acceptable values:\n\
                     #     jpeg, png\n\
uint8[] data         # Compressed image buffer\n\
\n================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  // Name the class, ROS will need this
  yarp::os::Type getType() YARP_OVERRIDE {
    yarp::os::Type typ = yarp::os::Type::byName("sensor_msgs/CompressedImage","sensor_msgs/CompressedImage");
    typ.addProperty("md5sum",yarp::os::Value("8f7a12909da2c9d3332d540a0977563f"));
    typ.addProperty("message_definition",yarp::os::Value(getTypeText()));
    return typ;
  }
};

#endif

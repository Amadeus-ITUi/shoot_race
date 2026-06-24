// Auto-generated. Do not edit!

// (in-package target_detector.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let MarkerPixel = require('./MarkerPixel.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class MarkerPixelArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.markers = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('markers')) {
        this.markers = initObj.markers
      }
      else {
        this.markers = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MarkerPixelArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [markers]
    // Serialize the length for message field [markers]
    bufferOffset = _serializer.uint32(obj.markers.length, buffer, bufferOffset);
    obj.markers.forEach((val) => {
      bufferOffset = MarkerPixel.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MarkerPixelArray
    let len;
    let data = new MarkerPixelArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [markers]
    // Deserialize array length for message field [markers]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.markers = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.markers[i] = MarkerPixel.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 12 * object.markers.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'target_detector/MarkerPixelArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '44de9ad6def1705d14c002806a70e9db';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    MarkerPixel[] markers
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: target_detector/MarkerPixel
    # AR 码像素坐标
    uint32 id          # 标签 ID
    float32 u          # 像素列坐标 (x)
    float32 v          # 像素行坐标 (y)
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MarkerPixelArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.markers !== undefined) {
      resolved.markers = new Array(msg.markers.length);
      for (let i = 0; i < resolved.markers.length; ++i) {
        resolved.markers[i] = MarkerPixel.Resolve(msg.markers[i]);
      }
    }
    else {
      resolved.markers = []
    }

    return resolved;
    }
};

module.exports = MarkerPixelArray;

// Auto-generated. Do not edit!

// (in-package first_project.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class sector_time {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.current_sector = null;
      this.current_sector_time = null;
      this.current_sector_mean_speed = null;
    }
    else {
      if (initObj.hasOwnProperty('current_sector')) {
        this.current_sector = initObj.current_sector
      }
      else {
        this.current_sector = 0;
      }
      if (initObj.hasOwnProperty('current_sector_time')) {
        this.current_sector_time = initObj.current_sector_time
      }
      else {
        this.current_sector_time = 0.0;
      }
      if (initObj.hasOwnProperty('current_sector_mean_speed')) {
        this.current_sector_mean_speed = initObj.current_sector_mean_speed
      }
      else {
        this.current_sector_mean_speed = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type sector_time
    // Serialize message field [current_sector]
    bufferOffset = _serializer.int32(obj.current_sector, buffer, bufferOffset);
    // Serialize message field [current_sector_time]
    bufferOffset = _serializer.float64(obj.current_sector_time, buffer, bufferOffset);
    // Serialize message field [current_sector_mean_speed]
    bufferOffset = _serializer.float64(obj.current_sector_mean_speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type sector_time
    let len;
    let data = new sector_time(null);
    // Deserialize message field [current_sector]
    data.current_sector = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [current_sector_time]
    data.current_sector_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [current_sector_mean_speed]
    data.current_sector_mean_speed = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'first_project/sector_time';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1faea2573a9ef916b33bea5b9868fa6d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 current_sector
    float64 current_sector_time
    float64 current_sector_mean_speed
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new sector_time(null);
    if (msg.current_sector !== undefined) {
      resolved.current_sector = msg.current_sector;
    }
    else {
      resolved.current_sector = 0
    }

    if (msg.current_sector_time !== undefined) {
      resolved.current_sector_time = msg.current_sector_time;
    }
    else {
      resolved.current_sector_time = 0.0
    }

    if (msg.current_sector_mean_speed !== undefined) {
      resolved.current_sector_mean_speed = msg.current_sector_mean_speed;
    }
    else {
      resolved.current_sector_mean_speed = 0.0
    }

    return resolved;
    }
};

module.exports = sector_time;

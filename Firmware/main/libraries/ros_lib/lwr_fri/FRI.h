#ifndef _ROS_lwr_fri_FRI_h
#define _ROS_lwr_fri_FRI_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace lwr_fri
{

  class FRI : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t name_length;
      typedef char* _name_type;
      _name_type st_name;
      _name_type * name;
      uint32_t position_length;
      typedef float _position_type;
      _position_type st_position;
      _position_type * position;
      uint32_t effort_length;
      typedef float _effort_type;
      _effort_type st_effort;
      _effort_type * effort;
      uint32_t stiffness_length;
      typedef float _stiffness_type;
      _stiffness_type st_stiffness;
      _stiffness_type * stiffness;
      uint32_t damping_length;
      typedef float _damping_type;
      _damping_type st_damping;
      _damping_type * damping;
      typedef int64_t _FRI_STATE_type;
      _FRI_STATE_type FRI_STATE;
      typedef int64_t _FRI_QUALITY_type;
      _FRI_QUALITY_type FRI_QUALITY;
      typedef int64_t _FRI_CTRL_type;
      _FRI_CTRL_type FRI_CTRL;
      typedef int64_t _ROBOT_CTRL_MODE_type;
      _ROBOT_CTRL_MODE_type ROBOT_CTRL_MODE;
      typedef bool _IsRobotArmPowerOn_type;
      _IsRobotArmPowerOn_type IsRobotArmPowerOn;
      typedef bool _DoesAnyDriveSignalAnError_type;
      _DoesAnyDriveSignalAnError_type DoesAnyDriveSignalAnError;

    FRI():
      header(),
      name_length(0), name(NULL),
      position_length(0), position(NULL),
      effort_length(0), effort(NULL),
      stiffness_length(0), stiffness(NULL),
      damping_length(0), damping(NULL),
      FRI_STATE(0),
      FRI_QUALITY(0),
      FRI_CTRL(0),
      ROBOT_CTRL_MODE(0),
      IsRobotArmPowerOn(0),
      DoesAnyDriveSignalAnError(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->name_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->name_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->name_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->name_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->name_length);
      for( uint32_t i = 0; i < name_length; i++){
      uint32_t length_namei = strlen(this->name[i]);
      varToArr(outbuffer + offset, length_namei);
      offset += 4;
      memcpy(outbuffer + offset, this->name[i], length_namei);
      offset += length_namei;
      }
      *(outbuffer + offset + 0) = (this->position_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->position_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->position_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->position_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_length);
      for( uint32_t i = 0; i < position_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->position[i]);
      }
      *(outbuffer + offset + 0) = (this->effort_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->effort_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->effort_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->effort_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->effort_length);
      for( uint32_t i = 0; i < effort_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->effort[i]);
      }
      *(outbuffer + offset + 0) = (this->stiffness_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stiffness_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stiffness_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stiffness_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stiffness_length);
      for( uint32_t i = 0; i < stiffness_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->stiffness[i]);
      }
      *(outbuffer + offset + 0) = (this->damping_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->damping_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->damping_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->damping_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->damping_length);
      for( uint32_t i = 0; i < damping_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->damping[i]);
      }
      union {
        int64_t real;
        uint64_t base;
      } u_FRI_STATE;
      u_FRI_STATE.real = this->FRI_STATE;
      *(outbuffer + offset + 0) = (u_FRI_STATE.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_FRI_STATE.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_FRI_STATE.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_FRI_STATE.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_FRI_STATE.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_FRI_STATE.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_FRI_STATE.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_FRI_STATE.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->FRI_STATE);
      union {
        int64_t real;
        uint64_t base;
      } u_FRI_QUALITY;
      u_FRI_QUALITY.real = this->FRI_QUALITY;
      *(outbuffer + offset + 0) = (u_FRI_QUALITY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_FRI_QUALITY.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_FRI_QUALITY.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_FRI_QUALITY.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_FRI_QUALITY.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_FRI_QUALITY.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_FRI_QUALITY.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_FRI_QUALITY.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->FRI_QUALITY);
      union {
        int64_t real;
        uint64_t base;
      } u_FRI_CTRL;
      u_FRI_CTRL.real = this->FRI_CTRL;
      *(outbuffer + offset + 0) = (u_FRI_CTRL.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_FRI_CTRL.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_FRI_CTRL.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_FRI_CTRL.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_FRI_CTRL.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_FRI_CTRL.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_FRI_CTRL.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_FRI_CTRL.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->FRI_CTRL);
      union {
        int64_t real;
        uint64_t base;
      } u_ROBOT_CTRL_MODE;
      u_ROBOT_CTRL_MODE.real = this->ROBOT_CTRL_MODE;
      *(outbuffer + offset + 0) = (u_ROBOT_CTRL_MODE.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ROBOT_CTRL_MODE.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ROBOT_CTRL_MODE.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ROBOT_CTRL_MODE.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_ROBOT_CTRL_MODE.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_ROBOT_CTRL_MODE.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_ROBOT_CTRL_MODE.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_ROBOT_CTRL_MODE.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->ROBOT_CTRL_MODE);
      union {
        bool real;
        uint8_t base;
      } u_IsRobotArmPowerOn;
      u_IsRobotArmPowerOn.real = this->IsRobotArmPowerOn;
      *(outbuffer + offset + 0) = (u_IsRobotArmPowerOn.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->IsRobotArmPowerOn);
      union {
        bool real;
        uint8_t base;
      } u_DoesAnyDriveSignalAnError;
      u_DoesAnyDriveSignalAnError.real = this->DoesAnyDriveSignalAnError;
      *(outbuffer + offset + 0) = (u_DoesAnyDriveSignalAnError.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->DoesAnyDriveSignalAnError);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t name_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->name_length);
      if(name_lengthT > name_length)
        this->name = (char**)realloc(this->name, name_lengthT * sizeof(char*));
      name_length = name_lengthT;
      for( uint32_t i = 0; i < name_length; i++){
      uint32_t length_st_name;
      arrToVar(length_st_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_name-1]=0;
      this->st_name = (char *)(inbuffer + offset-1);
      offset += length_st_name;
        memcpy( &(this->name[i]), &(this->st_name), sizeof(char*));
      }
      uint32_t position_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      position_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      position_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      position_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->position_length);
      if(position_lengthT > position_length)
        this->position = (float*)realloc(this->position, position_lengthT * sizeof(float));
      position_length = position_lengthT;
      for( uint32_t i = 0; i < position_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_position));
        memcpy( &(this->position[i]), &(this->st_position), sizeof(float));
      }
      uint32_t effort_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->effort_length);
      if(effort_lengthT > effort_length)
        this->effort = (float*)realloc(this->effort, effort_lengthT * sizeof(float));
      effort_length = effort_lengthT;
      for( uint32_t i = 0; i < effort_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_effort));
        memcpy( &(this->effort[i]), &(this->st_effort), sizeof(float));
      }
      uint32_t stiffness_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      stiffness_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      stiffness_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      stiffness_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->stiffness_length);
      if(stiffness_lengthT > stiffness_length)
        this->stiffness = (float*)realloc(this->stiffness, stiffness_lengthT * sizeof(float));
      stiffness_length = stiffness_lengthT;
      for( uint32_t i = 0; i < stiffness_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_stiffness));
        memcpy( &(this->stiffness[i]), &(this->st_stiffness), sizeof(float));
      }
      uint32_t damping_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      damping_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      damping_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      damping_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->damping_length);
      if(damping_lengthT > damping_length)
        this->damping = (float*)realloc(this->damping, damping_lengthT * sizeof(float));
      damping_length = damping_lengthT;
      for( uint32_t i = 0; i < damping_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_damping));
        memcpy( &(this->damping[i]), &(this->st_damping), sizeof(float));
      }
      union {
        int64_t real;
        uint64_t base;
      } u_FRI_STATE;
      u_FRI_STATE.base = 0;
      u_FRI_STATE.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_FRI_STATE.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_FRI_STATE.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_FRI_STATE.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_FRI_STATE.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_FRI_STATE.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_FRI_STATE.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_FRI_STATE.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->FRI_STATE = u_FRI_STATE.real;
      offset += sizeof(this->FRI_STATE);
      union {
        int64_t real;
        uint64_t base;
      } u_FRI_QUALITY;
      u_FRI_QUALITY.base = 0;
      u_FRI_QUALITY.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_FRI_QUALITY.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_FRI_QUALITY.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_FRI_QUALITY.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_FRI_QUALITY.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_FRI_QUALITY.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_FRI_QUALITY.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_FRI_QUALITY.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->FRI_QUALITY = u_FRI_QUALITY.real;
      offset += sizeof(this->FRI_QUALITY);
      union {
        int64_t real;
        uint64_t base;
      } u_FRI_CTRL;
      u_FRI_CTRL.base = 0;
      u_FRI_CTRL.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_FRI_CTRL.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_FRI_CTRL.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_FRI_CTRL.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_FRI_CTRL.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_FRI_CTRL.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_FRI_CTRL.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_FRI_CTRL.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->FRI_CTRL = u_FRI_CTRL.real;
      offset += sizeof(this->FRI_CTRL);
      union {
        int64_t real;
        uint64_t base;
      } u_ROBOT_CTRL_MODE;
      u_ROBOT_CTRL_MODE.base = 0;
      u_ROBOT_CTRL_MODE.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ROBOT_CTRL_MODE.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ROBOT_CTRL_MODE.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ROBOT_CTRL_MODE.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_ROBOT_CTRL_MODE.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_ROBOT_CTRL_MODE.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_ROBOT_CTRL_MODE.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_ROBOT_CTRL_MODE.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->ROBOT_CTRL_MODE = u_ROBOT_CTRL_MODE.real;
      offset += sizeof(this->ROBOT_CTRL_MODE);
      union {
        bool real;
        uint8_t base;
      } u_IsRobotArmPowerOn;
      u_IsRobotArmPowerOn.base = 0;
      u_IsRobotArmPowerOn.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->IsRobotArmPowerOn = u_IsRobotArmPowerOn.real;
      offset += sizeof(this->IsRobotArmPowerOn);
      union {
        bool real;
        uint8_t base;
      } u_DoesAnyDriveSignalAnError;
      u_DoesAnyDriveSignalAnError.base = 0;
      u_DoesAnyDriveSignalAnError.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->DoesAnyDriveSignalAnError = u_DoesAnyDriveSignalAnError.real;
      offset += sizeof(this->DoesAnyDriveSignalAnError);
     return offset;
    }

    const char * getType(){ return "lwr_fri/FRI"; };
    const char * getMD5(){ return "1c34faf79d9377b5acf8f04748e1f1a2"; };

  };

}
#endif
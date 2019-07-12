#ifndef _ROS_lwr_fri_JointStates_h
#define _ROS_lwr_fri_JointStates_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace lwr_fri
{

  class JointStates : public ros::Msg
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
      uint32_t velocity_length;
      typedef float _velocity_type;
      _velocity_type st_velocity;
      _velocity_type * velocity;
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

    JointStates():
      header(),
      name_length(0), name(NULL),
      position_length(0), position(NULL),
      velocity_length(0), velocity(NULL),
      effort_length(0), effort(NULL),
      stiffness_length(0), stiffness(NULL),
      damping_length(0), damping(NULL)
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
      *(outbuffer + offset + 0) = (this->velocity_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->velocity_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->velocity_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->velocity_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_length);
      for( uint32_t i = 0; i < velocity_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->velocity[i]);
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
      uint32_t velocity_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->velocity_length);
      if(velocity_lengthT > velocity_length)
        this->velocity = (float*)realloc(this->velocity, velocity_lengthT * sizeof(float));
      velocity_length = velocity_lengthT;
      for( uint32_t i = 0; i < velocity_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_velocity));
        memcpy( &(this->velocity[i]), &(this->st_velocity), sizeof(float));
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
     return offset;
    }

    const char * getType(){ return "lwr_fri/JointStates"; };
    const char * getMD5(){ return "4df6922391f62611c9eb737d6ee12fca"; };

  };

}
#endif
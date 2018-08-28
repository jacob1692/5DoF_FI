#ifndef _ROS_active_debris_control_StampedDepths_h
#define _ROS_active_debris_control_StampedDepths_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace active_debris_control
{

  class StampedDepths : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t depths_length;
      typedef float _depths_type;
      _depths_type st_depths;
      _depths_type * depths;

    StampedDepths():
      header(),
      depths_length(0), depths(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->depths_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->depths_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->depths_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->depths_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->depths_length);
      for( uint32_t i = 0; i < depths_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->depths[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t depths_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      depths_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      depths_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      depths_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->depths_length);
      if(depths_lengthT > depths_length)
        this->depths = (float*)realloc(this->depths, depths_lengthT * sizeof(float));
      depths_length = depths_lengthT;
      for( uint32_t i = 0; i < depths_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_depths));
        memcpy( &(this->depths[i]), &(this->st_depths), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "active_debris_control/StampedDepths"; };
    const char * getMD5(){ return "7c91cee73d43103907052781aabd8348"; };

  };

}
#endif
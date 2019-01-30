#ifndef _ROS_testTeo_Num_h
#define _ROS_testTeo_Num_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace testTeo
{

  class Num : public ros::Msg
  {
    public:
      typedef int64_t _velX_type;
      _velX_type velX;
      typedef int64_t _velY_type;
      _velY_type velY;
      typedef int64_t _velZ_type;
      _velZ_type velZ;

    Num():
      velX(0),
      velY(0),
      velZ(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_velX;
      u_velX.real = this->velX;
      *(outbuffer + offset + 0) = (u_velX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velX.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velX.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velX.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_velX.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_velX.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_velX.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_velX.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->velX);
      union {
        int64_t real;
        uint64_t base;
      } u_velY;
      u_velY.real = this->velY;
      *(outbuffer + offset + 0) = (u_velY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velY.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velY.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velY.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_velY.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_velY.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_velY.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_velY.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->velY);
      union {
        int64_t real;
        uint64_t base;
      } u_velZ;
      u_velZ.real = this->velZ;
      *(outbuffer + offset + 0) = (u_velZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velZ.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velZ.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velZ.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_velZ.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_velZ.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_velZ.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_velZ.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->velZ);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_velX;
      u_velX.base = 0;
      u_velX.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velX.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velX.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velX.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_velX.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_velX.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_velX.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_velX.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->velX = u_velX.real;
      offset += sizeof(this->velX);
      union {
        int64_t real;
        uint64_t base;
      } u_velY;
      u_velY.base = 0;
      u_velY.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velY.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velY.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velY.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_velY.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_velY.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_velY.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_velY.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->velY = u_velY.real;
      offset += sizeof(this->velY);
      union {
        int64_t real;
        uint64_t base;
      } u_velZ;
      u_velZ.base = 0;
      u_velZ.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velZ.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velZ.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velZ.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_velZ.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_velZ.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_velZ.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_velZ.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->velZ = u_velZ.real;
      offset += sizeof(this->velZ);
     return offset;
    }

    const char * getType(){ return "testTeo/Num"; };
    const char * getMD5(){ return "5ea854c36da04dcfb3ee15d31fc64c09"; };

  };

}
#endif
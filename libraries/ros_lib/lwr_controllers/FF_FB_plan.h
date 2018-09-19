#ifndef _ROS_lwr_controllers_FF_FB_plan_h
#define _ROS_lwr_controllers_FF_FB_plan_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Time.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"

namespace lwr_controllers
{

  class FF_FB_plan : public ros::Msg
  {
    public:
      uint32_t times_length;
      typedef std_msgs::Time _times_type;
      _times_type st_times;
      _times_type * times;
      uint32_t ff_length;
      typedef geometry_msgs::Wrench _ff_type;
      _ff_type st_ff;
      _ff_type * ff;
      uint32_t xd_length;
      typedef geometry_msgs::Pose _xd_type;
      _xd_type st_xd;
      _xd_type * xd;
      uint32_t xd_dot_length;
      typedef geometry_msgs::Twist _xd_dot_type;
      _xd_dot_type st_xd_dot;
      _xd_dot_type * xd_dot;
      uint32_t fb_length;
      typedef std_msgs::Float64MultiArray _fb_type;
      _fb_type st_fb;
      _fb_type * fb;

    FF_FB_plan():
      times_length(0), times(NULL),
      ff_length(0), ff(NULL),
      xd_length(0), xd(NULL),
      xd_dot_length(0), xd_dot(NULL),
      fb_length(0), fb(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->times_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->times_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->times_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->times_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->times_length);
      for( uint32_t i = 0; i < times_length; i++){
      offset += this->times[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->ff_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ff_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ff_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ff_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ff_length);
      for( uint32_t i = 0; i < ff_length; i++){
      offset += this->ff[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->xd_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->xd_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->xd_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->xd_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xd_length);
      for( uint32_t i = 0; i < xd_length; i++){
      offset += this->xd[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->xd_dot_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->xd_dot_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->xd_dot_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->xd_dot_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xd_dot_length);
      for( uint32_t i = 0; i < xd_dot_length; i++){
      offset += this->xd_dot[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->fb_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fb_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fb_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fb_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fb_length);
      for( uint32_t i = 0; i < fb_length; i++){
      offset += this->fb[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t times_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      times_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      times_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      times_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->times_length);
      if(times_lengthT > times_length)
        this->times = (std_msgs::Time*)realloc(this->times, times_lengthT * sizeof(std_msgs::Time));
      times_length = times_lengthT;
      for( uint32_t i = 0; i < times_length; i++){
      offset += this->st_times.deserialize(inbuffer + offset);
        memcpy( &(this->times[i]), &(this->st_times), sizeof(std_msgs::Time));
      }
      uint32_t ff_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      ff_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      ff_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      ff_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->ff_length);
      if(ff_lengthT > ff_length)
        this->ff = (geometry_msgs::Wrench*)realloc(this->ff, ff_lengthT * sizeof(geometry_msgs::Wrench));
      ff_length = ff_lengthT;
      for( uint32_t i = 0; i < ff_length; i++){
      offset += this->st_ff.deserialize(inbuffer + offset);
        memcpy( &(this->ff[i]), &(this->st_ff), sizeof(geometry_msgs::Wrench));
      }
      uint32_t xd_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      xd_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      xd_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      xd_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->xd_length);
      if(xd_lengthT > xd_length)
        this->xd = (geometry_msgs::Pose*)realloc(this->xd, xd_lengthT * sizeof(geometry_msgs::Pose));
      xd_length = xd_lengthT;
      for( uint32_t i = 0; i < xd_length; i++){
      offset += this->st_xd.deserialize(inbuffer + offset);
        memcpy( &(this->xd[i]), &(this->st_xd), sizeof(geometry_msgs::Pose));
      }
      uint32_t xd_dot_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      xd_dot_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      xd_dot_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      xd_dot_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->xd_dot_length);
      if(xd_dot_lengthT > xd_dot_length)
        this->xd_dot = (geometry_msgs::Twist*)realloc(this->xd_dot, xd_dot_lengthT * sizeof(geometry_msgs::Twist));
      xd_dot_length = xd_dot_lengthT;
      for( uint32_t i = 0; i < xd_dot_length; i++){
      offset += this->st_xd_dot.deserialize(inbuffer + offset);
        memcpy( &(this->xd_dot[i]), &(this->st_xd_dot), sizeof(geometry_msgs::Twist));
      }
      uint32_t fb_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->fb_length);
      if(fb_lengthT > fb_length)
        this->fb = (std_msgs::Float64MultiArray*)realloc(this->fb, fb_lengthT * sizeof(std_msgs::Float64MultiArray));
      fb_length = fb_lengthT;
      for( uint32_t i = 0; i < fb_length; i++){
      offset += this->st_fb.deserialize(inbuffer + offset);
        memcpy( &(this->fb[i]), &(this->st_fb), sizeof(std_msgs::Float64MultiArray));
      }
     return offset;
    }

    const char * getType(){ return "lwr_controllers/FF_FB_plan"; };
    const char * getMD5(){ return "a96cb59a5005197cccc1498c4103c512"; };

  };

}
#endif
#ifndef _ROS_foot_interfaces_FootMouseMsg_h
#define _ROS_foot_interfaces_FootMouseMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace foot_interfaces
{

  class FootMouseMsg : public ros::Msg
  {
    public:
      typedef uint8_t _event_type;
      _event_type event;
      typedef int32_t _buttonState_type;
      _buttonState_type buttonState;
      typedef int32_t _relX_type;
      _relX_type relX;
      typedef int32_t _relY_type;
      _relY_type relY;
      typedef int32_t _relZ_type;
      _relZ_type relZ;
      typedef int32_t _relWheel_type;
      _relWheel_type relWheel;
      typedef float _filteredRelX_type;
      _filteredRelX_type filteredRelX;
      typedef float _filteredRelY_type;
      _filteredRelY_type filteredRelY;
      typedef float _filteredRelZ_type;
      _filteredRelZ_type filteredRelZ;
      enum { FM_NONE =  0 };
      enum { FM_BTN_A =  1 };
      enum { FM_BTN_B =  2 };
      enum { FM_LEFT_CLICK =  3 };
      enum { FM_RIGHT_CLICK =  4 };
      enum { FM_WHEEL =  5 };
      enum { FM_CURSOR =  6 };

    FootMouseMsg():
      event(0),
      buttonState(0),
      relX(0),
      relY(0),
      relZ(0),
      relWheel(0),
      filteredRelX(0),
      filteredRelY(0),
      filteredRelZ(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->event >> (8 * 0)) & 0xFF;
      offset += sizeof(this->event);
      union {
        int32_t real;
        uint32_t base;
      } u_buttonState;
      u_buttonState.real = this->buttonState;
      *(outbuffer + offset + 0) = (u_buttonState.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_buttonState.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_buttonState.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_buttonState.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->buttonState);
      union {
        int32_t real;
        uint32_t base;
      } u_relX;
      u_relX.real = this->relX;
      *(outbuffer + offset + 0) = (u_relX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_relX.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_relX.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_relX.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->relX);
      union {
        int32_t real;
        uint32_t base;
      } u_relY;
      u_relY.real = this->relY;
      *(outbuffer + offset + 0) = (u_relY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_relY.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_relY.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_relY.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->relY);
      union {
        int32_t real;
        uint32_t base;
      } u_relZ;
      u_relZ.real = this->relZ;
      *(outbuffer + offset + 0) = (u_relZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_relZ.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_relZ.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_relZ.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->relZ);
      union {
        int32_t real;
        uint32_t base;
      } u_relWheel;
      u_relWheel.real = this->relWheel;
      *(outbuffer + offset + 0) = (u_relWheel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_relWheel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_relWheel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_relWheel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->relWheel);
      union {
        float real;
        uint32_t base;
      } u_filteredRelX;
      u_filteredRelX.real = this->filteredRelX;
      *(outbuffer + offset + 0) = (u_filteredRelX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_filteredRelX.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_filteredRelX.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_filteredRelX.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->filteredRelX);
      union {
        float real;
        uint32_t base;
      } u_filteredRelY;
      u_filteredRelY.real = this->filteredRelY;
      *(outbuffer + offset + 0) = (u_filteredRelY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_filteredRelY.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_filteredRelY.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_filteredRelY.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->filteredRelY);
      union {
        float real;
        uint32_t base;
      } u_filteredRelZ;
      u_filteredRelZ.real = this->filteredRelZ;
      *(outbuffer + offset + 0) = (u_filteredRelZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_filteredRelZ.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_filteredRelZ.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_filteredRelZ.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->filteredRelZ);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->event =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->event);
      union {
        int32_t real;
        uint32_t base;
      } u_buttonState;
      u_buttonState.base = 0;
      u_buttonState.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_buttonState.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_buttonState.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_buttonState.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->buttonState = u_buttonState.real;
      offset += sizeof(this->buttonState);
      union {
        int32_t real;
        uint32_t base;
      } u_relX;
      u_relX.base = 0;
      u_relX.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_relX.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_relX.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_relX.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->relX = u_relX.real;
      offset += sizeof(this->relX);
      union {
        int32_t real;
        uint32_t base;
      } u_relY;
      u_relY.base = 0;
      u_relY.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_relY.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_relY.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_relY.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->relY = u_relY.real;
      offset += sizeof(this->relY);
      union {
        int32_t real;
        uint32_t base;
      } u_relZ;
      u_relZ.base = 0;
      u_relZ.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_relZ.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_relZ.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_relZ.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->relZ = u_relZ.real;
      offset += sizeof(this->relZ);
      union {
        int32_t real;
        uint32_t base;
      } u_relWheel;
      u_relWheel.base = 0;
      u_relWheel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_relWheel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_relWheel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_relWheel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->relWheel = u_relWheel.real;
      offset += sizeof(this->relWheel);
      union {
        float real;
        uint32_t base;
      } u_filteredRelX;
      u_filteredRelX.base = 0;
      u_filteredRelX.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_filteredRelX.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_filteredRelX.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_filteredRelX.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->filteredRelX = u_filteredRelX.real;
      offset += sizeof(this->filteredRelX);
      union {
        float real;
        uint32_t base;
      } u_filteredRelY;
      u_filteredRelY.base = 0;
      u_filteredRelY.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_filteredRelY.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_filteredRelY.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_filteredRelY.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->filteredRelY = u_filteredRelY.real;
      offset += sizeof(this->filteredRelY);
      union {
        float real;
        uint32_t base;
      } u_filteredRelZ;
      u_filteredRelZ.base = 0;
      u_filteredRelZ.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_filteredRelZ.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_filteredRelZ.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_filteredRelZ.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->filteredRelZ = u_filteredRelZ.real;
      offset += sizeof(this->filteredRelZ);
     return offset;
    }

    const char * getType(){ return "foot_interfaces/FootMouseMsg"; };
    const char * getMD5(){ return "02215dfabeec681c7855213d3ab2d911"; };

  };

}
#endif
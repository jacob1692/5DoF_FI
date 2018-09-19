#ifndef _ROS_custom_msgs_FootOutputMsg_h
#define _ROS_custom_msgs_FootOutputMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace custom_msgs
{

  class FootOutputMsg : public ros::Msg
  {
    public:
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _phi_type;
      _phi_type phi;
      typedef float _theta_type;
      _theta_type theta;
      typedef float _psi_type;
      _psi_type psi;
      typedef float _Fx_type;
      _Fx_type Fx;
      typedef float _Fy_type;
      _Fy_type Fy;
      typedef float _Tphi_type;
      _Tphi_type Tphi;
      typedef float _Ttheta_type;
      _Ttheta_type Ttheta;
      typedef float _Tpsi_type;
      _Tpsi_type Tpsi;
      typedef int16_t _state_type;
      _state_type state;

    FootOutputMsg():
      x(0),
      y(0),
      phi(0),
      theta(0),
      psi(0),
      Fx(0),
      Fy(0),
      Tphi(0),
      Ttheta(0),
      Tpsi(0),
      state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_phi;
      u_phi.real = this->phi;
      *(outbuffer + offset + 0) = (u_phi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_phi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_phi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_phi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->phi);
      union {
        float real;
        uint32_t base;
      } u_theta;
      u_theta.real = this->theta;
      *(outbuffer + offset + 0) = (u_theta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->theta);
      union {
        float real;
        uint32_t base;
      } u_psi;
      u_psi.real = this->psi;
      *(outbuffer + offset + 0) = (u_psi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_psi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_psi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_psi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->psi);
      union {
        float real;
        uint32_t base;
      } u_Fx;
      u_Fx.real = this->Fx;
      *(outbuffer + offset + 0) = (u_Fx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Fx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Fx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Fx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Fx);
      union {
        float real;
        uint32_t base;
      } u_Fy;
      u_Fy.real = this->Fy;
      *(outbuffer + offset + 0) = (u_Fy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Fy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Fy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Fy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Fy);
      union {
        float real;
        uint32_t base;
      } u_Tphi;
      u_Tphi.real = this->Tphi;
      *(outbuffer + offset + 0) = (u_Tphi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Tphi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Tphi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Tphi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Tphi);
      union {
        float real;
        uint32_t base;
      } u_Ttheta;
      u_Ttheta.real = this->Ttheta;
      *(outbuffer + offset + 0) = (u_Ttheta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Ttheta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Ttheta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Ttheta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Ttheta);
      union {
        float real;
        uint32_t base;
      } u_Tpsi;
      u_Tpsi.real = this->Tpsi;
      *(outbuffer + offset + 0) = (u_Tpsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Tpsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Tpsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Tpsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Tpsi);
      union {
        int16_t real;
        uint16_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_state.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_phi;
      u_phi.base = 0;
      u_phi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_phi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_phi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_phi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->phi = u_phi.real;
      offset += sizeof(this->phi);
      union {
        float real;
        uint32_t base;
      } u_theta;
      u_theta.base = 0;
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->theta = u_theta.real;
      offset += sizeof(this->theta);
      union {
        float real;
        uint32_t base;
      } u_psi;
      u_psi.base = 0;
      u_psi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_psi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_psi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_psi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->psi = u_psi.real;
      offset += sizeof(this->psi);
      union {
        float real;
        uint32_t base;
      } u_Fx;
      u_Fx.base = 0;
      u_Fx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Fx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Fx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Fx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Fx = u_Fx.real;
      offset += sizeof(this->Fx);
      union {
        float real;
        uint32_t base;
      } u_Fy;
      u_Fy.base = 0;
      u_Fy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Fy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Fy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Fy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Fy = u_Fy.real;
      offset += sizeof(this->Fy);
      union {
        float real;
        uint32_t base;
      } u_Tphi;
      u_Tphi.base = 0;
      u_Tphi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Tphi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Tphi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Tphi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Tphi = u_Tphi.real;
      offset += sizeof(this->Tphi);
      union {
        float real;
        uint32_t base;
      } u_Ttheta;
      u_Ttheta.base = 0;
      u_Ttheta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Ttheta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Ttheta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Ttheta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Ttheta = u_Ttheta.real;
      offset += sizeof(this->Ttheta);
      union {
        float real;
        uint32_t base;
      } u_Tpsi;
      u_Tpsi.base = 0;
      u_Tpsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Tpsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Tpsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Tpsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Tpsi = u_Tpsi.real;
      offset += sizeof(this->Tpsi);
      union {
        int16_t real;
        uint16_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_state.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->state = u_state.real;
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return "custom_msgs/FootOutputMsg"; };
    const char * getMD5(){ return "8f7a23844c96aae47b26e29ed7083664"; };

  };

}
#endif
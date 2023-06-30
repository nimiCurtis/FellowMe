#ifndef _ROS_SERVICE_PwmVal_h
#define _ROS_SERVICE_PwmVal_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace fellowme_msgs
{

static const char PWMVAL[] = "fellowme_msgs/PwmVal";

  class PwmValRequest : public ros::Msg
  {
    public:
      typedef int16_t _pwm_right_type;
      _pwm_right_type pwm_right;
      typedef int16_t _pwm_left_type;
      _pwm_left_type pwm_left;

    PwmValRequest():
      pwm_right(0),
      pwm_left(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_pwm_right;
      u_pwm_right.real = this->pwm_right;
      *(outbuffer + offset + 0) = (u_pwm_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pwm_right.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->pwm_right);
      union {
        int16_t real;
        uint16_t base;
      } u_pwm_left;
      u_pwm_left.real = this->pwm_left;
      *(outbuffer + offset + 0) = (u_pwm_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pwm_left.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->pwm_left);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_pwm_right;
      u_pwm_right.base = 0;
      u_pwm_right.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pwm_right.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pwm_right = u_pwm_right.real;
      offset += sizeof(this->pwm_right);
      union {
        int16_t real;
        uint16_t base;
      } u_pwm_left;
      u_pwm_left.base = 0;
      u_pwm_left.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pwm_left.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pwm_left = u_pwm_left.real;
      offset += sizeof(this->pwm_left);
     return offset;
    }

    virtual const char * getType() override { return PWMVAL; };
    virtual const char * getMD5() override { return "d32b2953d98a6839238ff7dd052824e1"; };

  };

  class PwmValResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    PwmValResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    virtual const char * getType() override { return PWMVAL; };
    virtual const char * getMD5() override { return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class PwmVal {
    public:
    typedef PwmValRequest Request;
    typedef PwmValResponse Response;
  };

}
#endif

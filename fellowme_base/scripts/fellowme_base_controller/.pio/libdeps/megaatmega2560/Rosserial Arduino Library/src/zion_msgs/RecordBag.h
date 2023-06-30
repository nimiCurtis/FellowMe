#ifndef _ROS_SERVICE_RecordBag_h
#define _ROS_SERVICE_RecordBag_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace zion_msgs
{

static const char RECORDBAG[] = "zion_msgs/RecordBag";

  class RecordBagRequest : public ros::Msg
  {
    public:
      typedef int64_t _frames_num_type;
      _frames_num_type frames_num;
      typedef float _duration_type;
      _duration_type duration;

    RecordBagRequest():
      frames_num(0),
      duration(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_frames_num;
      u_frames_num.real = this->frames_num;
      *(outbuffer + offset + 0) = (u_frames_num.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_frames_num.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_frames_num.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_frames_num.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_frames_num.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_frames_num.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_frames_num.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_frames_num.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->frames_num);
      offset += serializeAvrFloat64(outbuffer + offset, this->duration);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_frames_num;
      u_frames_num.base = 0;
      u_frames_num.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_frames_num.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_frames_num.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_frames_num.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_frames_num.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_frames_num.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_frames_num.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_frames_num.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->frames_num = u_frames_num.real;
      offset += sizeof(this->frames_num);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->duration));
     return offset;
    }

    virtual const char * getType() override { return RECORDBAG; };
    virtual const char * getMD5() override { return "ba6765e0b214cebd9a54815df2312c8c"; };

  };

  class RecordBagResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _print_type;
      _print_type print;

    RecordBagResponse():
      success(0),
      print("")
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
      uint32_t length_print = strlen(this->print);
      varToArr(outbuffer + offset, length_print);
      offset += 4;
      memcpy(outbuffer + offset, this->print, length_print);
      offset += length_print;
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
      uint32_t length_print;
      arrToVar(length_print, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_print; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_print-1]=0;
      this->print = (char *)(inbuffer + offset-1);
      offset += length_print;
     return offset;
    }

    virtual const char * getType() override { return RECORDBAG; };
    virtual const char * getMD5() override { return "9612b34a88f66f27d7f77d14001fd23f"; };

  };

  class RecordBag {
    public:
    typedef RecordBagRequest Request;
    typedef RecordBagResponse Response;
  };

}
#endif

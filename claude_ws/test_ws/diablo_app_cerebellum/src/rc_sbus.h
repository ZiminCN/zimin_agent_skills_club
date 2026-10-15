#ifndef _RC_SBUS_H_
#define _RC_SBUS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "robot.h"

typedef struct {
  uint8_t start;
  uint16_t ch1 : 11;
  uint16_t ch2 : 11;
  uint16_t ch3 : 11;
  uint16_t ch4 : 11;
  uint16_t ch5 : 11;
  uint16_t ch6 : 11;
  uint16_t ch7 : 11;
  uint16_t ch8 : 11;
  uint16_t ch9 : 11;
  uint16_t ch10 : 11;
  uint16_t ch11 : 11;
  uint16_t ch12 : 11;
  uint16_t ch13 : 11;
  uint16_t ch14 : 11;
  uint16_t ch15 : 11;
  uint16_t ch16 : 11;
  uint8_t ch17 : 1;
  uint8_t ch18 : 1;
  uint8_t frame_lost : 1;
  uint8_t failsafe : 1;
  uint8_t shit : 4;
  uint8_t end;
} __attribute__((packed)) sbus_frame_t;

int rc_sbus_init(struct robot_t *robot);

bool rc_update_input(input_t *input);

#ifdef __cplusplus
}
#endif

#endif

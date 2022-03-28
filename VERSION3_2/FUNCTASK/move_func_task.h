#ifndef __MOVE_FUNC_TASK_H__
#define __MOVE_FUNC_TASK_H__

#include "movement_struct.h"
#include "struct_typedef.h"

#define right_angle 0.64f

extern void move_pid_calc(void);
extern void move_pid_init(void);
extern void coder_move_calc(uint8_t num);
extern void angle_pid_calc(float order ,float now);
extern void angle_pid_init(void);
extern void coder_postion_calc(void);
#endif

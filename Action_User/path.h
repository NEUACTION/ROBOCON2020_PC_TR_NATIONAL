#ifndef _PATH_H
#define _PATH_H

#include "MotionCard.h"
#include "stdint.h"

#define TEST_PATH_NUM_DEF 10
#define FIRST_PATH_NUM_DEF  3
#define SECOND_PATH_NUM_DEF  10
#define SECOND_PATH_NUM_BACK_DEF  10
#define THIRD_PATH_NUM_DEF  47
#define THIRD_PATH_NUM_BACK_DEF  47
#define FORTH_PATH_NUM_DEF  54
#define FORTH_PATH_NUM_BACK_DEF  53
#define FIFTH_PATH_NUM_DEF  54
#define SIX_PATH_NUM_DEF  15
#define SEVEN_PATH_NUM_DEF  4

//测试轨迹长
extern uint8_t TEST_PATH_NUM;
extern Pose_t testPath[];

extern uint8_t FIRST_PATH_NUM;
extern Pose_t firstPath[];

extern uint8_t SECOND_PATH_NUM;
extern Pose_t secondPath[];

extern uint8_t SECOND_PATH_NUM_BACK;
extern Pose_t secondPathBack[];

extern uint8_t THIRD_PATH_NUM;
extern Pose_t thirdPath[];
extern uint8_t THIRD_PATH_NUM_BACK;
extern Pose_t thirdPathBack[];

extern uint8_t FORTH_PATH_NUM;
extern Pose_t forthPath[];
extern uint8_t FORTH_PATH_NUM_BACK;
extern Pose_t forthPathBack[];

extern uint8_t FIFTH_PATH_NUM;
extern Pose_t fifthPath[];

extern uint8_t SIX_PATH_NUM;
extern Pose_t sixthPath[];

extern uint8_t SEVEN_PATH_NUM;
extern Pose_t seventhPath[];


#endif



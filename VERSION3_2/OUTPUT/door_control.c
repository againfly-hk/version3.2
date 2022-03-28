#include "door_control.h"
/*                    on   ->  off
tim8ch1 backdoor
tim8ch2 leftdoor
tim8ch3 rightdoor
tim1ch1
tim1ch2 frontdoor
tim1ch3 rocket
tim1ch4 door
*/

#define LeftDoorOn 1450
#define LeftDoorOff 1010
#define RightDoorOn 1500
#define RightDoorOff 900
#define DoorLeft 1800
#define DoorRight 1000
#define DoorMiddle 1380
#define DoorReset 1800 //有干涉，不能再向下了
#define FrontDoorLift 500
#define FrontDoorDown 1150 //因为干涉只能放这么低

#define RocketLevel 0
#define RocketVertical 0
#define RocketReserseSide 0//火箭的还没有调

#define BackDoorAway 1000
#define BackDoorDown 500

void left_door_on()
{
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,LeftDoorOn);
}

void left_door_off()
{
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,LeftDoorOff);
}

void right_door_on()
{
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,RightDoorOn);
}

void right_door_off()
{
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,RightDoorOff);
}

void door_left()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,DoorLeft);
}

void door_middle()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,DoorMiddle);
}

void door_right()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,DoorRight);
}

void door_reset()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,DoorReset);
}

void front_door_lift()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,FrontDoorLift);
}

void front_door_down()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,FrontDoorDown);
}

void rocket_level()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,RocketLevel);
}

void rocket_vertical()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,RocketVertical);
}

void rocket_reserse_side()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,RocketReserseSide);
}

void back_door_away()
{
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,BackDoorAway);
}

void back_door_down()
{
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,BackDoorDown);
}

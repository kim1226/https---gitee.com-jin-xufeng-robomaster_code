#ifndef __vision_task_H
#define __vision_task_H

typedef enum{
    VISION_ON= 0, //自瞄开
    VISION_OFF,    //
}VISION_t;
extern float vision_yaw,vision_pitch;
typedef struct{
	
	float target_angle;
	float target_speed;
}GIMBAL_VI_t;

//上位机发送的值
typedef struct{
	GIMBAL_VI_t yaw;
	GIMBAL_VI_t pitch;
}VISION_GET_t;

extern VISION_t vision_mode;
extern VISION_GET_t vision_sent;

void Vision_Task(void);
extern double vision_actual_speed,vision_actual_angle;
void vision_getSpeed(void);
#endif

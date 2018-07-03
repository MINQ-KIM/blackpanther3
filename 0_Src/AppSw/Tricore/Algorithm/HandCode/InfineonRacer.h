#ifndef INFINEONRACER_H_
#define INFINEONRACER_H_


/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include <Ifx_Types.h>
#include "Configuration.h"

/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/
#define IR_getLs0Margin()		IR_Ctrl.Ls0Margin
#define IR_getLs1Margin()		IR_Ctrl.Ls1Margin
#define IR_getGainAngleP()		Handcode.Gain_angle_p
#define IR_getGainAngleD()		Handcode.Gain_angle_d
#define IR_getGainSpeedP()		Handcode.Gain_speed_p


/******************************************************************************/
/*--------------------------------Enumerations--------------------------------*/
/******************************************************************************/



/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/
typedef struct{
	sint32 Ls0Margin;
	sint32 Ls1Margin;
	boolean basicTest;
}InfineonRacer_t;

typedef struct{
	float32 Gain_angle_p;
	float32 Gain_angle_d;
	float32 Gain_speed_p;
}Handcode_t;

/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/
IFX_EXTERN InfineonRacer_t IR_Ctrl;
IFX_EXTERN Handcode_t Handcode;

/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/
IFX_EXTERN void InfineonRacer_init(void);
IFX_EXTERN void InfineonRacer_detectLane(void);
IFX_EXTERN void InfineonRacer_control(void);
IFX_EXTERN void InfineonRacer_Avoid(sint32 task_cnt);
IFX_EXTERN void InfinedonRacer_AEB(void);

#endif

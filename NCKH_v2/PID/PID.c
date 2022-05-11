#include "PID.h"

float duty;

void Constant(float *variable,float max,float min){
		if ( *variable > max ) {
			*variable = max;
		}
		else if ( *variable< min ){
			*variable = min ;
		} 	
}

float PID(float SP,float Value, float Kp,float Ki, float Kd,float *error,float *last_error,float *pTerm,float *iTerm, float *dTerm)
{
	*error= SP - Value;

	*pTerm = Kp * (*error);
	*iTerm = Ki * (*error + *last_error);
	*dTerm = Kd * (*error - *last_error);
	*last_error = *error;
	duty =  *pTerm + *iTerm + *dTerm;	
//	if ( duty > 999 ) {
//			duty = 999;}
//	else if ( duty< -999 ){
//			duty= -999 ;
//	} 		
	Constant(&duty,999,-999);
	
	return duty;
}	

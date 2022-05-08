#ifndef LFR_REGULATOR_H
#define LFR_REGULATOR_H

//constants
#define ERROR_THRESHOLD			5.0f	//[cm] because of the noise of the camera
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define KP						1.0f
#define KI 						0.25f	//must not be zero
#define ROTATION_THRESHOLD		10      //to optimise
#define ROTATION_COEFF			2       //to optimise
#define DEFAULT_SPEED           400       //to define

//start Line Follower Robot (LFR) PI regulator thread
void lfr_regulator_start(void);

#endif /* LFR_REGULATOR_H */

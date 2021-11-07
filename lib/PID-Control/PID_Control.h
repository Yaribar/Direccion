// PID_Control.h

#ifndef _PID_CONTROL_h
#define _PID_CONTROL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


class PIDCONTROL {
public:
	PIDCONTROL(float kp,float kd, float ki, float interval_i,int bits_resolution);
	float Data(float error,float dt);
	void changeConstants(float kp,float kd, float ki);
private:
	float _kp = 0, _kd = 0, _ki = 0;
	float _pid=0, _pid_p=0, _pid_d=0, _pid_i=0;
	float _interval_i, _previous_error = 0, _elapsed_time;
	int16_t _resolution=0;
	float  _time=0, _timePrev=0;
};

#endif


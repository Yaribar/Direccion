// 
// 
// 

#include "PID_Control.h"

PIDCONTROL::PIDCONTROL(float kp, float kd, float ki, float interval_i,int bits_resolution)
{
	_kp = kp;
	_kd = kd;
	_ki = ki;
	_interval_i = interval_i;
	_resolution =( pow(2, bits_resolution) - 1);
}

void PIDCONTROL::changeConstants(float kp,float kd, float ki){
	_kp = kp;
	_kd = kd;
	_ki = ki;
}

float PIDCONTROL::Data(float error,float dt)
{
	_pid_p = _kp*error;
	_elapsed_time = dt;
	if ((-1*(_interval_i)) <error <_interval_i)
	{
	_pid_i += (_ki*error);
	}
	_pid_d = _kd*((error - _previous_error) / _elapsed_time);
	_pid = _pid_p + _pid_i + _pid_d;
	_previous_error = error;
	if (_pid > _resolution)
		_pid = _resolution;
	if (_pid < (-1 * _resolution))
		_pid = -1 * _resolution;

	return _pid;
}

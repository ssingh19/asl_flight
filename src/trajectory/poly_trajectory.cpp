#include <trajectory/trajectory.h>
#include <math.h>
#include <ros/ros.h>


PolyTrajectory::PolyTrajectory(double _start_delay, double _scale):
	switch_time(5),
	start_delay(_start_delay),
	scale(_scale)
{

  // encode to cancel initial position in coeffs
	start_pos << 1.0, 1.0, 0.0;

	Eigen::MatrixXd coeffs(50,3);
	coeffs <<
		-1,-1,0,
		-1.2448e-13,-6.204e-13,-1.6736e-13,
		0,0,0,
		9.1096e-14,4.4558e-13,1.2113e-13,
		0.46811,2.5164,0.54136,
		-0.4339,-2.4284,-0.46762,
		0.10603,0.61015,0.092081,
		0.027157,0.1371,0.039092,
		-0.017662,-0.092713,-0.021229,
		0.0024815,0.013046,0.0029755,
		-0.60006,0.74994,0.48894,
		0.48864,1.512,0.56022,
		-0.056593,-1.3986,-0.20788,
		-0.11144,-0.94311,-0.30412,
		0.12071,0.75416,0.093916,
		1.6407,-1.0082,0.75902,
		-5.9398,3.2973,-2.8205,
		8.2924,-4.0456,4.1887,
		-5.2845,2.1687,-2.7916,
		1.2847,-0.44123,0.70023,
		-0.2,0.75,0.68408,
		0.41327,-1.2061,-0.17972,
		0.050998,-0.63296,-0.32985,
		0.033462,0.78479,0.1962,
		-0.082768,-0.16635,0.074627,
		0.5564,-1.8683,-0.49192,
		-1.5767,4.8449,1.2432,
		1.7246,-5.0611,-1.3465,
		-0.84025,2.4415,0.65826,
		0.15481,-0.45087,-0.12176,
		0.29996,-0.74995,0.35155,
		0.28589,-0.75718,-0.13113,
		-0.13368,0.67653,0.15575,
		0.067629,0.3378,-0.10362,
		0.098905,0.12803,-0.10369,
		2.6334,0.30038,-0.78249,
		-11.53,0.51099,3.4636,
		19.308,-3.4061,-5.6754,
		-14.735,3.9067,4.2833,
		4.2916,-1.4095,-1.2406,
		0.5,-0.75001,0.27102,
		0.3063,0.93667,-0.17291,
		0.15412,1.2771,-0.19133,
		0.051668,-0.17104,-0.061605,
		-0.11518,-0.45102,0.12215,
		0.16243,0.78524,-0.13991,
		-0.41977,-1.8779,0.35774,
		0.40594,1.7843,-0.3482,
		-0.16785,-0.73539,0.14398,
		0.025927,0.11361,-0.022196;

	// switch time
	switch_time << 1.5216,0.9165,1.2091,0.7624,1.4437;
	for (int i=1; i<switch_time.size(); i++)
	{
		switch_time(i) = switch_time(i-1) + switch_time(i);
	}

	for (int i=0; i<switch_time.size(); i++)
	{
		Eigen::MatrixXd segment = coeffs.block(i*10,0,10,3);
		cp.push_back(expand_coeffs(segment));
	}

}

Eigen::MatrixXd PolyTrajectory::expand_coeffs(Eigen::MatrixXd& _coeffs)
{

	Eigen::MatrixXd seg_coeff(12,10);
	seg_coeff.setZero();

	for (int i=0; i<3; i++)
	{
		// position
		for (int k=0; k<10; k++)
		{
			seg_coeff(i,k) = _coeffs(k,i);
		}
		// velocity
		for (int k=0; k<9; k++)
		{
			seg_coeff(i+3,k) = (k+1)*_coeffs(k+1,i);
		}
		// acceleration
		for (int k=0; k<8; k++)
		{
			seg_coeff(i+6,k) = (k+1)*(k+2)*_coeffs(k+2,i);
		}
		// jerk
		for (int k=0; k<7; k++)
		{
			seg_coeff(i+9,k) = (k+1)*(k+2)*(k+3)*_coeffs(k+3,i);
		}
	}

	return seg_coeff;
}

void PolyTrajectory::eval(double t, Eigen::Vector3d &pos,
                                    Eigen::Vector3d &vel,
                                    Eigen::Vector3d &acc,
                                    Eigen::Vector3d &jer)
{

	double traj_valid = 1.0;

	double _t = 0.0;

  // Apply delay
	if (t >= start_delay)	{
		_t = t - start_delay;
	} else {
    _t = 0.0;
  }

  // Scale for slow-down
	_t /= scale;

  // Check for termination
	if (_t > switch_time(switch_time.size()-1))
	{
		_t = switch_time(switch_time.size()-1);
		traj_valid = 0.0;
	}

  // Find segment
	int ti = 0;
	for (int i = 0; i < switch_time.size(); i++)
	{
		ti = i;
		if (_t <= switch_time(i))
		{
			break;
		}
	}

  // Reference start time for segment
	double t0 = 0;
	if (ti > 0)
	{
		t0 = switch_time(ti - 1);
	}

  // Get coeffs
	Eigen::MatrixXd coeffs = cp[ti];

  // Compute
	Eigen::VectorXd output(coeffs.rows());

	for (int i = 0; i < coeffs.rows(); i++)
	{
		output(i) = coeffs(i,0);
		for (int j = 1; j < coeffs.cols(); j++)
		{
			output(i) += coeffs(i,j) * pow(_t-t0,j);
		}
	}

	pos(0) = start_pos(0) + output(0);
	pos(1) = start_pos(1) + output(1);
	pos(2) = start_pos(2) + output(2);

	vel(0) = output(3) * traj_valid/scale;
	vel(1) = output(4) * traj_valid/scale;
	vel(2) = output(5) * traj_valid/scale;

	acc(0) = output(6) * traj_valid/(scale*scale);
	acc(1) = output(7) * traj_valid/(scale*scale);
	acc(2) = output(8) * traj_valid/(scale*scale);

	jer(0) = output(9) * traj_valid/(scale*scale*scale);
	jer(1) = output(10) * traj_valid/(scale*scale*scale);
	jer(2) = output(11) * traj_valid/(scale*scale*scale);

}

void PolyTrajectory::set_start_pos(const Eigen::Vector3d& _start_pos)
{
  // encode initial cancellation of coeffs (to zero out) + input adjustment
  start_pos += _start_pos;
}

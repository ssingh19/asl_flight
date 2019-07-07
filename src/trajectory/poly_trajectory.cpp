#include <trajectory/trajectory.h>
#include <math.h>
#include <ros/ros.h>
#include <trajectory/poly_coeffs.h>


PolyTrajectory::PolyTrajectory(double _scale, double _start_delay):
	switch_time(14),
	output(12),
	start_delay(_start_delay),
	scale(_scale)
{

	output.setZero();
	output_yaw.setZero();

  // encode to cancel initial position in coeffs
	start_pos.setZero();

	// switch time
	switch_time <<  3.3701,
    7.0708,
    8.4846,
   11.3556,
   16.1719,
   18.7593,
   21.2462,
   23.6438,
   24.8812,
   26.0630,
   27.2448,
   28.4823,
   30.5485,
   35.9912;

	// for (int i=1; i<switch_time.size(); i++)
	// {
	// 	switch_time(i) = switch_time(i-1) + switch_time(i);
	// }

	for (int i=0; i<switch_time.size(); i++)
	{
		Eigen::MatrixXd segment = coeffs.block(i*10,0,10,3);
		cp.push_back(expand_coeffs(segment));

		Eigen::MatrixXd segment_yaw = coeffs_yaw.block(i*3,0,3,1);
		cp_yaw.push_back(expand_coeffs_yaw(segment_yaw));
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

Eigen::MatrixXd PolyTrajectory::expand_coeffs_yaw(Eigen::MatrixXd& _coeffs)
{

	Eigen::MatrixXd seg_coeff(2,3);
	seg_coeff.setZero();

	// yaw
	for (int k=0; k<3; k++){
		seg_coeff(0,k) = _coeffs(k,0);
	}

	// yaw_dot
	for (int k=0; k<2; k++){
		seg_coeff(1,k) = (k+1)*_coeffs(k+1,0);
	}

	return seg_coeff;
}

void PolyTrajectory::eval(double t, Eigen::Vector3d &pos,
                                    Eigen::Vector3d &vel,
                                    Eigen::Vector3d &acc,
                                    Eigen::Vector3d &jer,
																	  double &yaw,
																	  double &yaw_dot)
{

	double traj_valid = 1.0;

	double _t = 0.0;

  // Apply delay
	if (t >= start_delay)	{
		_t = t - start_delay;
	} else {
    _t = 0.0; // use to reference initial pose
		traj_valid = 0.0; // use to switch off dynamic commands
  }

  // Scale for slow-down
	_t /= scale;

  // Check for termination
	if (_t > switch_time(switch_time.size()-1))
	{
		_t = switch_time(switch_time.size()-1);
		traj_valid = 0.0; // switch off dynamic commands
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

  // Get coeffs for segment
	Eigen::MatrixXd coeffs = cp[ti];
	Eigen::MatrixXd coeffs_yaw = cp_yaw[ti];

  // Compute
	for (int i = 0; i < coeffs.rows(); i++)
	{
		output(i) = coeffs(i,0);
		for (int j = 1; j < coeffs.cols(); j++)
		{
			output(i) += coeffs(i,j) * pow(_t-t0,j);
		}
	}

	for (int i = 0; i< coeffs_yaw.rows(); i++) {
		output_yaw(i) = coeffs_yaw(i,0);
		for (int j=1; j< coeffs_yaw.cols(); j++) {
			output_yaw(i) += coeffs_yaw(i,j) * pow(_t-t0,j);
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

	yaw = output_yaw(0);
	yaw_dot = output_yaw(1) * traj_valid/scale;

}

void PolyTrajectory::set_start_pos(const Eigen::Vector3d& _start_pos)
{
  // encode initial cancellation of coeffs (to zero out) + input adjustment
  start_pos += _start_pos;
}

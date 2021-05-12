 /**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "DroneController.h"
#include "gvars3/instances.h"
#include "../HelperFunctions.h"
#include "ControlNode.h"
#include <numeric>

DroneController::DroneController(void)
{
	
	target = DronePosition(TooN::makeVector(0.0,0.0,0.0),0.0);
	targetValid = false;
	last_err[2] = 0;
	lastTimeStamp = 0;

	// LQR variables
	droll = 0;
	dpitch = 0;
	roll_before = 0;
	pitch_before = 0;
	new_int_err[0] = 0;
	new_int_err[1] = 0;
	new_int_err[2] = 0;
	new_int_err[3] = 0;


	hoverCommand.gaz = hoverCommand.pitch = hoverCommand.roll = hoverCommand.yaw = 0;

	node = NULL;
}


DroneController::~DroneController(void)
{
}

double angleFromTo2(double angle, double min, double sup)
{
	while(angle < min) angle += 360;
	while(angle >=  sup) angle -= 360;
	return angle;
}


// LQR update
ControlCommand DroneController::update(tum_ardrone::filter_stateConstPtr state)
{
	// pose
	TooN::Vector<3> position = TooN::makeVector(state->x, state->y, state->z);
	TooN::Vector<3> pose = TooN::makeVector((state->roll)*3.141592/180,
	                                        (state->pitch)*3.141592/180,
											(state->yaw)*3.141592/180); // angles in rad

	// velocity
	TooN::Vector<3> speed_position = TooN::makeVector(state->dx, state->dy, state->dz);

	// estimate Derivative using function
	// df(x)/dx~=(f(x)-f(x-1))/e
	double e = getMS()/1000.0 - lastTimeStamp; 
	droll = (pose[0] - roll_before)/e;
	dpitch = (pose[1] - pitch_before)/e;

    roll_before = (state->roll)*3.141592/180;
	pitch_before = (state->pitch)*3.141592/180;

    // moving average to derivative term
	if(roll_queue.size() < 8 || pitch_queue.size() < 8)
	{
	  roll_queue.push_back(droll);
	  pitch_queue.push_back(dpitch);

	  return lastSentControl;	
	}
	roll_queue.erase(roll_queue.begin());
	roll_queue.push_back(droll);
	pitch_queue.erase(pitch_queue.begin());
	pitch_queue.push_back(dpitch);
  
	double roll_average = 0;
	double pitch_average = 0;
	for(int i=0;i<roll_queue.size();i++)
	{
		roll_average += roll_queue[i];
		pitch_average += pitch_queue[i];
	}
	droll = roll_average/roll_queue.size();
	dpitch = pitch_average/pitch_queue.size();

	//make vector
	TooN::Vector<3> speed_pose = TooN::makeVector(droll, dpitch, (state->dyaw)*3.141592/180); // angles in rad

	// status
	ptamIsGood = state->ptamState == state->PTAM_BEST || state->ptamState == state->PTAM_GOOD || state->ptamState == state->PTAM_TOOKKF;
	scaleAccuracy = state->scaleAccuracy;

	// calculate (new) errors.
	TooN::Vector<4> new_err = TooN::makeVector(
		target.pos[0] - position[0],
		target.pos[1] - position[1],
 		target.pos[2] - position[2],
 		(target.yaw)*3.141592/180 - pose[2]
		);

	// yaw needs special attention, it can always be pushed in between 180 and -180.
	// this does not affect speeds and makes the drone always take the quickest rotation side.
	new_err[3] = angleFromTo2(new_err[3],-3.141592,3.141592);

    // Integrator error
	new_int_err[0]+= new_err[0]*e;
	new_int_err[1]+= new_err[1]*e;
	new_int_err[2]+= new_err[2]*e;
	new_int_err[3]+= new_err[3]*e;

	// Update timestamp
	lastTimeStamp = getMS()/1000.0;

	// calculate state vector 2
	TooN::Vector<> states(16);
	states = TooN::makeVector(
		position[0],
		speed_position[0],
		position[1],
		speed_position[1],
		pose[1],
		speed_pose[1],
		pose[0],
		speed_pose[0],
		position[2],
		speed_position[2],
		pose[2],
		speed_pose[2],
		new_int_err[0],
		new_int_err[1],
		new_int_err[2],
		new_int_err[3]
	   );

	if(targetValid)
		calcControl(states);
	else
	{
		lastSentControl = hoverCommand;
		ROS_WARN("Warning: no valid target, sending hover.");
	}

    last_err = new_err;
	return lastSentControl;
}

void DroneController::setTarget(DronePosition newTarget)
{
	target = newTarget;
	target.yaw = angleFromTo2(target.yaw,-180,180);
	targetSetAtClock = getMS()/1000.0;
	targetNew = TooN::makeVector(1.0,1.0,1.0,1.0);
	targetValid = true;
	last_err = i_term = TooN::makeVector(0,0,0,0);

	char buf[200];
	snprintf(buf,200,"New Target: xyz = %.3f, %.3f, %.3f,  yaw=%.3f", target.pos[0],target.pos[1],target.pos[2],target.yaw);
	ROS_INFO(buf);

	if(node != NULL)
		node->publishCommand(std::string("u l ") + buf);
}

DronePosition DroneController::getCurrentTarget()
{
	return target;
}

void DroneController::clearTarget()
{
	targetValid = false;
}

//LQR calcControl
// The feedback gains have been found by simulations on matlab and real tests.
void DroneController::calcControl(TooN::Vector<16> states)
{
	// pitch
    TooN::Vector<> vector_k11(16);
	vector_k11 = TooN::makeVector(0,0,-3.12,-1.59,3.06,0.47,0,0,0,0,0,0,0,2.86,0,0);

	// roll
	TooN::Vector<> vector_k21(16);
	vector_k21 = TooN::makeVector(3.63,1.64,0,0,0,0,3.43,0.49,0,0,0,0,-3.44,0,0,0);

	// Z
	TooN::Vector<> vector_k31(16);
	vector_k31 = TooN::makeVector(0,0,0,0,0,0,0,0,-5.15,-1.6,0,0,0,0,4.47,0);

    // yaw
    TooN::Vector<> vector_k41(16);
    vector_k41 = TooN::makeVector(0,0,0,0,0,0,0,0,0,0,1.19,0.08,0,0,0,-1.00);

	// YAW
	// windup
	if(vector_k41*states > 1 || vector_k41*states < -1)
	  new_int_err[3] = 0.9 * new_int_err[3]; 
	lastSentControl.yaw = vector_k41*states;
	lastSentControl.yaw = std::min(max_yaw,std::max(-max_yaw,
	                                               (double)(lastSentControl.yaw)));

    // ROLL and PITCH
	double pitch_cmd = vector_k11*states;
	double roll_cmd = vector_k21*states;
    // windup
	if(roll_cmd > 1 || roll_cmd < -1)
	  new_int_err[1] = 0.9 * new_int_err[1];
	lastSentControl.roll = roll_cmd;
	lastSentControl.roll = std::min(max_rp,std::max(-max_rp,
	                                               (double)(lastSentControl.roll)));
    // windup
	if(pitch_cmd > 1 || pitch_cmd < -1)
	  new_int_err[0] = 0.9 * new_int_err[0];
	lastSentControl.pitch = pitch_cmd;
	lastSentControl.pitch = std::min(max_rp,std::max(-max_rp,
	                                                (double)(lastSentControl.pitch)));

	// GAZ
	// windup
	if(vector_k31*states > 1 || vector_k31*states < -1)
	  new_int_err[2] = 0.9 * new_int_err[2];
	lastSentControl.gaz = vector_k31*states;
	lastSentControl.gaz = std::min(max_gaz_rise,std::max(max_gaz_drop,
	                                                    (double)lastSentControl.gaz));
	if(lastSentControl.gaz > 0) lastSentControl.gaz *= rise_fac;
}

TooN::Vector<4> DroneController::getLastErr()
{
	return last_err;
}
ControlCommand DroneController::getLastControl()
{
	return lastSentControl;
}

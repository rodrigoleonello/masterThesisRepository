#pragma once
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
#ifndef __DRONECONTROLLER_H
#define __DRONECONTROLLER_H
 
 
#include "ros/ros.h"
#include "TooN/se3.h"
#include <queue>
#include "geometry_msgs/Twist.h"
#include "tum_ardrone/filter_state.h"
#include <gram_savitzky_golay/gram_savitzky_golay.h>

class ControlNode;

struct ControlCommand
{
	inline ControlCommand() {roll = pitch = yaw = gaz = 0;}
	inline ControlCommand(double roll, double pitch, double yaw, double gaz)
	{
		this->roll = roll;
		this->pitch = pitch;
		this->yaw = yaw;
		this->gaz = gaz;
	}
	double yaw, roll, pitch, gaz;
};


struct DronePosition
{
public:
	double yaw;
	TooN::Vector<3> pos;
	inline DronePosition(TooN::Vector<3> pos, double yaw)
		: yaw(yaw), pos(pos) {}
	inline DronePosition(){ yaw=0; pos=TooN::makeVector(0,0,0);}
};

class DroneController
{
private:
	ControlCommand lastSentControl;

    bool aux;
	bool aux1;
	bool aux2;
	bool aux3;
	bool aux4;
	bool aux5;
	double after;
	double tempo;
	double tempo1;

	double droll;
	double dpitch;
	double droll_before;
	double dpitch_before;
	double roll_before;
	double pitch_before;

	std::vector<double> roll_queue;
	std::vector<double> pitch_queue;

    //test quadratic signal
	double aux_yaw;
	double aux_roll;
	double aux_pitch;
	double aux_gaz;
	bool chavear1;
	bool chavear2;
	bool chavear3;
	bool chavear4;
	int timer1;
	int timer2;
	int timer3;
	int timer4;

    ros::NodeHandle nh_;
    ros::Publisher setpoint_pub;
	std::string setpoint_channel;
	ros::Publisher derivative_pub;
	std::string derivative_channel;

	
	TooN::Vector<4> new_int_err;

	// currentTarget.
	DronePosition target;
	bool targetValid;

	// used for integral term
	TooN::Vector<4> targetNew;	// 0=target has been reached before
								// 1=target is new

	// need to keep track of integral terms
	TooN::Vector<4> i_term;
	TooN::Vector<4> last_err;
	TooN::Vector<4> speedAverages;

	double lastTimeStamp;
	double targetSetAtClock;
	ControlCommand hoverCommand;

	// filled with info (on update)
	bool  ptamIsGood;
	double scaleAccuracy;
	// void calcControl(TooN::Vector<4> new_err, TooN::Vector<4> d_error, double yaw);
	// void calcControl(double yaw, TooN::Vector<12> states);
	void calcControl(double yaw, TooN::Vector<16> states, TooN::Vector<4> new_err);

public:

	// generates and sends a new control command to the drone, based on the currently active command ant the drone's position.
	ControlCommand update(tum_ardrone::filter_stateConstPtr);

	ControlNode* node;

	// for logging, gets filled with recent infos on control.
	TooN::Vector<28> logInfo;

	// adds a waypoint
	void setTarget(DronePosition newTarget);
	void clearTarget();
	DronePosition getCurrentTarget();
	ControlCommand getLastControl();

	// gets last error
	TooN::Vector<4> getLastErr();

	DroneController(void);
	~DroneController(void);





	// PID control parameters. settable via dynamic_reconfigure
	// target is where i want to get to.
	// pose and yaw are where i am.
	double max_yaw;
	double max_rp;
	double max_gaz_rise;
	double max_gaz_drop;

	double rise_fac;
	double agressiveness;

	double Ki_yaw;
	double Kd_yaw;
	double Kp_yaw;

	double Ki_gaz;
	double Kd_gaz;
	double Kp_gaz;

	double Ki_rp;
	double Kd_rp;
	double Kp_rp;

};
#endif /* __DRONECONTROLLER_H */


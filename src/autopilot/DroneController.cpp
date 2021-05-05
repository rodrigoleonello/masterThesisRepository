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
	// publisher
	setpoint_channel = nh_.resolveName("setpoint");
	setpoint_pub = nh_.advertise<geometry_msgs::Twist>(setpoint_channel,100);

    derivative_channel = nh_.resolveName("derivative");
	derivative_pub = nh_.advertise<geometry_msgs::Twist>(derivative_channel,100);
	
	target = DronePosition(TooN::makeVector(0.0,0.0,0.0),0.0);
	targetValid = false;
	last_err[2] = 0;
	lastTimeStamp = 0;

    aux = false;
	aux1 = false;
	aux2 = false;
	aux3 = false;
	aux4 = false;
	aux5 = false;
	tempo = 0;
	tempo1 = 0;
	after = 0;

	// LQR variables
	droll = 0;
	dpitch = 0;
	droll_before = 0;
	dpitch_before = 0;
	roll_before = 0;
	pitch_before = 0;
	
	new_int_err[0] = 0;
	new_int_err[1] = 0;
	new_int_err[2] = 0;
	new_int_err[3] = 0;

	// Test variables
	aux_yaw = -0.5;
	aux_roll = 0.5;
	aux_pitch = -0.5;
	aux_gaz = 0.2;
	chavear1 = false;
	chavear2 = false;
	chavear3 = false;
	chavear4 = false;
	timer1 = 0;
	timer2 = 0;
	timer3 = 0;
	timer4 = 0;

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

// PID update
// ControlCommand DroneController::update(tum_ardrone::filter_stateConstPtr state)
// {
// 	TooN::Vector<3> pose = TooN::makeVector(state->x, state->y, state->z);
// 	double yaw = state->yaw;
// 	// std::cout<<"state_atual_yaw"<<state->yaw<<std::endl;
// 	TooN::Vector<4> speeds = TooN::makeVector(state->dx, state->dy, state->dz, state->dyaw);
// 	ptamIsGood = state->ptamState == state->PTAM_BEST || state->ptamState == state->PTAM_GOOD || state->ptamState == state->PTAM_TOOKKF;
// 	scaleAccuracy = state->scaleAccuracy;
// 	// std::cout<<"state_derivada_yaw"<<state->dyaw<<std::endl;

    
// 	// Teste do 8
// 	// double time = getMS()/1000.0;
//     // target.pos[0] = 0.5*sin(0.8*time);
// 	// target.pos[1] = sin(0.4*time);
// 	// target.pos[2] = 0.5 + 0.5*sin(0.4*time);
// 	// target.yaw = (-3.14/6)*sin(0.4*time)*180/3.141592;

// 	// Teste do circulo
// 	double time = getMS()/1000.0;
//     target.pos[0] = 0.8*sin(0.5*time);
// 	target.pos[1] = 0.8*cos(0.5*time);
// 	target.pos[2] = 0.7;
// 	target.yaw = 0;


// 	// calculate (new) errors.
// 	TooN::Vector<4> new_err = TooN::makeVector(
// 		target.pos[0] - pose[0],
// 		target.pos[1] - pose[1],
// 		target.pos[2] - pose[2],
// 		target.yaw - yaw
// 		);

// 	// yaw error needs special attention, it can always be pushed in between 180 and -180.
// 	// this does not affect speeds and makes the drone always take the quickest rotation side.
// 	new_err[3] = angleFromTo2(new_err[3],-180,180);	
// 	// std::cout<<"yaw_erro "<<new_err[3]<<std::endl;
// 	TooN::Vector<4> d_err = TooN::makeVector(-speeds[0], -speeds[1], -speeds[2], -speeds[3]);

// 	if(targetValid)
// 		calcControl(new_err, d_err, yaw);
// 	else
// 	{
// 		lastSentControl = hoverCommand;
// 		ROS_WARN("Warning: no valid target, sending hover.");
// 	}

// 	last_err = new_err;
// 	return lastSentControl;
// }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// LQR update
ControlCommand DroneController::update(tum_ardrone::filter_stateConstPtr state)
{
	// pose
	TooN::Vector<3> position = TooN::makeVector(state->x, state->y, state->z);
	TooN::Vector<3> pose = TooN::makeVector((state->roll)*3.141592/180, (state->pitch)*3.141592/180, (state->yaw)*3.141592/180);
    double yaw = state->yaw;

	// velocity
	TooN::Vector<3> speed_position = TooN::makeVector(state->dx, state->dy, state->dz);

	// estimate Derivative using function
	// df(x)/dx~=(f(x)-f(x-1))/e
	double e = getMS()/1000.0 - lastTimeStamp; 
	// double e = 0.032;
	// droll = (droll_before + (pose[0] - roll_before)/e)/2;
	// dpitch = (dpitch_before + (pose[1] - pitch_before)/e)/2;
	droll = (pose[0] - roll_before)/e;
	dpitch = (pose[1] - pitch_before)/e;

    roll_before = (state->roll)*3.141592/180;
	pitch_before = (state->pitch)*3.141592/180;

    if(roll_queue.size() < 10 || pitch_queue.size() < 10)
	{
	  roll_queue.push_back(droll);
	  pitch_queue.push_back(dpitch);

	  return lastSentControl;	
	}

	roll_queue.erase(roll_queue.begin());
	roll_queue.push_back(droll);

	pitch_queue.erase(pitch_queue.begin());
	pitch_queue.push_back(dpitch);
  
    // média movel
	double roll_average = 0;
	double pitch_average = 0;
	for(int i=0;i<roll_queue.size();i++)
	{
		roll_average += roll_queue[i];
		pitch_average += pitch_queue[i];
	}
	droll = roll_average/roll_queue.size();
	dpitch = pitch_average/pitch_queue.size();

	//Filtro golay
	// // Window size is 2*m+1
    // const size_t m = 3;
    // // Polynomial Order
    // const size_t n = 2;
    // // Initial Point Smoothing (ie evaluate polynomial at first point in the window)
    // // Points are defined in range [-m;m]
    // const size_t t = m;
    // // Derivation order? 0: no derivation, 1: first derivative, 2: second derivative...
    // const int d = 0;
	// gram_sg::SavitzkyGolayFilter first_derivative_filter1(m, t, n, d);
    // // Should be =.1
    // droll = first_derivative_filter1.filter(roll_queue);
	// gram_sg::SavitzkyGolayFilter first_derivative_filter2(m, t, n, d);
    // // Should be =.1
    // dpitch = first_derivative_filter2.filter(pitch_queue);

	//make vector
	TooN::Vector<3> speed_pose = TooN::makeVector(droll, dpitch, (state->dyaw)*3.141592/180);

    // droll_before = droll;
	// dpitch_before = dpitch;

	// status
	ptamIsGood = state->ptamState == state->PTAM_BEST || state->ptamState == state->PTAM_GOOD || state->ptamState == state->PTAM_TOOKKF;
	scaleAccuracy = state->scaleAccuracy;

	// Teste do 8
	if(aux)
    {
	  double time = getMS()/1000.0;
       target.pos[0] = 0.5*sin(0.8*time);
	//  target.pos[0] = 0;
	   target.pos[1] = sin(0.4*time);
	//    target.pos[1] = 0;
	   target.pos[2] = 0.7 + 0.5*sin(0.4*time);
	//  target.pos[2] = 0.7;
	  target.yaw = (-3.14/12)*sin(0.4*time)*180/3.141592;
	//    target.yaw = 0;
	}

	// Teste do circulo
	// if(aux)
    // {
	//   double time = getMS()/1000.0;
    //   target.pos[0] = 0.8*sin(0.8*time);
	//   target.pos[1] = 0.8*cos(0.8*time);
	//   target.pos[2] = 0.7;
	//   target.yaw = 0;
	// }

	// Teste sen() em Z
	// if(aux)
    // {
	//   double time = getMS()/1000.0;
    //   target.pos[0] = 0;
	//   target.pos[1] = 0;
	//   target.pos[2] = 0.5*cos(0.8*time) + 0.7;
	//   target.yaw = 0;
	// }

	// Teste sen() em yaw
	// if(aux)
    // {
	//   double time = getMS()/1000.0;
    //   target.pos[0] = 0;
	//   target.pos[1] = 0;
	//   target.pos[2] = 0.7;
	//   target.yaw = (-3.14/12)*sin(0.4*time)*180/3.141592;;
	// }

    if(tempo>300)
	 aux = true;
    
	tempo++;

    // Teste do Quadrado1
    // if(aux1)
    // {
    //    target.pos[0] = 1;
	//    target.pos[1] = 0;
	//    aux2 = false;
	//    aux4 = false;
	// }
	// if(aux2)
    // {
    //    target.pos[0] = 1;
	//    target.pos[1] = 1;
	//    aux1 = false;
	// }
	// if(aux3)
    // {
    //    target.pos[0] = 0;
	//    target.pos[1] = 1;
	//    aux2 = false;
	// }
	// if(aux4)
    // {
    //    target.pos[0] = 0;
	//    target.pos[1] = 0;
	//    aux3 = false;
	// }

    // target.pos[2] = 0.7;
	// target.yaw = 0;
    // if(tempo1 == 200)
	// {
	//  aux1 = true;
	//  aux2 = false;
	//  aux4 = false;
	// }
	// if(tempo1 == 400)
	// {
	//  aux2 = true;
	//  aux1 = false;
	// }
	// if(tempo1 == 600)
	// {
	//  aux3 = true;
	//  aux2 = false;
	// }
	// if(tempo1 == 800)
	// {
	//  aux4 = true;
	//  aux3 = false;
	//  tempo1 = 0;
	// }
	// tempo1++;

   //Teste do Quadrado2
//    if(aux)
//    { 
//     if(!chavear1)
//     {
//      std::cout<<"passou_chavear"<<std::endl;
// 	 after = ros::Time::now().toSec();
// 	 chavear1 = true;
//     }
// 	if(!aux1)
//     {
//      double time = ros::Time::now().toSec() - after;
//      std::cout<<"passou__"<<time<<std::endl;
// 	 target.pos[0] = time;
//      target.pos[1] = 0;
// 	 if(time > 2)
// 	 {
// 	   aux1 = true;
// 	 }
//     }
// 	if(!chavear2 && aux1)
//     {
//      after = ros::Time::now().toSec();
// 	 chavear2 = true;
// 	}
//     if(!aux2 && aux1)
//     {
//      double time = ros::Time::now().toSec() - after;
//      std::cout<<"passo111__"<<time<<std::endl;
// 	 target.pos[1] = time;
// 	 if(time > 2)
// 	 {
// 	   aux2 = true;
// 	 }
//     }
//     if(!chavear3 && aux2)
//     {
// 	 after = ros::Time::now().toSec();
// 	 chavear3 = true;
// 	}
//     if(!aux3 && aux2)
//     {
//      double time = ros::Time::now().toSec() - after;
//      std::cout<<"passo222__"<<time<<std::endl;
// 	 target.pos[0] = 0;
// 	 target.pos[1] = time;
// 	 if(time > 2)
// 	 {
// 	   aux3 = true;
// 	 }
//     }
//     if(!chavear4 && aux3)
//     {
// 	 after = ros::Time::now().toSec();
// 	}
//     if(!aux4 && aux3)
//     {
//      double time = ros::Time::now().toSec() - after;
//      std::cout<<"passo333__"<<time<<std::endl;
// 	 target.pos[0] = 0;
// 	 target.pos[1] = 0;
// 	 if(time > 2)
// 	 {
// 	   aux4 = true;
// 	//    aux1 = false;
// 	 }
//     }
//    }

	// publisher
    geometry_msgs::Twist msg;
	msg.linear.x = target.pos[0];
	msg.linear.y = target.pos[1];
	msg.linear.z = target.pos[2];
	msg.angular.z = target.yaw;
	setpoint_pub.publish(msg);

	geometry_msgs::Twist derivative_msg;
	derivative_msg.angular.x = droll*180/3.141592;
	derivative_msg.angular.y = dpitch*180/3.141592;
	derivative_pub.publish(derivative_msg);

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

    // calculate state vector
	//  TooN::Vector<> states(12);
	//  states = TooN::makeVector(
	//  	new_err[0],
	//  	speed_position[0],
	//  	new_err[1],
	//  	speed_position[1],
	//  	pose[1],
	//  	speed_pose[1],
	//  	pose[0],
	//   	speed_pose[0],
	//  	new_err[2],
	//  	speed_position[2],
	//  	new_err[3],
	//  	speed_pose[2]
	//      );

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
		calcControl(yaw, states, new_err);
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

void i_term_increase(double& i_term, double new_err, double cap)
{
	if(new_err < 0 && i_term > 0)
		i_term = std::max(0.0, i_term + 2.5 * new_err);
	else if(new_err > 0 && i_term < 0)
		i_term = std::min(0.0, i_term + 2.5 * new_err);
	else
	i_term += new_err;

	if(i_term > cap) i_term =  cap;
	if(i_term < -cap) i_term =  -cap;
}


// PID calcControl
// void DroneController::calcControl(TooN::Vector<4> new_err, TooN::Vector<4> d_error, double yaw)
// {
// 	float agr = agressiveness;
// 	if(!ptamIsGood) agr *= 0.75;
// 	agr *= scaleAccuracy;

// 	//TooN::Vector<4> d_term = new_err - last_err;	// d-term:just differentiate
// 	TooN::Vector<4> d_term = d_error;
// 	TooN::Vector<4> p_term = new_err;	// p-term is error.

// 	// rotate error to drone CS, invert pitch
// 	double yawRad = yaw * 2 * 3.141592 / 360;
// 	d_term[0] = cos(yawRad)*d_error[0] - sin(yawRad)*d_error[1];
// 	d_term[1] = - sin(yawRad)*d_error[0] - cos(yawRad)*d_error[1];

// 	p_term[0] = cos(yawRad)*new_err[0] - sin(yawRad)*new_err[1];
// 	p_term[1] = - sin(yawRad)*new_err[0] - cos(yawRad)*new_err[1];

// 	// integrate & cap
// 	double sec = getMS()/1000.0 - lastTimeStamp; lastTimeStamp = getMS()/1000.0;

// 	i_term_increase(i_term[2],new_err[2] * sec, 0.2f / Ki_gaz);
// 	i_term_increase(i_term[1],new_err[1] * sec, 0.1f / Ki_rp+(1e-10));
// 	i_term_increase(i_term[0],new_err[0] * sec, 0.1f / Ki_rp+(1e-10));

// 	// kill integral term when first crossing target
// 	// that is, thargetNew is set, it was set at least 100ms ago, and err changed sign.
// 	for(int i=0;i<4;i++)
// 		if(targetNew[i] > 0.5 && getMS()/1000.0 - targetSetAtClock > 0.1 && last_err[i] * new_err[i] < 0)
// 		{
// 			i_term[i] = 0; targetNew[i] = 0;
// 		}

//     // // Test quadratic signal
// 	// if (timer1 == 120)
// 	// {
// 	// 	chavear1 = !chavear1;
// 	// 	timer1 = 0;
// 	// }
// 	// if(!chavear1){
// 	//   aux_yaw = 0.3;
// 	//   lastSentControl.yaw = 0;
// 	// } else {
// 	//   aux_yaw = -0.3;
// 	//   lastSentControl.yaw = 0;
// 	// }
// 	// timer1+=1;

// 	// if (timer2 == 130)
// 	// {
// 	// 	chavear2 = !chavear2;
// 	// 	timer2 = 0;
// 	// }
// 	// if(!chavear2){
// 	//   aux_roll = 0.15;
// 	//   lastSentControl.roll = 0;
// 	// } else {
// 	//   aux_roll = -0.15;
// 	//   lastSentControl.roll = 0;
// 	// }
// 	// timer2+=1;

// 	// if (timer3 == 120)
// 	// {
// 	// 	chavear3 = !chavear3;
// 	// 	timer3 = 0;
// 	// }
// 	// if(!chavear3){
// 	//   aux_pitch = 0.10;
// 	//   lastSentControl.pitch = aux_pitch;
// 	// } else {
// 	//   aux_pitch = -0.15;
// 	//   lastSentControl.pitch = aux_pitch;
// 	// }
// 	// timer3+=1;

// 	// if (timer4 == 120)
// 	// {
// 	// 	chavear4 = !chavear4;
// 	// 	timer4 = 0;
// 	// }
// 	// if(!chavear4){
// 	//   aux_gaz = 0.3;
// 	//   lastSentControl.gaz = 0;
// 	// } else {
// 	//   aux_gaz = -0.3;
// 	//   lastSentControl.gaz = 0;
// 	// }
// 	// timer4+=1;

// 	// YAW
// 	lastSentControl.yaw = Kp_yaw * p_term[3] + Kd_yaw * d_term[3];	// yaw can be translated directly
//     // double i = ros::Time::now().toSec();
// 	// lastSentControl.yaw = 0;
// 	// lastSentControl.yaw = std::min(max_yaw,std::max(-max_yaw,(double)(lastSentControl.yaw)));
// 	// lastSentControl.yaw = aux_yaw;
// 	// std::cout<<"yaw"<<lastSentControl.yaw<<std::endl;

// 	// RP
// 	// calculate signals only based on d and p:
// 	double cX_p = Kp_rp * p_term[0];
// 	double cY_p = Kp_rp * p_term[1];

// 	double cX_d = Kd_rp * d_term[0];
// 	double cY_d = Kd_rp * d_term[1];

// 	double cX_i = Ki_rp * i_term[0];
// 	double cY_i = Ki_rp * i_term[1];

// 	lastSentControl.roll = cX_p + cX_d + cX_i;
// 	lastSentControl.pitch = cY_p + cY_d + cY_i;

// 	// clip
// 	// double i = ros::Time::now().toSec();
// 	// lastSentControl.roll = 0.2*sin(1.256*i)+0.1*sin(6.28*i);
// 	// lastSentControl.roll = std::min(max_rp,std::max(-max_rp,(double)(lastSentControl.roll)));
// 	// lastSentControl.roll = 0;
// 	// std::cout<<"roll"<<lastSentControl.roll<<std::endl;
//     // lastSentControl.pitch = 0.5*sin(1.256*i)+0.3*sin(6.28*i);
	
// 	// lastSentControl.pitch = std::min(max_rp,std::max(-max_rp,(double)(lastSentControl.pitch)));
// 	// lastSentControl.pitch = 0.2*sin(1.256*i)+0.1*sin(6.28*i);;
// 	// std::cout<<"pitch"<<lastSentControl.pitch<<std::endl;
// 	// GAZ
// 	double gazP = Kp_gaz * p_term[2];
// 	double gazD = Kd_gaz * d_term[2];
// 	double gazI = Ki_gaz * i_term[2];

// 	// if(lastSentControl.gaz > 0) lastSentControl.gaz *= rise_fac;
//     // double i = ros::Time::now().toSec();
// 	// lastSentControl.gaz = 0.5*sin(1.256*i)+0.3*sin(6.28*i);
// 	lastSentControl.gaz = gazP + gazD + gazI;
// 	// lastSentControl.gaz = std::min(max_gaz_rise,std::max(-max_gaz_rise,(double)(lastSentControl.gaz)));
// 	// lastSentControl.gaz = 0;
// 	// std::cout<<"gaz"<<lastSentControl.gaz<<std::endl;

// 	logInfo = TooN::makeVector(
// 		Kp_rp * p_term[0], Kp_rp * p_term[1], gazP, Kp_yaw * p_term[3],
// 		Kd_rp * d_term[0], Kd_rp * d_term[1], gazD, Kd_yaw * d_term[3],
// 		Ki_rp * i_term[0], Ki_rp * i_term[1], gazI, Ki_yaw * i_term[3],
// 		lastSentControl.roll, lastSentControl.pitch, lastSentControl.gaz, lastSentControl.yaw,
// 		lastSentControl.roll, lastSentControl.pitch, lastSentControl.gaz, lastSentControl.yaw,
// 		new_err[0],new_err[1],new_err[2],new_err[3],
// 		target.pos[0],target.pos[1],target.pos[2],target.yaw
// 		);
// }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//LQR calcControl 1
void DroneController::calcControl(double yaw, TooN::Vector<16> states, TooN::Vector<4> new_err)
{
	// pitch
    TooN::Vector<> vector_k11(16);
	// vector_k11 = TooN::makeVector(0, 0, 10, -2.11, 2.41, 0.083, 0, 0, 0, 0, 0, 0); // fase2 - teste3 - razoavel (não alcança setpoint tão bem)
	// vector_k11 = TooN::makeVector(0, 0, -0.78, -0.22, 0.31, 0.011, 0, 0, 0, 0, 0, 0, 0, 0.28, 0, 0); // fase2 - teste4 - ação integral (melhor desempenho, porém sobressinal em Z)
	// vector_k11 = TooN::makeVector(0, 0, -0.78, -0.22, 0.31, 0.011, 0, 0, 0, 0, 0, 0, 0, 0.28, 0, 0); // fase2 - teste5 - mesmo que o anterior, com Z um pouco mais suave
	// vector_k11 = TooN::makeVector(0, 0, -2.38, -0.79, 2.47, 0.73, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0); // fase3 - teste1 - muito ruim, angulos agressivos e Z sem conseguir seguir referência (sinais saturados)
	// vector_k11 = TooN::makeVector(0, 0, -4.5, -1.04, 1.48, 0.06, 0, 0, 0, 0, 0, 0, 0, 4.47, 0, 0); // fase3 - teste2 - muito ruim, assim como o anterior
	// vector_k11 = TooN::makeVector(0, 0, -0.78, -0.22, 0.31, 0.011, 0, 0, 0, 0, 0, 0, 0, 0.28, 0, 0); // fase4 - teste1 - ação intergral com Z sem saturar (Z muito lento!)
    // vector_k11 = TooN::makeVector(0, 0, -1.58, -0.42, 0.80, 0.03, 0, 0, 0, 0, 0, 0, 0, 0.89, 0, 0); // fase5 - teste 2 - bom desempenho no teste do circulo <------- Estava sendo utilizado por ultimo!!
	// vector_k11 = TooN::makeVector(0, 0, -2.23, -0.56, 1.03, 0.037, 0, 0, 0, 0, 0, 0, 0, 1.54, 0, 0); // fase5 - teste 1 
	// vector_k11 = TooN::makeVector(0.0000, 0.0000, -2.8832, -0.6539, 2.0297, 0.3160, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 3.1623, 0.0000, 0.0000); // controlador humberto
	// vector_k11 = TooN::makeVector(0, 0, -1.44, -0.96, 1.87, 0.32, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0);
	// vector_k11 = TooN::makeVector(0, 0, -1.59, -0.98, 1.90, 0.32, 0, 0, 0, 0, 0, 0, 0, 1.04, 0, 0); //fase7 - teste1 - y - pitch!!!
	// vector_k11 = TooN::makeVector(0, 0, -1.51, -0.97, 2.74, 0.59, 0, 0, 0, 0, 0, 0, 0, 1.04, 0, 0); //fase8 - nova ident roll e pitch

	// gazebo
	// vector_k11 = TooN::makeVector(0, 0, -1.5467, -1.0066, 1.9519, 0.3311, 0, 0, 0, 0, 0, 0, 0, 1.0954, 0, 0); //fase9
	// vector_k11 = TooN::makeVector(0, 0, -1.494, -0.980, 1.907, 0.324, 0, 0, 0, 0, 0, 0, 0, 1.044, 0, 0); //fase10
	// vector_k11 = TooN::makeVector(0, 0, -0.6, -0.54, 1.06, 0.20, 0, 0, 0, 0, 0, 0, 0, 0.28, 0, 0); //altern_1
	// vector_k11 = TooN::makeVector(0, 0, -0.96, -0.74, 1.44, 0.25, 0, 0, 0, 0, 0, 0, 0, 0.56, 0, 0); //altern_2

	// teste apos ajuste das derivadas
	// vector_k11 = TooN::makeVector(0, 0, -3.12, -1.59, 3.06, 0.47, 0, 0, 0, 0, 0, 0, 0, 2.86, 0, 0); //fase11_test4
	// vector_k11 = TooN::makeVector(0, 0, -3.48, -1.71, 3.28, 0.49, 0, 0, 0, 0, 0, 0, 0, 3.30, 0, 0); //fase11_test5_circulo

	/////// Humberto
	// LQR
	// vector_k11 = TooN::makeVector(0, 0, -28.20, -11.81, 28.17, 3.44, 0, 0, 0, 0, 0, 0, 0, 31.62, 0, 0); 
    // vector_k11 = TooN::makeVector(0, 0, -4.13, -1.95, 4.27, 0.62, 0, 0, 0, 0, 0, 0, 0, 4.08, 0, 0); 
	// Hinf
	// vector_k11 = TooN::makeVector(0, 0, -4.68, -2.29, 5.07, 0.69, 0, 0, 0, 0, 0, 0, 0, 3.72, 0, 0); // minimiza saída
	// vector_k11 = TooN::makeVector(0, 0, -3.97, -1.74, 3.49, 0.47, 0, 0, 0, 0, 0, 0, 0, 3.45, 0, 0); // minimiza saída
	// vector_k11 = TooN::makeVector(0, 0, -3.79, -1.69, 3.40, 0.46, 0, 0, 0, 0, 0, 0, 0, 3.28, 0, 0); // minimiza saída - bom resultado no gazebo
	// vector_k11 = TooN::makeVector(0, 0, -7.62, -3.09, 6.15, 0.71, 0, 0, 0, 0, 0, 0, 0, 7.18, 0, 0); // minimiza saída e controle 
    // Robusto
	vector_k11 = TooN::makeVector(0, 0, -2.55, -2.93, 5.30, 0.95, 0, 0, 0, 0, 0, 0, 0, 1.19, 0, 0); //

	// roll
	TooN::Vector<> vector_k21(16);
	// vector_k21 = TooN::makeVector(-10, 2.93, 0, 0, 0, 0, 3.33, 0.07, 0, 0, 0, 0);
	// vector_k21 = TooN::makeVector(0.71, 0.37, 0, 0, 0, 0, 0.58, 0.014, 0, 0, 0, 0, -0.31, 0, 0, 0);
	// vector_k21 = TooN::makeVector(0.71, 0.37, 0, 0, 0, 0, 0.58, 0.014, 0, 0, 0, 0, -0.31, 0, 0, 0);
	// vector_k21 = TooN::makeVector(2.29, 1.29, 0, 0, 0, 0, 3.38, 0.68, 0, 0, 0, 0, -1, 0, 0, 0);
	// vector_k21 = TooN::makeVector(4.43, 1.52, 0, 0, 0, 0, 2.13, 0.06, 0, 0, 0, 0, -4.47, 0, 0, 0);
	// vector_k21 = TooN::makeVector(0.71, 0.37, 0, 0, 0, 0, 0.58, 0.014, 0, 0, 0, 0, -0.31, 0, 0, 0);
	// vector_k21 = TooN::makeVector(1.49, 0.65, 0, 0, 0, 0, 0.68, 0.02, 0, 0, 0, 0, -1.0, 0, 0, 0);
    // vector_k21 = TooN::makeVector(2.16, 0.86, 0, 0, 0, 0, 2.23, 0.57, 0, 0, 0, 0, -1.0, 0, 0, 0);
	// vector_k21 = TooN::makeVector(2.8438, 0.9238, 0.0000, 0.0000, 0.0000, 0.0000, 2.3328, 0.3159, 0.0000, 0.0000, 0.0000, 0.0000, -3.1623, 0.0000, 0.0000, 0.0000);
    // vector_k21 = TooN::makeVector(2.12, 1.05, 0, 0, 0, 0, 1.67, 0.04, 0, 0, 0, 0, -1.82, 0, 0, 0);
	// vector_k21 = TooN::makeVector(1.88, 0.97, 0, 0, 0, 0, 1.56, 0.04, 0, 0, 0, 0, -1.53, 0, 0, 0); //fase7 - teste1 - x - roll
	// vector_k21 = TooN::makeVector(1.46, 0.81, 0, 0, 0, 0, 2.19, 0.34, 0, 0, 0, 0, -1.09, 0, 0, 0); //fase8 - teste1 - x - roll

	// vector_k21 = TooN::makeVector(1.5272, 0.8990, 0, 0, 0, 0, 1.9359, 0.3119, 0, 0, 0, 0, -1.0541, 0, 0, 0); //fase9
	// vector_k21 = TooN::makeVector(1.475, 0.878, 0, 0, 0, 0, 1.892, 0.306, 0, 0, 0, 0, -1.005, 0, 0, 0); //fase10
	// vector_k21 = TooN::makeVector(0.65, 0.50, 0, 0, 0, 0, 1.09, 0.19, 0, 0, 0, 0, -0.31, 0, 0, 0); //altern_1
	// vector_k21 = TooN::makeVector(1.03, 0.69, 0, 0, 0, 0, 1.49, 0.25, 0, 0, 0, 0, -0.61, 0, 0, 0); //altern_2

	// vector_k21 = TooN::makeVector(3.63, 1.64, 0, 0, 0, 0, 3.43, 0.49, 0, 0, 0, 0, -3.44, 0, 0, 0); //fase11_test4
	// vector_k21 = TooN::makeVector(4.04, 1.77, 0, 0, 0, 0, 3.68, 0.51, 0, 0, 0, 0, -3.96, 0, 0, 0); //fase11_test5_circulo

	// vector_k21 = TooN::makeVector(27.94, 10.84, 0, 0, 0, 0, 27.68, 3.41, 0, 0, 0, 0, -31.62, 0, 0, 0); 
	// vector_k21 = TooN::makeVector(4.13, 1.8, 0, 0, 0, 0, 4.21, 0.60, 0, 0, 0, 0, -4.08, 0, 0, 0); 
    // vector_k21 = TooN::makeVector(4.58, 2.07, 0, 0, 0, 0, 4.93, 0.67, 0, 0, 0, 0, -3.64, 0, 0, 0);
	// vector_k21 = TooN::makeVector(7.22, 2.58, 0, 0, 0, 0, 5.35, 0.57, 0, 0, 0, 0, -6.95, 0, 0, 0);  
	// vector_k21 = TooN::makeVector(6.87, 2.50, 0, 0, 0, 0, 5.21, 0.56, 0, 0, 0, 0, -6.51, 0, 0, 0); 
	// vector_k21 = TooN::makeVector(7.46, 2.83, 0, 0, 0, 0, 6.04, 0.68, 0, 0, 0, 0, -7.02, 0, 0, 0); 
	vector_k21 = TooN::makeVector(2.80, 1.85, 0, 0, 0, 0, 5.39, 0.91, 0, 0, 0, 0, -1.36, 0, 0, 0); 


	// Z
	TooN::Vector<> vector_k31(16);
	// vector_k31 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, 1.224, -0.72, 0, 0);
	// vector_k31 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, -1.85, -0.98, 0, 0, 0, 0, 10, 0);
	// vector_k31 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, -1.85, -0.98, 0, 0, 0, 0, 1, 0);
	// vector_k31 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, -2.29, -1.39, 0, 0, 0, 0, 1, 0);
	// vector_k31 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, -3.29, -1.57, 0, 0, 0, 0, 2.23, 0);
	// vector_k31 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, -0.74, -0.49, 0, 0, 0, 0, 0.16, 0);
	// vector_k31 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, -0.82, -0.53, 0, 0, 0, 0, 0.21, 0);
	// vector_k31 = TooN::makeVector(0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, -0.8579, -0.3173, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000, 0.0000);
    // vector_k31 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, -0.88, -0.39, 0, 0, 0, 0, 0.20, 0);
	// vector_k31 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, -5.15, -1.6, 0, 0, 0, 0, 4.47, 0);

	// vector_k31 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, -10.36, -3.7, 0, 0, 0, 0, 10.00, 0);
	// vector_k31 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, -5.49, -1.99, 0, 0, 0, 0, 4.47, 0);
	// vector_k31 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, -5.85, -1.84, 0, 0, 0, 0, 3.66, 0);
	// vector_k31 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, -6.81, -1.24, 0, 0, 0, 0, 5.39, 0);
	// vector_k31 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, -6.81, -1.24, 0, 0, 0, 0, 5.39, 0);
	// vector_k31 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, -5.36, -1.36, 0, 0, 0, 0, 3.71, 0);
	vector_k31 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, -2.67, -0.71, 0, 0, 0, 0, 0.93, 0);


    // psi
    TooN::Vector<> vector_k41(16);
	// vector_k41 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.387, 0.030);
	// vector_k41 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.54, 0.04, 0, 0, 0, -0.22);
	// vector_k41 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.54, 0.04, 0, 0, 0, -0.22);
	// vector_k41 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.87, 0.63, 0, 0, 0, -1);
	// vector_k41 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2.01, 0.17, 0, 0, 0, -2.23);
	// vector_k41 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.45, 0.035, 0, 0, 0, -0.15);
	// vector_k41 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.45, 0.035, 0, 0, 0, -0.16);
	// vector_k41 = TooN::makeVector(0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0088, 0.0027, 0.0000, 0.0000, 0.0000, -0.0100);
    // vector_k41 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.19, 0.08, 0, 0, 0, -1.00);

	// vector_k41 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8.78, 2.72, 0, 0, 0, -10.00);
	// vector_k41 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.29, 0.16, 0, 0, 0, -1.00);
	// vector_k41 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.22, 0.22, 0, 0, 0, -0.94);
	// vector_k41 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2.45, 0.22, 0, 0, 0, -2.46);
	// vector_k41 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2.45, 0.22, 0, 0, 0, -2.46);
	// vector_k41 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.38, 0.22, 0, 0, 0, -1.15);
	vector_k41 = TooN::makeVector(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.01, 0.05, 0, 0, 0, -0.43);


	// YAW
	// if(vector_k41*states > 1 || vector_k41*states < -1)
	// {
	//   new_int_err[3] = 0.7 * new_int_err[3];
	//   std::cout<<"yaw_cmd_saturando"<<std::endl;  
	// }
	lastSentControl.yaw = vector_k41*states;
	// std::cout<<"yaw_cmd"<<lastSentControl.yaw<<std::endl;
	// lastSentControl.yaw = std::min(max_yaw,std::max(-max_yaw,(double)(lastSentControl.yaw)));

    // ROLL and PITCH
	double pitch_cmd = vector_k11*states;
	double roll_cmd = vector_k21*states;

	// if(roll_cmd > 1 || roll_cmd < -1)
	// {
	//   new_int_err[1] = 0.7 * new_int_err[1];
	//   std::cout<<"roll_cmd_saturando"<<std::endl;
	// }
	lastSentControl.roll = roll_cmd;
	// std::cout<<"roll_cmd"<<lastSentControl.roll<<std::endl;
	// lastSentControl.roll = std::min(max_rp,std::max(-max_rp,(double)(lastSentControl.roll)));

	// if(pitch_cmd > 1 || pitch_cmd < -1)
	// {
	//   new_int_err[0] = 0.7 * new_int_err[0];
	//   std::cout<<"pitch_cmd_saturando"<<std::endl;
	// }
	lastSentControl.pitch = pitch_cmd;
	// std::cout<<"pitch_cmd"<<lastSentControl.pitch<<std::endl;
	// lastSentControl.pitch = std::min(max_rp,std::max(-max_rp,(double)(lastSentControl.pitch)));

	// GAZ
	// if(vector_k31*states > 1 || vector_k31*states < -1)
	// {
	//   new_int_err[2] = 0.7 * new_int_err[2];
	//   std::cout<<"gaz_cmd_saturando"<<std::endl;
	// }
	lastSentControl.gaz = vector_k31*states;
	// std::cout<<"gaz_cmd"<<lastSentControl.gaz<<std::endl;
	// lastSentControl.gaz = std::min(max_gaz_rise,std::max(max_gaz_drop,(double)lastSentControl.gaz));
	// if(lastSentControl.gaz > 0) lastSentControl.gaz *= rise_fac;
}

TooN::Vector<4> DroneController::getLastErr()
{
	return last_err;
}
ControlCommand DroneController::getLastControl()
{
	return lastSentControl;
}

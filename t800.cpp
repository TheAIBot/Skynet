#include <iostream>
#include <stdlib.h>
#include <string>
#include "includes/robotconnector.h"
#include "includes/odometry.h"
#include "includes/log.h"
#include "includes/commands.h"
#include "includes/stopconditions.h"
#include "includes/lasersensor.h"

#define WHEEL_DIAMETER   0.067	// m
#define WHEEL_SEPARATION 0.2735	// m
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define TICKS_PER_SECOND 100
#define STD_SPEED 0.2

#define SIMULATE_FLOOR_ARG "-floor"
#define USE_REAL_CALIB_ARG "-real"
#define SIM_LINE_SENSOR_CALIB_FILE_NAME "sensor_calib_scripts/linesensor_calib_sim.txt"
#define REAL_LINE_SENSOR_CALIB_FILE_NAME "sensor_calib_scripts/linesensor_calib_real.txt"
#define SIM_IR_SENSOR_CALIB_FILE_NAME "sensor_calib_scripts/irSensorCalib_sim.txt"
#define REAL_IR_SENSOR_CALIB_FILE_NAME "sensor_calib_scripts/irSensorCalib_real.txt"

/*
 * Load all the robots calibrations
 */
static void loadCalibrations(bool useSimCalibrations)
{
	//default is sim calibration
	std::string lineSensorCalibFileName;
	std::string irSensorCalibFileName;
	if (useSimCalibrations)
	{
		std::cout << "Using simulation calibrations" << std::endl;
		lineSensorCalibFileName = SIM_LINE_SENSOR_CALIB_FILE_NAME;
		irSensorCalibFileName = SIM_IR_SENSOR_CALIB_FILE_NAME;
	}
	else
	{
		std::cout << "Using real world calibrations" << std::endl;
		lineSensorCalibFileName = REAL_LINE_SENSOR_CALIB_FILE_NAME;
		irSensorCalibFileName = REAL_IR_SENSOR_CALIB_FILE_NAME;
	}
	//need calib file for the problem to work
	if (!loadLineSensorCalibrationData(lineSensorCalibFileName.c_str()))
	{
		exit(EXIT_FAILURE);
	}

	//need calib file for the problem to work
	if (!loadIRCalibrationData(irSensorCalibFileName.c_str()))
	{
		exit(EXIT_FAILURE);
	}
}

static void toTheBoxAndTakeMeasurements(odotype* const odo, const bool inSim)
{
	followLine(odo, 0.3, STD_SPEED*2, LineCentering::right, LineColor::black, &noStopCondition);
	followLine(odo, 100, STD_SPEED, LineCentering::right, LineColor::black, &stopAtLine<LineColor::black, 7>);
	//these are not the correct numbers
	if (inSim)
	{

		std::cout << "Distance: " << getLaserDistance(LaserDistance::laser_center) + 1.6 << std::endl;
	}
	else
	{
		std::cout << "Distance: " << getLaserDistance(LaserDistance::laser_center) + 1.935 << std::endl;
	}

}

static void handleObstacle(odotype* const odo, const bool inSim)
{
	//go to line towards box to move
	turn(odo, ANGLE(90), STD_SPEED, &noStopCondition);
	fwd(odo, 100, STD_SPEED, &stopAtLine<LineColor::black, 4>);
	fwd(odo, 0.1, STD_SPEED, &noStopCondition);
	fwd(odo, 100, STD_SPEED, &stopAtLine<LineColor::black, 4>);
	fwd(odo, 0.25, STD_SPEED, &noStopCondition);
	turn(odo, ANGLE(-45), STD_SPEED, &noStopCondition);
	turn(odo, ANGLE(-180), STD_SPEED, &stopAtParallelLine<LineColor::black>);

	//push box and go through gate
	followLine(odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtLine<LineColor::black, 4>);

	if (inSim)
	{
		fwd(odo, 0.135, STD_SPEED, &noStopCondition);
	}
	else
	{
		fwd(odo, 0.05, STD_SPEED / 2, &noStopCondition);
	}
	turn(odo, ANGLE(45), STD_SPEED / 2, &noStopCondition);
	turn(odo, ANGLE(90), STD_SPEED / 2, &stopAtParallelLine<LineColor::black>);

	followLine(odo, 1, STD_SPEED, LineCentering::right, LineColor::black, &stopAtLine<LineColor::black, 6>);
	followLine(odo, 1, STD_SPEED, LineCentering::right, LineColor::black, &stopAtLine<LineColor::black, 5>);
}

static void throughTheGateAndToTheWall(odotype* const odo, const bool inSim)
{
	//Around the gates and to the wall.
	//printf();
	//followLine(odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtDetectedPillar<IRSensor::ir_left, 50>);
	//fwd(odo, 0.1, STD_SPEED, &noStopCondition);
	followLine(odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtLaserDetectedPillar<-90, -80, 50>); //changed to laser here
	if (inSim){
		followLine(odo, 0.52, STD_SPEED, LineCentering::center, LineColor::black, &noStopCondition); // change this to detect the wall?
		turn(odo, ANGLE(90), STD_SPEED, &noStopCondition);
		throughGate(odo, 2, STD_SPEED, &stopAtLaserDetectedPillar<-5, 5, 20>);
	}
	else {
		followLine(odo, 0.54, STD_SPEED, LineCentering::center, LineColor::black, &noStopCondition); // change this to detect the wall?
		turn(odo, ANGLE(90), STD_SPEED, &noStopCondition);
		fwd(odo, 0.5, STD_SPEED, &noStopCondition);
		fwd(odo, 2, STD_SPEED, &stopAtLaserDetectedPillar<-10, 10, 20>);
	}
	//Is through the gates, and facing the wall
	turn(odo, ANGLE(-90), STD_SPEED, &noStopCondition);
	fwd(odo, 3, STD_SPEED, &stopAtLine<LineColor::black, 4>);
	fwd(odo, 0.25, STD_SPEED, &noStopCondition);
	turn(odo, ANGLE(-180), STD_SPEED, &stopAtParallelLine<LineColor::black>);
	followLine(odo, 0.3, STD_SPEED, LineCentering::center, LineColor::black, &noStopCondition);
	turn(odo, ANGLE(90), STD_SPEED, &noStopCondition);
	turn(odo, ANGLE(180), STD_SPEED, &stopAtParallelLine<LineColor::black>);
	followLine(odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtLine<LineColor::black, 5>); //Stop at the wall.
	//Go to the start of the wall, and get ready to follow it:s
	followLine(odo, 0.6, STD_SPEED, LineCentering::left, LineColor::black, &noStopCondition);
	turn(odo, ANGLE(90), STD_SPEED, &noStopCondition);
	fwd(odo, 0.3, STD_SPEED, &noStopCondition);
}

static void followTheWall(odotype* const odo, const bool inSim)
{
	//Follow first side of the wall
	//fwd(odo, 0.1, 0.2, &noStopCondition);
	followWall(odo, 0.6, 0.20, STD_SPEED, &stopAtBlankSpace<LaserDistance::laser_left, 60>);
	followWall(odo, 3, 0.20, STD_SPEED / 2, &stopAtBlankSpace<LaserDistance::laser_left, 60>);
	
	if(inSim) 
	{
	  fwd(odo, 0.48, STD_SPEED, &noStopCondition); //From 43 centimeters
	}
	else
	{ 
	  fwd(odo, 0.53, STD_SPEED, &noStopCondition);
	}
	
	turn(odo, ANGLE(90), 0.3, &noStopCondition);
	//Pass the wall gate
	fwd(odo, 0.95, STD_SPEED, &noStopCondition);
	////changed here
	turn(odo, ANGLE(90), 0.3, &noStopCondition);
	fwd(odo, 0.3, STD_SPEED, &noStopCondition);
	//Follow other side of the wall
	followWall(odo, 0.7, 0.20, STD_SPEED, &stopAtBlankSpace<LaserDistance::laser_left, 60>);
	followWall(odo, 3, 0.20, STD_SPEED / 2, &stopAtBlankSpace<LaserDistance::laser_left, 60>);
	fwd(odo, 1, STD_SPEED, &stopAtLine<LineColor::black, 5>);
	fwd(odo, 0.3, STD_SPEED, &noStopCondition);
	turn(odo, ANGLE(-180), STD_SPEED, &stopAtParallelLine<LineColor::black>);
	followLine(odo, 0.3, STD_SPEED, LineCentering::center, LineColor::black, &noStopCondition);
	turn(odo, ANGLE(90), STD_SPEED, &noStopCondition);
	turn(odo, ANGLE(180), STD_SPEED, &stopAtParallelLine<LineColor::black>);
	//Back at black line, and to the white line
	if (inSim){
		followLine(odo, 100, STD_SPEED, LineCentering::left, LineColor::black, &stopAtLine<LineColor::black, 6>);
	}
	else{
		followLine(odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtLine<LineColor::black, 5>);
	}
	followLine(odo, 0.3, STD_SPEED, LineCentering::center, LineColor::black, &noStopCondition);
}

static void followTheWhiteLine(odotype* const odo, const bool inSim)
{
	followLine(odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtLine<LineColor::black, 5>);
	fwd(odo, 0.45, STD_SPEED, &noStopCondition);
	turn(odo, ANGLE(180), STD_SPEED, &stopAtParallelLine<LineColor::white>);
	//Follow white line:
	if (inSim){
		followLine(odo, 100, STD_SPEED, LineCentering::center, LineColor::white, &stopAtLine<LineColor::white, 4>);
		fwd(odo, 0.2, STD_SPEED, &noStopCondition);
	}
	else{
		followLine(odo, 2, STD_SPEED, LineCentering::center, LineColor::white, &stopAtLine<LineColor::black, 4>);
		followLine(odo, 100, STD_SPEED/2, LineCentering::center, LineColor::white, &stopAtLine<LineColor::black, 4>);
		turn(odo, ANGLE(-10), STD_SPEED, &noStopCondition);
		fwd(odo, 0.05, STD_SPEED, &noStopCondition);
		followLine(odo, 0.40, STD_SPEED/2, LineCentering::center, LineColor::black, &noStopCondition);
		
		/*
		turn(odo, ANGLE(45), STD_SPEED, &noStopCondition);
		fwd(odo, 0.15, STD_SPEED, &noStopCondition);
		fwd(odo, 0.30, STD_SPEED, &stopAtParallelLine<LineColor::black>);
		followLine(odo, 0.2, STD_SPEED / 2, LineCentering::center, LineColor::black, &noStopCondition);
		turn(odo, ANGLE(-180), STD_SPEED, &stopAtParallelLine<LineColor::black>);
		*/
	}
	turn(odo, ANGLE(-25), STD_SPEED, &noStopCondition);
	turn(odo, ANGLE(-180), STD_SPEED, &stopAtParallelLine<LineColor::black>);
}

static void handleEnclosure(odotype* const odo, const bool inSim)
{
	followLine(odo, 100, STD_SPEED / 2, LineCentering::center, LineColor::black, &stopAtDetectedPillar<IRSensor::ir_front_center, 15>);
	turn(odo, ANGLE(90), STD_SPEED, &noStopCondition);
	if (inSim)
	{
		fwd(odo, 0.65, STD_SPEED, &noStopCondition);
	}
	else
	{
		fwd(odo, 0.65, STD_SPEED, &noStopCondition);
	}
	turn(odo, ANGLE(-90), STD_SPEED, &noStopCondition);
	fwd(odo, 0.3, STD_SPEED, &noStopCondition);
	//Smack the gate.
	turn(odo, ANGLE(-180), 0.5, &noStopCondition);
	if (inSim)
	{
		fwd(odo, 0.35, STD_SPEED, &noStopCondition);
		turn(odo, ANGLE(90), STD_SPEED, &noStopCondition);
		fwd(odo, 1, STD_SPEED, &stopAtLine<LineColor::black, 5>);
		fwd(odo, 0.2, STD_SPEED, &noStopCondition);
		turn(odo, ANGLE(180), STD_SPEED, &stopAtParallelLine<LineColor::black>);
		followLine(odo, 0.2, STD_SPEED / 2, LineCentering::center, LineColor::black, &noStopCondition);
		followLine(odo, 100, STD_SPEED / 2, LineCentering::center, LineColor::black, &stopAtLaserDetectedPillar<-10, 10, 15>);
	}
	else
	{
		turn(odo, ANGLE(40), STD_SPEED, &noStopCondition);
		fwd(odo, 1, STD_SPEED, &stopAtLine<LineColor::black, 3>);
		fwd(odo, 0.3, STD_SPEED, &noStopCondition);
		turn(odo, ANGLE(75), STD_SPEED, &noStopCondition);
		turn(odo, ANGLE(90), STD_SPEED, &stopAtParallelLine<LineColor::black>);
		followLine(odo, 0.3, STD_SPEED / 2, LineCentering::center, LineColor::black, &noStopCondition);
		followLine(odo, 100, STD_SPEED / 2, LineCentering::center, LineColor::black, &stopAtLaserDetectedPillar<-10, 10, 15>);
	}
}

int main(int argc, char* argv[])
{
	odotype odo = { 0 };
	//use sim calibs as default
	bool useSimCalibs = true;

	for (int i = 1; i < argc; ++i)
	{
		const std::string argument = argv[i];
		if (argument.compare(std::string(USE_REAL_CALIB_ARG)) == 0)
		{
			useSimCalibs = false;
		}
		if (argument.compare(std::string(SIMULATE_FLOOR_ARG)) == 0)
		{
			std::cout << "Simulating floor" << std::endl;
			simulateFloor = true;
		}
	}

	loadCalibrations(useSimCalibs);

	//can't run program if can't connect to robot
	if (!connectRobot())
	{
		exit(EXIT_FAILURE);
	}

	/* Read sensors and zero our position.
	 */
	rhdSync();
	
	odo.wheelSeparation = WHEEL_SEPARATION;
	odo.metersPerEncoderTick = DELTA_M;
	odo.wheelsEncoderTicks.left = lenc->data[0];
	odo.wheelsEncoderTicks.right = renc->data[0];
	odo.oldWheelsEncoderTicks = odo.wheelsEncoderTicks;
	

	toTheBoxAndTakeMeasurements(&odo, useSimCalibs);
	handleObstacle(&odo, useSimCalibs);
	throughTheGateAndToTheWall(&odo, useSimCalibs);
	followTheWall(&odo, useSimCalibs);
	followTheWhiteLine(&odo, useSimCalibs);
	handleEnclosure(&odo, useSimCalibs);

	forceSetMotorSpeeds(0, 0);
	rhdSync();
	rhdDisconnect();
	writeLogs("logging.txt");
	exit(0);
}

#include <iostream>
#include <stdlib.h>
#include <string>
#include "includes/robotconnector.h"
#include "includes/odometry.h"
#include "includes/log.h"
#include "includes/commands.h"
#include "includes/stopconditions.h"

#define WHEEL_DIAMETER   0.067	// m
#define WHEEL_SEPARATION 0.256	// m
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
		printf("Using simulation calibrations\n");
		lineSensorCalibFileName = SIM_LINE_SENSOR_CALIB_FILE_NAME;
		irSensorCalibFileName = SIM_IR_SENSOR_CALIB_FILE_NAME;
	}
	else
	{
		printf("Using real world calibrations\n");
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
	odo.leftWheelEncoderTicks = lenc->data[0];
	odo.rightWheelEncoderTicks = renc->data[0];
	odo.oldLeftWheelEncoderTicks = odo.leftWheelEncoderTicks;
	odo.oldRightWheelEncoderTicks = odo.rightWheelEncoderTicks;

	//throughGate(&odo, 10, STD_SPEED, &noStopCondition); //changed here

	//printf("Follow line first\n");
	//go to box
	followLine(&odo, 100, STD_SPEED, LineCentering::right, LineColor::black, &stopAtLine<LineColor::black, 7>);
	//printf("Turn first\n");
	//go to line towards box to move
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);
	fwd(&odo, 100, STD_SPEED, &stopAtLine<LineColor::black, 5>);
	fwd(&odo, 0.1, STD_SPEED, &noStopCondition);
	fwd(&odo, 100, STD_SPEED, &stopAtLine<LineColor::black, 5>);
	fwd(&odo, 0.3, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(-180), STD_SPEED, &stopAtParallelLine<LineColor::black>);

	//push box and go through gate
	followLine(&odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtLine<LineColor::black, 7>);
	fwd(&odo, 0.155, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(90), STD_SPEED / 2, &noStopCondition);
	followLine(&odo, 1, STD_SPEED, LineCentering::right, LineColor::black, &stopAtLine<LineColor::black, 7>);
	followLine(&odo, 1, STD_SPEED, LineCentering::right, LineColor::black, &stopAtLine<LineColor::black, 6>);

	//Around the gates and to the wall.
	followLine(&odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtDetectedPillar<IRSensor::ir_left, 50>);
	followLine(&odo, 0.4, STD_SPEED, LineCentering::center, LineColor::black, &noStopCondition); // change this to detect the wall?
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);
	throughGate(&odo, 1, STD_SPEED, &noStopCondition); //changed here
	//fwd(&odo, 1, STD_SPEED, &noStopCondition);
	//Is thorugh the gates
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);
	fwd(&odo, 0.6, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);
	//Back to the line
	fwd(&odo, 100, STD_SPEED, &stopAtLine<LineColor::black, 5>);
	fwd(&odo, 0.3, STD_SPEED, &noStopCondition);
	//Turn around untill the line has been found.
	turn(&odo, ANGLE(180), STD_SPEED, &stopAtParallelLine<LineColor::black>);
	//Go up to the wall:
	followLine(&odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtLine<LineColor::black, 7>);
	fwd(&odo, 0.3, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(180), STD_SPEED, &stopAtParallelLine<LineColor::black>);
	fwd(&odo, 0.1, STD_SPEED, &noStopCondition);

	//Stop at the wall.
	followLine(&odo, 100, STD_SPEED, center, LineColor::black, &stopAtLine<LineColor::black, 5>);
	followLine(&odo, 0.6, STD_SPEED, center, LineColor::black, &noStopCondition);
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);
	fwd(&odo, 0.25, STD_SPEED, &noStopCondition);
	//Follow wall	
	followWall(&odo, 0.7, 20, STD_SPEED, &stopAtBlankSpace<IRSensor::ir_left, 60>);
	followWall(&odo, 3, 20, STD_SPEED / 2, &stopAtBlankSpace<IRSensor::ir_left, 60>);
	fwd(&odo, 0.4, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(90), 0.3, &noStopCondition);
	throughGate(&odo, 0.78, STD_SPEED, &noStopCondition); //changed here
	//fwd(&odo, 0.78, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(90), 0.3, &noStopCondition);
	fwd(&odo, 0.3, STD_SPEED, &noStopCondition);
	followWall(&odo, 0.7, 30, STD_SPEED, &stopAtBlankSpace<IRSensor::ir_left, 60>);
	followWall(&odo, 3, 30, STD_SPEED / 2, &stopAtBlankSpace<IRSensor::ir_left, 60>);
	fwd(&odo, 0.50, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(180), STD_SPEED, &stopAtParallelLine<LineColor::black>);
	//Back at black line
	followLine(&odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtLine<LineColor::black, 6>);
	fwd(&odo, 0.15, STD_SPEED, &noStopCondition);
	followLine(&odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtLine<LineColor::black, 6>);
	fwd(&odo, 0.4, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition); //Stop at white line (*)

	//Follow white line:		
	followLine(&odo, 100, STD_SPEED, LineCentering::center, LineColor::white, &stopAtLine<LineColor::black, 8>);
	fwd(&odo, 0.4, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(-90), STD_SPEED, &noStopCondition);
	followLine(&odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtDetectedPillar<IRSensor::ir_front_center, 30>);

	forceSetMotorSpeeds(0, 0);
	rhdSync();
	rhdDisconnect();
	writeLogs("logging.txt");
	exit(0);
}


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

#define USE_REAL_CALIB_ARG "-real"
#define SIM_LINE_SENSOR_CALIB_FILE_NAME "sensor_calib_scripts/linesensor_calib_sim.txt"
#define REAL_LINE_SENSOR_CALIB_FILE_NAME "sensor_calib_scripts/linesensor_calib_real.txt"
#define SIM_IR_SENSOR_CALIB_FILE_NAME "sensor_calib_scripts/irSensorCalib_sim.txt"
#define REAL_IR_SENSOR_CALIB_FILE_NAME "sensor_calib_scripts/irSensorCalib_real.txt"

#define SIMULATE_FLOOR_ARG "-floor"

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
	if (!readLineSensorCalibrationData(lineSensorCalibFileName.c_str()))
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
	bool useSimCalibs = true;

	for (int i = 1; i < argc; ++i)
	{
		const std::string argument = argv[i];
		//std::cout << argument << std::endl;
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

	followLine(&odo, 100, STD_SPEED, LineCentering::left, LineColor::black, &noStopCondition);

	//go to box
	followLine(&odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtLine<LineColor::black, 4>);

	//go to line towards box to move
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);
	fwd(&odo, 100, STD_SPEED, &stopAtLine<LineColor::black, 4>);
	fwd(&odo, 0.1, STD_SPEED, &noStopCondition);
	fwd(&odo, 100, STD_SPEED, &stopAtLine<LineColor::black, 4>);
	fwd(&odo, 0.3, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(-90), STD_SPEED, &stopAtParallelLine<LineColor::black>);

	//push box and go through gate
	followLine(&odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtLine<LineColor::black, 4>);
	fwd(&odo, 0.135, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);
	followLine(&odo, 1, STD_SPEED, LineCentering::right, LineColor::black, &stopAtLine<LineColor::black, 4>);
	followLine(&odo, 1, STD_SPEED, LineCentering::center, LineColor::black, &stopAtLine<LineColor::black, 4>);

	//Around the gates and to the wall.
	followLine(&odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtDetectedPillar<IRSensor::ir_left, 50>);
	followLine(&odo, 0.4, STD_SPEED, LineCentering::center, LineColor::black, &noStopCondition); // change this to detect the wall?
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);
	fwd(&odo, 1, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);
	fwd(&odo, 0.6, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);
	fwd(&odo, 100, STD_SPEED, &stopAtLine<LineColor::black, 4>);
	fwd(&odo, 0.3, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(180), STD_SPEED, &stopAtParallelLine<LineColor::black>);

	followLine(&odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtLine<LineColor::black, 6>);
	fwd(&odo, 0.3, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);
	fwd(&odo, 0.1, STD_SPEED, &noStopCondition);

	followLine(&odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtDetectedPillar<IRSensor::ir_right, 20>);
	fwd(&odo, 0.5, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);

	/*
	turn(&odo, ANGLE(90), 0.3, &noStopCondition);
	followLine(&odo, 100, 0.2, center, black, &stopAtBlockedForwardPath);
	double distance = measureDistance(&odo);
	turn(&odo, ANGLE(90), 0.3, &noStopCondition);
	followWall(&odo, distance, 0.2, &stopAtFreeRightIR);
	*/

	forceSetMotorSpeeds(0, 0);
	rhdSync();
	rhdDisconnect();
	writeLogs("logging.txt");
	exit(0);
}


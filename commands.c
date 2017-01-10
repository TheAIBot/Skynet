
#include "includes/commands.h"


//Methods used by the commands


void setMotorSpeeds(const double leftSpeed, const double rightSpeed){
	//printf("%f %f\n", leftSpeed, rightSpeed);

	speedl->data[0] = 100 * leftSpeed;
	speedl->updated = 1;
	speedr->data[0] = 100 * rightSpeed;
	speedr->updated = 1;
}

static inline double min(const double x, const double y){
	return ((x) < (y)) ? (x) : (y);
}

static inline double max(const double x, const double y){
	return ((x) > (y)) ? (x) : (y);
}

static double getAcceleratedSpeed(const double stdSpeed, const double distanceLeft, const int tickTime){
	const double speedFunc = sqrt(2 * (MAX_ACCELERATION) * distanceLeft);
	const double accFunc = (MAX_ACCELERATION / TICKS_PER_SECOND) * tickTime;
	const double speed = min(min(stdSpeed, speedFunc), accFunc);
	//printf("%f %f %f %d %f\n", stdSpeed, speedFunc, accFunc, tickTime, speed);
	return speed;
}

static void syncAndUpdateOdo(odotype *odo){
	if (lmssrv.config && lmssrv.status && lmssrv.connected)
	{
		while ((xml_in_fd(xmllaser, lmssrv.sockfd) > 0))
		{
			xml_proca(xmllaser);
		}
	}

	if (camsrv.config && camsrv.status && camsrv.connected)
	{
		while ((xml_in_fd(xmldata, camsrv.sockfd) > 0))
		{
			xml_proc(xmldata);
		}
	}

	rhdSync();
	odo->leftWheelEncoderTicks = lenc->data[0];
	odo->rightWheelEncoderTicks = renc->data[0];
	updateOdo(odo);
}

static void exitOnButtonPress(){
	int arg;
	ioctl(0, FIONREAD, &arg);
	if (arg != 0){
		rhdSync();
		rhdDisconnect();
		exit(0);
	}
}




//The actual commands:

void fwd(odotype *odo, const double dist, const double speed){
	const double startpos = (odo->rightWheelPos + odo->leftWheelPos) / 2;
	int time = 0;

	double distLeft;
	do
	{
		syncAndUpdateOdo(odo);

		distLeft = dist - (((odo->rightWheelPos + odo->leftWheelPos) / 2) - startpos);

		const double motorSpeed = max(getAcceleratedSpeed(speed, distLeft, time), MIN_SPEED);

		setMotorSpeeds(motorSpeed, motorSpeed);

		time++;

		exitOnButtonPress();

	} while (distLeft > 0);

	setMotorSpeeds(0, 0);
}

void fwdTurn(odotype *odo, const double angle, const double speed){
	int time = 0;
	//angle %= 2*M_PI; //Setting it in the range of 0 to 2 Pi.
	//printf("Starting with angle = %f, odo angle = %f\n", angle, odo->angle);
	double angleDifference;
	do{
		//printf("%f, %f\n", angleDifference, ANGLE(0.5));
		syncAndUpdateOdo(odo);
		angleDifference = angle - odo->angle;
		double deltaV = max(K_MOVE_TURN * (angleDifference), MIN_SPEED); //Check this for general case.(*)
		//printf("deltaV = %f\n", deltaV);
		const double motorSpeed = max(getAcceleratedSpeed(speed, deltaV / 4, time) / 2, MIN_SPEED); //Modify to use this (*)
		setMotorSpeeds(motorSpeed - deltaV / 2, motorSpeed + deltaV / 2);
		time++;
		exitOnButtonPress();
	} while (fabs(angleDifference) > ANGLE(0.1));
	//printf("%f\n", angleDifference);

	setMotorSpeeds(0, 0);
}

void turn(odotype *odo, const double angle, const double speed){
	const double startpos = (angle > 0) ? odo->rightWheelPos : odo->leftWheelPos;
	int time = 0;

	double distLeft;
	do
	{
		syncAndUpdateOdo(odo);

		distLeft = (fabs(angle) * odo->wheelSeparation) / 2 - (((angle > 0) ? odo->rightWheelPos : odo->leftWheelPos) - startpos);

		const double motorSpeed = max(getAcceleratedSpeed(speed, distLeft, time) / 2, MIN_SPEED);
		if (angle > 0)
		{
			setMotorSpeeds(-motorSpeed, motorSpeed);
		}
		else
		{
			setMotorSpeeds(motorSpeed, -motorSpeed);
		}

		time++;

		exitOnButtonPress();

	} while (distLeft > 0);

	setMotorSpeeds(0, 0);
}

void followLine(odotype *odo, const double dist, const double speed, const enum lineCentering centering){
	const double endPosition = odo->totalDistance + dist;
	int time = 0;

	double distLeft;
	do
	{
		syncAndUpdateOdo(odo);

		distLeft = endPosition - odo->totalDistance;

		const double motorSpeed = max(getAcceleratedSpeed(speed, distLeft, time), MIN_SPEED);
		const double lineOffDist = getLineOffSetDistance(centering);
		const double thetaRef = atan(lineOffDist / WHEEL_CENTER_TO_LINE_SENSOR_DISTANCE) + odo->angle;
		const double K = 2;
		const double speedDiffPerMotor = (K * (thetaRef - odo->angle)) / 2;

		setMotorSpeeds(motorSpeed - speedDiffPerMotor, motorSpeed + speedDiffPerMotor);

		time++;
		exitOnButtonPress();

	} while (distLeft > 0);

	setMotorSpeeds(0, 0);
}





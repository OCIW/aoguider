/*-------------------------------------------------------------------
-------------------------------------------------------------------*/

/* aoguider

	2012-04-12 au - added calibration routine
	2012-04-06 au - added homing routine
	2012-04-05 au - start

	Test program demonstrating the Magellan AO guider using
	a Galil DMC 4060 controller. Compiles under Windows/Cygwin.


*/

#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cygwin/in.h>
//#include <netinet/in.h>

//#define GALILIP	"192.168.1.2"		// Generic private IP address
#define GALILIP 	"192.91.178.197"	// Jorge's sodium

#define XAXIS		1
#define YAXIS		2
#define ZAXIS		3
#define Y1AXIS		4
#define Y2AXIS		5
#define	SAXIS		6

#define BLOCKING	0
#define NONBLOCKING	1

#define LEDERROR	1
#define CONNECTERROR	2
#define SOCKETERROR	3
#define INETPTONERROR	4
#define BADMOVE		5

#define NOTHOMED	6
#define FOCUSREL	3
#define FOCUSABS	2
#define ABSOLUTE	1
#define RELATIVE	0

// brake status
#define OFF		0
#define	ON		1
#define STATUS		2
#define	PASS		1
#define FAIL		0
#define UNKNOWN		-1
#define RETRACT		0
#define EXTEND		1

#define	IN		1
#define OUT		0

#define XMAXSTEPS	21000		// Maximum x-motor steps after homing
#define YMAXSTEPS	13000		// Maximum y-motor steps after homing
#define	ZMAXSTEPS	5000		// Maximum z-motor steps after homing
#define XSTEPSPERTURN	500		// Motor steps in 360 degrees
#define YSTEPSPERTURN	500
#define ZSTEPSPERTURN	200
#define XENCPULSPERTURN	2000
#define YENCPULSPERTURN	2000
#define XSCREWPITCH	6.0
#define YSCREWPITCH	6.0

//X and Y axis defaults
#define DEFSPEED	2000		// Speed
#define DEFACCEL	9000		// Acceleration
#define DEFDECEL	20000		// Deceleration
#define LIMITHYSTER	40		// Limit switch hysteresis

// Function prototypes
void	backOff();
void	demo();
void	fakeHome(void);
void	askGalil(char *, char *, int);
int	askGalilForInt(char *);
long int askGalilForLong(char *);
int	limitSwitch(int);
int	brake(int, int);
int	calibrate(void);
void	cmdLoop(void);
long int creepToLimits(int, int, int);
int	cylinder(int, int);
void	debug(void);
long int encPosition(int);
void	errmsg(int);
int	fieldCam(void);
int	fieldLens(void);
void	focus(int);
void	focusRel(long int);
int	getKey(void);
//int	getLimits(int);
int	help(void);
int	homeAxes(void);
void	initGuider(void);
int	ledInOut(void);
void	move(int);
void	moveAbs(float, float);
void	moveRel(long int, long int);
float	mmPosition(int);
void	passthru(void);
void	resetGalil(void);
int	shCam(void);
int	selfCheck();
void	setMode(int);
int	smallAp(void);
void	statusPrint(void);
long int stepPosition(int);
int	tellGalil(char *);
int	telnetToGalil(char *);
void	testFunction(void);

/* Globals */

int galilfd = -1;			// file descriptor to Galil
int debugFlag = 0;			// Turn on debug print statements
int isCalibrated = 0;
long int homeTime = -9999;		// Galil TIME that axes were homed
long int xEncOffset, yEncOffset;	// Encoder values at home position
long int xEncMin, yEncMin;		// Minimum legal encoder value
float xEncPerStep, yEncPerStep;		// Encoder pulses per motor step

int main(argv, argc)
int argv;
char *argc[];
{

	char ipaddress[80];

	if (argv == 2) {
		strcpy(ipaddress, argc[1]);
	} else {
		strcpy(ipaddress, GALILIP);
	}
	galilfd = telnetToGalil(ipaddress);
	for (;;) {
		cmdLoop();
	}

}

/*-------------------------------------------------------------------

	askGalil(cmd, buf, n);

	askGalil sends a command string (cmd) to the Galil controller
	and returns the controller's reply in buf. n is the available
	space in buf.  The global variable galilfd must already be a
	valid socket to the Galil controller.

	cmd is a pointer to a NUL terminated string containing the
	Galil command. For example, "TPA" (Galil command "Tell
	Position, A axis. askGalil adds a carriage return ('\r') to
	the command sent to the Galil. The reply string from the Galil
	is returned in buf.

-------------------------------------------------------------------*/

void askGalil(cmd, buf, n)
char *cmd, *buf;
int n;
{

	char cmdstr[80];

	strcpy(cmdstr, cmd);
	strcat(cmdstr, "\r");
	write(galilfd, cmdstr, strlen(cmdstr));
	memset(buf, 0, n);
	read (galilfd, buf, n);

}

/*-------------------------------------------------------------------

	askGalilForInt(cmd);

	askGalilForInt sends the command string (cmd) to the Galil
	controller and returns the Galil reply as an integer.

	cmd is a pointer to a NUL terminated string containing a
	Galil command that returns an integer (as do most commands).

-------------------------------------------------------------------*/

int askGalilForInt(cmd)
char *cmd;
{

	char buf[80];

	askGalil(cmd, buf, 80);
	if (debugFlag) {
		printf("asking %s\n",cmd);
	}
	return(atoi(buf));

}

/*-------------------------------------------------------------------

	askGalilForLong(cmd);

	askGalilForLong sends the command string (cmd) to the Galil
	controller and returns the Galil reply as a long integer.

	cmd is a pointer to a NUL terminated string containing a
	Galil command that returns an integer (as do most commands).

-------------------------------------------------------------------*/

long int askGalilForLong(cmd)
char *cmd;
{

	char buf[80];

	askGalil(cmd, buf, 80);
	return(atol(buf));

}

/*-------------------------------------------------------------------

	int limitSwitch(axis);

	limitSwitch returns an integer with bits 0 and 1 indicating
	the reverse and forward limit switch states respectively.
	If the limit is engaged, the bit is 1 and motion is diabled
	in that direction. If the limit is not engaged the bit is 0.

-------------------------------------------------------------------*/
int limitSwitch(axis)
{

	int temp, limitVal;

	switch (axis) {

		case XAXIS:
			temp = askGalilForInt("TSA");
			break;

		case YAXIS:
			temp = askGalilForInt("TSB");
			break;

		case ZAXIS:
			temp = askGalilForInt("TSC");
			break;

		default:
			printf("askLimits(%d) was sent a bad axis value\n", axis);
			return(UNKNOWN);
	}


	limitVal = 0;
	if ((temp>>2 & 0x01) == 0) {			// Reverse limit
		limitVal |= 0x01;
	}
	if ((temp>>3 & 0x01) == 0) {			// Forward limit
		limitVal |= 0x02;
	}
	return(limitVal);

}

/*-------------------------------------------------------------------

	backOff();

	backOff clears a limit switch situation by searching for
	engaged limits and backing away a few steps.

-------------------------------------------------------------------*/
void backOff()
{

	int testVal;

	if (testVal = limitSwitch(XAXIS)) {
		if (testVal & 0x01) {
			moveOneAxis(XAXIS, 500, 500);
			motorPower(XAXIS, OFF);
		} else if (testVal & 0x02) {
			moveOneAxis(XAXIS, -500, 500);
			motorPower(XAXIS, OFF);
		}
	}

	if (testVal = limitSwitch(YAXIS)) {
		if (testVal & 0x01) {
			moveOneAxis(YAXIS, 500, 500);
			motorPower(YAXIS, OFF);
		} else if (testVal & 0x02) {
			moveOneAxis(YAXIS, -500, 500);
			motorPower(YAXIS, OFF);
		}
	}
	if (testVal = limitSwitch(ZAXIS)) {
		if (testVal & 0x01) {
			moveOneAxis(ZAXIS, 200, 500);
			motorPower(ZAXIS, OFF);
		} else if (testVal & 0x02) {
			moveOneAxis(ZAXIS, -200, 500);
			motorPower(ZAXIS, OFF);
		}
	}

}	

/*-------------------------------------------------------------------

	brake(axis, onOffStatus);

	brake(axis, onOffStatus) sets or releases a motor brake or
	returns the current brake state for the selected axis.  Only
	the X and Y axes on the Magellan AO guider have brakes, so
	axis can only be one of [XAXIS|YAXIS]. onOffStatus can be
	[ON|OFF|STATUS]. Values returned may be [ON|OFF|UNKNOWN]

-------------------------------------------------------------------*/
int brake(axis, onOffStatus)
{

	int testVal;

	if (onOffStatus == STATUS) {
		switch (axis) {
			case XAXIS:
				if (askGalilForInt("MG@OUT[1]")) {
					return(OFF);
				} else {
					return(ON);
				}

			case YAXIS:
				if (askGalilForInt("MG@OUT[2]")) {
					return(OFF);
				} else {
					return(ON);
				}

			case ZAXIS:
				return(UNKNOWN);

			default:
				printf("brake() was passed a bad axis (%d)\n", axis);
				return(UNKNOWN);
		}
	}

	if (onOffStatus == ON) {
		switch (axis) {
			case XAXIS:
				usleep(50000);
				tellGalil("CB1");
				if (askGalilForInt("MG@OUT[1]") == 0) {
					return(ON);
				} else {
					return(UNKNOWN);
				}
			case YAXIS:
				usleep(500000);
				tellGalil("CB2");
				if (askGalilForInt("MG@OUT[2]") == 0) {
					return(ON);
				} else {
					return(UNKNOWN);
				}
			default:
				printf("brake(%d, %d) was passed a bad axis for brake ON\n", axis, onOffStatus);
				return(UNKNOWN);
		}
	} else if (onOffStatus == OFF) {
		usleep(25000);
		switch (axis) {
			case XAXIS:
				tellGalil("SB1");
				usleep(50000);
				if (askGalilForInt("MG@OUT[1]")) {
					return(OFF);
				} else {
					return(UNKNOWN);
				}
			case YAXIS:
				tellGalil("SB2");
				usleep(50000);
				if (askGalilForInt("MG@OUT[2]")) {
					return(OFF);
				} else {
					return(UNKNOWN);
				}
			default:
				printf("brake(%d, OFF) was sent bad axis value\n", axis);
				return(UNKNOWN);
		}
	} else {
		printf("brake(%d,%d) was sent a bad onOffStatus option\n", axis, onOffStatus);
		return(UNKNOWN);
	}
}


/*-------------------------------------------------------------------

	calibrate()
	
	This routine calibrates the motor steps to encoder pulses scale.
	
	It first calls the homeAxes routine to re-home the stepper
	motors against the reverse limit switches. It then runs the
	stage to the forward limit switches and computes the ratio
	of the number of encoder pulses to the number of motor steps
	in that long run. This ratio is almost always not exactly right
	since (we believe) the steppers occasionally miss a step or two.
	
	This routine must be called before any absolute position move.

-------------------------------------------------------------------*/
int calibrate()
{

	printf("calibrating; ");
	fflush(stdout);

	homeAxes();

	// get encoder position at (0,0)
	printf("xEncOffset = %ld", xEncOffset);
	printf("yEncOffset = %ld", yEncOffset);

// go to the reverse limit
	moveOneAxis(YAXIS, -20000, 1500);
	motorPower(YAXIS, OFF);
	moveOneAxis(XAXIS, -23000, 1500);
	motorPower(XAXIS, OFF);

// back off hysteresis
	moveOneAxis(XAXIS, LIMITHYSTER, 1500);
	motorPower(XAXIS, OFF);
	moveOneAxis(YAXIS, LIMITHYSTER, 1500);
	motorPower(YAXIS, OFF);

// find the edge
	creepToLimits(XAXIS, 1, 20);
	creepToLimits(YAXIS, 1, 20);

// At reverse limits
	printf("xSteps at reverse limit = %d\n", stepPosition(XAXIS));
	printf("ySteps at reverse limit = %d\n", stepPosition(YAXIS));

// Set global xEncPerStep, yEncPerStep
	xEncPerStep = (float) (encPosition(XAXIS) - xEncOffset) / (float) stepPosition(XAXIS);
	yEncPerStep = (float) (encPosition(YAXIS) - yEncOffset) / (float) stepPosition(YAXIS);
	printf("xEncPerStep = %7.5f\n", xEncPerStep);
	printf("yEncPerStep = %7.5f\n", yEncPerStep);

	moveOneAxis(XAXIS, XSTEPSPERTURN/2, 1500);		// Half turn back
	moveOneAxis(YAXIS, YSTEPSPERTURN/2, 1500);

// Set global xEncMin, yEncMin
	xEncMin = encPosition(XAXIS);
	yEncMin = encPosition(YAXIS);

	printf("X-Y min values: %ld %ld\n", xEncMin, yEncMin);
	printf("X-Y max values: %7.3f %7.3f (mm)\n", mmPosition(XAXIS), mmPosition(YAXIS));
	fflush(stdout);

	isCalibrated = 1;

	return(0);

}

/*-------------------------------------------------------------------

	cmdLoop()
	
	Polls the keyboard for a one-key command.
	
-------------------------------------------------------------------*/
void cmdLoop()
{

	int cmd;

	printf("> ");			// Prompt character on the terminal
	fflush(stdout);

	setMode(NONBLOCKING);
	while (!(cmd = getKey())) {	// Wait for a command
		usleep(10000);
	}

	if (cmd == 'a') {		// Insert the small aperture
		smallAp();
	} else if (cmd == 'A') {	// Insert the field lens
		fieldLens();
	} else if (cmd == 'B') {	// Back off limits
		backOff();
	} else if (cmd == 'C') {	// Calibrate
		calibrate();
	} else if (cmd == 'D') {	// Demo mode 
		demo();
	} else if (cmd == 'd') {	// Toggle debug flag
		debug();
	} else if (cmd == 'H') {	// Home the X and Y axes
		homeAxes();
	} else if (cmd == 'h') {	// list commands help
		help();
	} else if (cmd == 'F') {	// Focus to absolute position
		focus(FOCUSABS);
	} else if (cmd == 'f') {	// focus relative
		focus(FOCUSREL);
	} else if (cmd == 'i') {	// Initialize
		initGuider();
	} else if (cmd == 'l') {	// Set up LED with S-H
		ledInOut();
	} else if (cmd == 'm') {	// relative position move
		move(RELATIVE);
	} else if (cmd == 'M') {	// absolute position move
		move(ABSOLUTE);
	} else if (cmd == 'q') {	// quit
		exit(0);
	} else if (cmd == 'R') {	// Reset
		resetGalil();
	} else if (cmd == 'S') {	// Self check
		selfCheck();
	} else if (cmd == 's') {	// Set up for Shack-Hartmann
		shCam();
	} else if (cmd == 'T') {	// Run the Test function
		testFunction();
	} else if (cmd == 'w') {	// Set up for wide field viewing
		fieldCam();
	} else if (cmd == '?') {	// status
		statusPrint();
	} else if (cmd == ':') {	// Talk directly to the Galil
		passthru();
//	} else if (cmd == '$') {
//		fakeHome();		// testing home function (REMOVE LATER)
	} else {
		printf("%c? Type \"h\" for command list help\n", cmd);
		fflush(stdout);
	}

}


/*-------------------------------------------------------------------

	creepToLimits(axis, steps, speed);

	creepToLimits moves the selected motor axis in increments of
	"steps" at the selected "speed" until that axis's limit switch
	changes state. Note that the limit switch may alread be engaged
	so a change of state could be from engaged to not engaged.

	Pay attention the step direction since you don't want to travel
	the full length of the stage at slow speed.

-------------------------------------------------------------------*/
long int creepToLimits(axis, steps, speed)
int axis, steps, speed;
{

	char axischar;
	int i, oldLimits;

	switch (axis) {

		case XAXIS:
			axischar = 'A';
			i = 0;
			oldLimits = limitSwitch(XAXIS);
			while (oldLimits == limitSwitch(XAXIS) && i < 200) {	// wait until the switch changes state
				moveOneAxis(XAXIS, steps, speed);
				i++;
			}
			motorPower(XAXIS, OFF);
			break;

		case YAXIS:
			axischar = 'B';
			i = 0;
			oldLimits = limitSwitch(YAXIS);
			while (oldLimits == limitSwitch(YAXIS) && i < 200) {
				moveOneAxis(YAXIS, steps, speed);
				i++;
			}
			motorPower(YAXIS, OFF);
			break;

		case ZAXIS:
			axischar = 'C';
			i = 0;
			oldLimits = limitSwitch(ZAXIS);
			while (oldLimits == limitSwitch(ZAXIS) && i < 200) {
				moveOneAxis(ZAXIS, steps, speed);
				i++;
			}
			motorPower(ZAXIS, ON);
			break;

		default:
			printf("creepToLimits(%d, STEPS, SPEED) was sent a bad axis value\n", axis);
			return(UNKNOWN);

	}
}

/*-------------------------------------------------------------------

	cylinder(axis, exRetStatus)
	
	cylinder sets or returns information on the three pneumatic
	cylinders. "axis" should be [Y1AXIS|Y2AXIS|SAXIS] and 
	exRetStatus" should be one of [EXTEND|RETRACT|STATUS].
	
	EXTEND and RETRACT cause that action to be performed. STATUS
	returns one of EXTEND, RETRACT, or UNKNOWN.

-------------------------------------------------------------------*/
int cylinder(axis, extRetStatus)
{

	int i, status, y1e, y1r, y2e, y2r;
	static int sAxisStatus = UNKNOWN;


	if (extRetStatus == STATUS) {

		status = ~askGalilForInt("TI0");
		y1e = ((status>>4) & 0x01);
		y1r = ((status>>5) & 0x01);
		y2e = ((status>>2) & 0x01);
		y2r = ((status>>3) & 0x01);
		if (debugFlag) {
			printf("y1e,y1r,y2e,y2r = %d,%d,%d,%d\n", y1e,y1r,y2e,y2r);
		}
		if ((y1e == y1r) || (y2e == y2r)) {
			printf("cylinder(%d,STATUS) error: both sensors the same\n", axis);
			return(UNKNOWN);
		}

		switch(axis) {

			case Y1AXIS:
				if (y1e) {
					return(EXTEND);
				} else {
					return(RETRACT);
				}

			case Y2AXIS:
				if (y2e) {
					return(EXTEND);
				} else {
					return(RETRACT);
				}

			case SAXIS:
				return(sAxisStatus);

			default:
				printf("cylinder(%d,STATUS) was passed a bad axis value\n", axis);
				return(UNKNOWN);

		}

	} else if (extRetStatus == RETRACT) {
		switch(axis) {
			case Y1AXIS:
				tellGalil("CB7;SB8");
				for (i = 0; i < 50; i++) {
					usleep(100000);
					status = ~askGalilForInt("TI0");
					y1e = ((status>>4) & 0x01);
					y1r = ((status>>5) & 0x01);
					if (y1r && !y1e) {
						return(RETRACT);
					}
				}
				printf("cylinder(Y1AXIS,RETRACT) timeout\n");
				return(UNKNOWN);

			case Y2AXIS:
				tellGalil("SB5;CB6");
				for (i = 0; i < 50; i++) {
					usleep(100000);
					status = ~askGalilForInt("TI0");
					y2e = ((status>>2) & 0x01);
					y2r = ((status>>3) & 0x01);
					if (y2r && !y2e) {
						return(RETRACT);
					}
				}
				printf("cylinder(Y2AXIS,RETRACT) timeout\n");
				return(UNKNOWN);

			case SAXIS:
				tellGalil("CB3");
				sAxisStatus = RETRACT;
				sleep(1);
				return(RETRACT);

			default:
				printf("cylinder(%d,RETRACT) was passed a bad axis\n", axis);
				return(UNKNOWN);
		}

	} else if (extRetStatus == EXTEND) {

		switch(axis) {

			case Y1AXIS:
				tellGalil("SB7;CB8");
				for (i = 0; i < 50; i++) {
					usleep(100000);
					status = ~askGalilForInt("TI0");
					y1e = ((status>>4) & 0x01);
					y1r = ((status>>5) & 0x01);
					if (y1e && !y1r) {
						return(EXTEND);
					}
				}
				if (debugFlag) {
					printf("y1e = %d, y1r = %d\n", y1e,y1r);
				}
				printf("cylinder(Y2AXIS,EXTEND) timeout\n");
				return(UNKNOWN);

			case Y2AXIS:
				tellGalil("CB5;SB6");
				for (i = 0; i < 50; i++) {
					usleep(100000);
					status = ~askGalilForInt("TI0");
					y2e = ((status>>2) & 0x01);
					y2r = ((status>>3) & 0x01);
					if (y2e && !y2r) {
						return(EXTEND);
					}
				}
				printf("cylinder(Y2AXIS,EXTEND) timeout\n");
				return(UNKNOWN);

			case SAXIS:
				tellGalil("SB3");
				sAxisStatus = EXTEND;
				sleep(1);
				return(EXTEND);

			default:
				printf("cylinder(%d,EXTEND) was passed a bad axis\n", axis, extRetStatus);
				return(UNKNOWN);
		} 
	} else {
		printf("cylinder(%d,%d) was passed a bad extRetStatus\n", axis, extRetStatus);
		return(UNKNOWN);
	}
}

/*-------------------------------------------------------------------

	debug()
	
	Turns the global variable debugFlag on or off.

-------------------------------------------------------------------*/
void debug()
{

	if (debugFlag) {
		debugFlag = 0;
		printf("debug off\n");
	} else {
		debugFlag = 1;
		printf("debug on\n");
	}

}

/*-------------------------------------------------------------------

	demo()
	
	Uses the rand() function to select random postions for the
	three stepper motors and random states for the various pneumatic
	cylinder positions.

-------------------------------------------------------------------*/
void demo()
{

	char buf[80];
	float x, y, z, randomNum;

	if (! isCalibrated) {
		printf("Calibrate first\n");
		return;
	}

	randomNum = (float) rand() / (float) RAND_MAX;
	x = 250.0 * randomNum;
	randomNum = (float) rand() / (float) RAND_MAX;
	y = 170.0 * randomNum;
	printf("Moving to %lf %lf\n", x, y);
	moveAbs(x,y);

	randomNum = (float) rand() / (float) RAND_MAX;
	if (randomNum > 0.5) {
		cylinder(SAXIS, IN);
		led(ON);
		printf("LED on and in\n");
	} else {
		led(OFF);
		cylinder(SAXIS, OUT);
		printf("LED off and out\n");
	}

	focusRel(z);

	randomNum = (float) rand() / (float) RAND_MAX;
	if (randomNum > 0.5) {
		smallAp();
		printf("small aperture in\n");
	} else {
		fieldLens();
		printf("field lens in\n");
	}

	randomNum = (float) rand() / (float) RAND_MAX;
	if (randomNum > 0.5) {
		shCam();
		printf("Lenslets in\n");
	} else {
		fieldCam();
		printf("wide field camera in\n");
	}
	
	fflush(stdout);
}

/*-------------------------------------------------------------------

	encPosition(axis)
	
	encPosition returns the current encoder position of the selected
	axis. "axis" may be either XAXIS or YAXIS.

-------------------------------------------------------------------*/
long int encPosition(axis)
int axis;
{

	switch (axis) {

		case XAXIS:
			return(askGalilForLong("TPA"));
		case YAXIS:
			return(askGalilForLong("TPB"));
		default:
			printf("encPosition(%d) was passed a bad axis value\n", axis);
			return(UNKNOWN);
	}
}

/*-------------------------------------------------------------------

	mmPosition(axis)
	
	mmPosition returns the current position in mm away from the home
	position of the selected axis. "axis" may be either XAXIS or
	YAXIS. This position is computed from the manufacturer's specs
	on the pitch of the lead screw and the number of pulses per turn
	sent by the encoder.

-------------------------------------------------------------------*/
float mmPosition(axis)
int axis;
{

	switch(axis) {
		case XAXIS:
			return((float) ((xEncOffset - encPosition(XAXIS)) * XSCREWPITCH) / (float) (XENCPULSPERTURN));

		case YAXIS:
			return((float) ((yEncOffset - encPosition(YAXIS)) * YSCREWPITCH) / (float) (YENCPULSPERTURN));

		default:
			printf("floatToMm(%d) was passed a bad axis\n", axis);
			return(-999.0);
	}
}

/*-------------------------------------------------------------------

	errmsg(errnum)
	
	errmsg prints an error message. Most error messages are printed
	inside the relevant routines, though.

-------------------------------------------------------------------*/
void errmsg(errnum)
int errnum;
{

	switch (errnum) {
		case LEDERROR:
			printf("led error\n");
			fflush(stdout);
			break;
		case CONNECTERROR:
			printf("connect() failed to %s\n", GALILIP);
			exit(0);
		case SOCKETERROR:
			printf("socket() failed\n");
			exit(0);
		case INETPTONERROR:
			printf("inet_pton() failed\n");
			exit(0);
		case BADMOVE:
			printf("Move command not ABSOLUTE or RELATIVE\n");
			exit(0);
		case NOTHOMED:
			printf("Motors not homed\n");
			return;
		default:
			return;
	}
}

/*-------------------------------------------------------------------

	fieldCam()
	
	fieldCam retracts the Y1 cylinder to put Derek Kopon's wide field
	camera into the beam ahead of the CCD.

-------------------------------------------------------------------*/
int fieldCam()
{

	int status;

	printf("field camera ");
	fflush(stdout);
	status = cylinder(Y1AXIS, RETRACT);
	if (status != RETRACT) {
		printf("error: Galil reports Y1AXIS motion error\n");
		return(UNKNOWN);
	} else {
		printf("in\n");
		fflush(stdout);
		return(IN);
	}

}

/*-------------------------------------------------------------------

	focus(type)
	
	focus sets the focus stage position. "type" may be either
	FOCUSREL or FOCUSABS for relative or absolute motion, both
	in motor steps (change this to mm when we get the focus motor
	installed).

-------------------------------------------------------------------*/
void focus(type)
int type;
{

	char buf[20];
	long int x, currentFocus;

	if (type == FOCUSREL) {
		printf("Focus offset: ");
		fflush(stdout);
		x = atol(gets(buf));
	} else if (type == FOCUSABS) {
		if (homeTime != askGalilForLong("MG homeTime")) {			// USE ISHOMED INSTEAD
			errmsg(NOTHOMED);
			return;
		}
		printf("New focus value: ");
		x = atol(gets(buf)) - askGalilForLong("RPC");
	}

	focusRel(x);

}

/*-------------------------------------------------------------------

	focusRel(x)
	
	focusRel moves the focus motor by x steps, positive or negative.
	Re-do this after the focus motor is installed.

-------------------------------------------------------------------*/
void focusRel(x)
long int x;
{

	char buf[80];
	int i, keypress, motorState;

	tellGalil("SP 0,0,1024");			// Focus motor speed
	tellGalil("AC 1024,1024,256000");		// Acceleration
	tellGalil("DC 1024,1024,256000");		// Deceleration

	if (x == 0) {
		printf("No motion\n");
		return;
	} else {
		sprintf(buf, "PR 0,0,%ld", x);
		tellGalil(buf);
	}
	tellGalil("SHC");				// Focus motor on
	tellGalil("BGC");				// Focus motion only
	i = 0;
	setMode(NONBLOCKING);
	while (!(keypress = getKey())) {
		motorState = askGalilForInt("TSC");
		if (((motorState>>7) & (0x01)) != 0) {		// Motor still moving
			usleep(10000);
			i++;
		} else {
			if (debugFlag) {
				printf("%d loops in focus motion\n", i);
			}
			tellGalil("MOC");
			return;
		}
	}

	// Keyboard stop
	printf("Focus motion stopped by user\n");
	tellGalil("STC");
	motorState = askGalilForInt("TSC");
	while (((motorState>>7) & (0x01)) != 0) {
		usleep(10000);
		motorState = askGalilForInt("TSC");
	}
	tellGalil("MOC");
	return;
}

/*-------------------------------------------------------------------

	getLimits(axis)
	
	getLimits returns the state of the limit switches on the selected
	axis. "axis" may be [XAXIS|YAXIS|ZAXIS].
	
	RETIRED; replaced by limitSwitch
	
int getLimits(axis)
int axis;
{

	int motorState, forward, reverse;

	forward = reverse = 0;
	switch (axis) {
		case XAXIS:
			motorState = askGalilForInt("TSA");
			break;
		case YAXIS:
			motorState = askGalilForInt("TSB");
			break;
		case ZAXIS:
			motorState = askGalilForInt("TSC");
			break;
		default:
			printf("getLimits() was passed illegal axis\n");
			return(-1);
	}

	forward = ((motorState >> 3) & 0x01);
	reverse = ((motorState >> 2) & 0x01);

	if (forward == 0 && reverse == 0) {	// Neither switch active
		return(0);
	} else if (forward == 1 && reverse == 1) {
		printf("getLimits() Error: both limit switches active\n");
		return(-1);
	} else if (reverse) {
		return(reverse);
	} else {
		return(forward);
	}
}
-------------------------------------------------------------------*/




/*-------------------------------------------------------------------

	getKey()
	
	getKey returns a keyboard character hit (from stdin).

-------------------------------------------------------------------*/
int getKey()
{

	int c = 0;
	struct timeval tv;
	fd_set fs;
	tv.tv_usec = tv.tv_sec = 0;

	FD_ZERO(&fs);
	FD_SET(STDIN_FILENO, &fs);
	select(STDIN_FILENO + 1, &fs, 0, 0, &tv);
	if (FD_ISSET(STDIN_FILENO, &fs)) {
		c = getchar();
		setMode(BLOCKING);
	}
	return c;

}

/*-------------------------------------------------------------------

	help()
	
	prints a list of commands. Coordinate with cmdLoop().

-------------------------------------------------------------------*/
int help()
{

	printf("commands:\n\n");
	printf("\tB - Back off limits\n");
	printf("\tC - Calibrate\n");
	printf("\tD - Demo routine\n");
	printf("\td - toggle diagnostic prints (debug)\n");
	printf("\tF - Focus, absolute position\n");
	printf("\tf - focus, relative motion\n");
	printf("\tH - Home the axes\n");
	printf("\th - this help listing\n");
	printf("\ti - initialize\n");
	printf("\tl - led (i)n | (o)ut\n");
	printf("\tm - move relative\n");
	printf("\tM - Move absolute\n");
	printf("\tq - quit\n");
	printf("\tR - Reset Galil\n");
	printf("\ts - Shack-Hartmann lenslets in\n");
	printf("\tT - Test function execute\n");
	printf("\tw - wide field camera in\n");
	printf("\t? - print status\n");
	printf("\t: - send commands directly to Galil\n");
	printf("\t$ - fake homing (unplug limit switches and motor power first)\n");
	fflush(stdout);

}


/*-------------------------------------------------------------------

	fakeHome()
	
	Set up the controller and this software to believe that a homing
	operation occurred. Used this for testing non-motion operations
	that needed the system in a homed state.

	RETIRED?

void fakeHome()
{

	tellGalil("homeTime=TIME");
	homeTime = askGalilForLong("MG homeTime");
	tellGalil("CN-1");
	tellGalil("DP 0,0,0");
	if (debugFlag) {
		printf("homeTime = %ld\n", homeTime);
	}
	printf("Faked a homing operation\n");

}
-------------------------------------------------------------------*/


/*-------------------------------------------------------------------

	fieldLens()
	
	fieldLens retracts the Y2 cylinder to put the field lens into the
	camera beam. The field lens alternates with the small aperture.

-------------------------------------------------------------------*/
int fieldLens()
{

	int status;

	printf("field lens ");
	fflush(stdout);
	status = cylinder(Y2AXIS, RETRACT);
	if (status != RETRACT) {
		printf("error: Galil reports field lens insertion (Y2AXIS) error\n");
		return(UNKNOWN);
	} else {
		printf("field lens in\n");
		fflush(stdout);
		return(IN);
	}
}

/*-------------------------------------------------------------------

	homeAxes()
	
	homeAxes drives the stage to the reverse limits, finds the last
	point where the limit switch still allows motion, then moves
	half a motor turn back from there to define the home position.
	
	The  motor positions are zeroed out and the X-Y encoder positions
	are noted (they cannot be zeroed out).

-------------------------------------------------------------------*/
int homeAxes()
{

	long int position;

	if (limitSwitch(XAXIS) + limitSwitch(YAXIS) + limitSwitch(ZAXIS)) {
		printf("homeAxes() on a limit already; backing off\n");
		backOff();
	}

	printf("homing");
	fflush(stdout);
	creepToLimits(XAXIS, 15000, 1500);
	printf(".");
	fflush(stdout);
	creepToLimits(YAXIS, 15000, 1500);
	printf(".");
	fflush(stdout);
//	creepToLimits(ZAXIS, 15000, 500);
	printf(".");
	fflush(stdout);

	moveOneAxis(XAXIS, -(LIMITHYSTER), 50);
	motorPower(XAXIS, OFF);
	moveOneAxis(YAXIS, -(LIMITHYSTER), 50);
	motorPower(YAXIS, OFF);
//	moveOneAxis(ZAXIS, -(LIMITHYSTER), 20);
//	motorPower(ZAXIS, OFF);
	printf(".");
	fflush(stdout);

	if (limitSwitch(XAXIS) == 0) {
		printf("homeAxes() error in X hysteresis value\n");
		return(UNKNOWN);
	}
	creepToLimits(XAXIS, -1, 20); 

	if (limitSwitch(YAXIS) == 0) {
		printf("homeAxes() error in Y hysteresis value\n");
		return(UNKNOWN);
	}
	creepToLimits(YAXIS, -1, 20);

	if (limitSwitch(ZAXIS) == 0) {
//		printf("homeAxes() error in Z hysteresis value\n");
//		return(UNKNOWN);
	}
//	creepToLimits(ZAXIS, -1, 20);

	moveOneAxis(XAXIS, -250, 1500);		// half-turn more
	motorPower(XAXIS, OFF);
	moveOneAxis(YAXIS, -250, 1500);
	motorPower(YAXIS, OFF);
	tellGalil("DP 0,0,0");
	xEncOffset = askGalilForLong("TPA");
	yEncOffset = askGalilForLong("TPB");
	tellGalil("homeTime=TIME");
	homeTime = askGalilForLong("MG homeTime");
	if (debugFlag) {
		printf("homeTime = %ld", homeTime);
	}
	printf(".done\n");
	fflush(stdout);

}

/*-------------------------------------------------------------------

	initGuider()
	
	initGuider sets up the guider so the pneumatic actuators are
	all retracted, motor brakes are turned on, and various Galil
	parameters are set to reasonable values.
	


-------------------------------------------------------------------*/
void initGuider()
{

	printf("initializing ");
	fflush(stdout);
	brake(XAXIS, ON);
	brake(YAXIS, ON);
	tellGalil("MT 2,2,2");					// Tell Galil they're stepper motors
	motorPower(XAXIS, OFF);
	motorPower(YAXIS, OFF);
	motorPower(ZAXIS, OFF);
	tellGalil("CB4");						// LED off (CHANGE TO USE LED COMMAND HERE)
	tellGalil("CN1");						// Limit switch configuration (or is it -1?)

	cylinder(Y1AXIS, RETRACT);
	cylinder(Y2AXIS, RETRACT);
	cylinder(SAXIS, RETRACT);

	tellGalil("SP 200,200,200");			// Slow speed
	tellGalil("AC 256000,256000,256000");	// These are defaults
	tellGalil("DC 256000,256000,256000");
	tellGalil("SD 256000,256000,256000");	// Deceleration after hitting a limit switch
	tellGalil("VS 200");					// Slow vector speed
	tellGalil("VA 256000");					// Default vector values
	tellGalil("VD 256000");
	tellGalil("KS 5,5,5");					// Step motor smoothing
	tellGalil("CAS");						// S coordinate system for vector motion
	printf("done\n");

}


int isHomed()
{

	if (homeTime != askGalilForLong("MG homeTime")) {
		return(0);
	} else {
		return(1);
	}
}

int isMoving(axis)
{

	int status;
	char buf[20];

	switch(axis) {
		case XAXIS:
			strcpy(buf, "TSA");
			break;
		case YAXIS:
			strcpy(buf, "TSB");
			break;
		case ZAXIS:
			strcpy(buf, "TSC");
			break;
		default:
			printf("isMoving(%d) error: passed bad axis value\n", axis);
			return(UNKNOWN);
	}
	status = askGalilForInt(buf);
	return((status>>7) & 0x01);
}

int led(onOffStatus)
{

	static int status = UNKNOWN;

	switch (onOffStatus) {
		case ON:
			tellGalil("SB4");
			if (askGalilForInt("MG@OUT[4]") == 0) {
				printf("led() requested ON, Galil reports OFF\n");
				return(UNKNOWN);
			} else {
				status = ON;
				return(ON);
			}
		case OFF:
			tellGalil("CB4");
			if (askGalilForInt("MG@OUT[4]") != 0) {
				printf("Led() requested off, Galil reports ON\n");
				return(UNKNOWN);
			} else {
				status = OFF;
				return(OFF);
			}
		case STATUS:
			if (askGalilForInt("MG@OUT[4]") == 0) {
				return(OFF);
			} else {
				return(ON);
			}
		default:
			printf("led(%d) bad onOffStatus value\n", onOffStatus);
			status = UNKNOWN;
			return(UNKNOWN);
	}
}

int ledInOut()
{

	int cmd;

	printf("led (i=in, o=out): ");
	fflush(stdout);
	setMode(NONBLOCKING);
	while (!(cmd = getKey())) {
		usleep(10000);
	}
	cmd = tolower(cmd);

	if (cmd == 'i') {
		cylinder(SAXIS, EXTEND);	// LED into the beam
		led(ON);
		printf("led in and on\n");
		fflush(stdout);
		return(1);
	} else if (cmd == 'o') {
		led(OFF);
		cylinder(SAXIS, RETRACT);	// LED into the beam
		printf("led out and off\n");
		fflush(stdout);
		return (1);
	} else {
		printf("ledInOut() bad choice; try again\n");
		return (0);
	}

}

int motorPower(axis, onOffStatus)
{

	char buf[20], axischar;

	switch (axis) {

		case XAXIS:
			axischar = 'A';
			break;

		case YAXIS:
			axischar = 'B';
			break;

		case ZAXIS:
			axischar = 'C';
			break;

		default:
			printf("motorPower(%d, %d) passed a bad axis value\n", axis, onOffStatus);
			return(UNKNOWN);
	}	

	if (onOffStatus == ON) {
		sprintf(buf, "SH%c", axischar);
		tellGalil(buf);
		if (axis != ZAXIS) {
			brake(axis, OFF);
		}
		return(ON);

	} else if (onOffStatus == OFF) {
		if (axis != ZAXIS) {
			brake(axis, ON);
		}
		sprintf(buf, "MO%c", axischar);
		tellGalil(buf);
		return(OFF);

	} else if (onOffStatus == STATUS) {
		sprintf(buf, "TS%c", axischar);
		if ((askGalilForInt(buf) >> 5) & 0x01) {
			return(OFF);
		} else {
			return(ON);
		}

	} else {
		printf("motorPower(AXIS, %d) passed a bad onOffStatus value\n", onOffStatus);
	}
}


/*-------------------------------------------------------------------

	move(type)

	Moves the X-Y stage. type is [ABSOLUTE|RELATIVE]. For
	absolute moves, the user is requested for the position in
	mm.

-------------------------------------------------------------------*/
void move(type)
int type;
{

	char buf[80];
	long int x, y, currentX, currentY, xoff, yoff;

	if (type == ABSOLUTE) {
		if (!isHomed()) {
			errmsg(NOTHOMED);
			return;
		}
/*
		currentX = stepPosition(XAXIS);
		currentY = stepPosition(YAXIS);
*/
		printf("Absolute postion X-Y move\n");
		printf("New X position (mm): ");
		fflush(stdout);
		x = atof(gets(buf));
		printf("New Y position (mm): ");
		fflush(stdout);
		y = atof(gets(buf));
		moveAbs(x,y);
		return;
/*
		// Linear interpolation works only for offsets; compute offsets
		xoff = x - currentX;
		yoff = y - currentY;
*/

	} else if (type == RELATIVE) {
		printf("Relative position X-Y move\n");
		printf("X offset (steps): ");
		fflush(stdout);
		xoff = atol(gets(buf));
		printf("Y offset (steps): ");
		fflush(stdout);
		yoff = atol(gets(buf));
		if (xoff == 0 && yoff == 0) {
			printf("No motion\n");
			return;
		}
		if (debugFlag) {
			printf("X, Y offsets: %ld %ld\n", xoff, yoff);
		}
		moveRel(xoff,yoff);
		return;
	} else {
		errmsg(BADMOVE);
	}
}

/*-------------------------------------------------------------------

	moveAbs(x,y)

	moveAbs positions the guider stage at coordinates (x,y) where
	x and y are in mm.

	The coordinates are positive numbers referenced to the
	upper right hand corner of the guider patrol space (the
	home position).


	A Calibrate operation must be done before this command.

-------------------------------------------------------------------*/
void moveAbs(x, y)
float x, y;
{

	long xEncOld, yEncOld, xEncNew, yEncNew, xSteps, ySteps, temp;
	float xPulsPerStep, yPulsPerStep;

	printf("x,y (mm) = %7.3f %7.3f\n", x, y);

	xPulsPerStep = (float) XENCPULSPERTURN / (float) XSTEPSPERTURN;
	yPulsPerStep = (float) YENCPULSPERTURN / (float) YSTEPSPERTURN;

	// current encoder position
	xEncOld = encPosition(XAXIS);
	yEncOld = encPosition(YAXIS);

	if (debugFlag) {
		printf("Current encoder positions %d %d\n", xEncOld, yEncOld);
	}

	// target encoder position
	xEncNew = xEncOffset - (long int) (x * ((float) XENCPULSPERTURN)/XSCREWPITCH);
	yEncNew = yEncOffset - (long int) (y * ((float) XENCPULSPERTURN)/XSCREWPITCH);

	if (debugFlag) {
		printf("New encoder positions %d %d\n", xEncNew, yEncNew);
	}
	if ((xEncNew > xEncOffset) || (xEncNew < xEncMin)) {
		printf("xEncNew out of range (%ld)\n", xEncNew);
		return;
	}
	if ((yEncNew > yEncOffset) || (yEncNew < yEncMin)) {
		printf("yEncNew out of range (%ld)\n", yEncNew);
		return;
	}

	// compute motor steps
	temp = (xEncNew - xEncOld);
	xSteps = (temp >= 0) ? (long int) (((double) (temp + 0.5)) / xPulsPerStep) : (long int) (((double) (temp - 0.5)) / xPulsPerStep);
	temp = (yEncNew - yEncOld);
	ySteps = (temp >= 0) ? (long int) (((double) (temp + 0.5)) / yPulsPerStep) : (long int) (((double) (temp - 0.5)) / yPulsPerStep);

	printf("Motor steps %d %d\n", xSteps, ySteps);
	fflush(stdout);

	moveRel(xSteps, ySteps);

}

/*-------------------------------------------------------------------

	moveOneAxis(axis,steps,speed)

	moveOneAxis commands a single-axis motion with the selected
	number of motor steps and speed. Acceleration and deceleration
	values are defaults.

	IMPORTANT NOTE: moveOneAxis turns on the motor power and
	releases the brakes (if any) but does not turn off the motor
	or set the brakes. This must be done after with a call to
	motorPower(axis, onOffStatus).

-------------------------------------------------------------------*/
int moveOneAxis(axis, steps, speed)
int axis, steps, speed;
{

	char axischar, buf[20];
	int testVal;
	long int acceleration, deceleration;

	acceleration = DEFACCEL;
	deceleration = DEFDECEL;

	switch (axis) {
		case XAXIS:
			axischar = 'A';
			motorPower(XAXIS, ON);
			break;

		case YAXIS:
			axischar = 'B';
			motorPower(YAXIS, ON);
			break;

		case ZAXIS:
			axischar = 'C';
			motorPower(ZAXIS, ON);
			break;

		default:
			printf("moveOneAxis(%d,%d,%d) received an illegal axis\n", axis, steps, speed);
			return(-1);
	}

	sprintf(buf, "SP%c=%d", axischar, speed);
	tellGalil(buf);
	sprintf(buf, "AC%c=%ld", axischar, acceleration);
	tellGalil(buf);
	sprintf(buf, "DC%c=%ld", axischar, deceleration);
	tellGalil(buf);
	sprintf(buf, "PR%c=%d", axischar, steps);
	tellGalil(buf);
	sprintf(buf, "BG%c", axischar);
	tellGalil(buf);
	while (isMoving(axis)) {
		usleep(10000);
	}
/* DOn't do this here (not symetric, but allows small motions without brakes
	switch (axis) {
		case XAXIS:
			brake(XAXIS, ON);
			motorPower(XAXIS, OFF);
			break;
		case YAXIS:
			brake(YAXIS, ON);
			motorPower(YAXIS, OFF);
			break;
		case ZAXIS:
			motorPower(ZAXIS, OFF);
			break;
	}
	
*/
}

void moveRel(x, y)
long int x, y;
{

	char buf[80];
	int i, keypress, motorState;
	long int acceleration, deceleration, speed;

	if (x == 0 && y == 0) {
		printf("No motion\n");
		return;
	}
	acceleration = DEFACCEL;
	deceleration = DEFDECEL;
	speed = DEFSPEED;

	sprintf(buf, "ACA=%d", acceleration);
	tellGalil(buf);
	sprintf(buf, "DCA=%d", deceleration);
	tellGalil(buf);
	sprintf(buf, "ACB=%d", acceleration);
	tellGalil(buf);
	sprintf(buf, "DCB=%d", deceleration);
	tellGalil(buf);
	sprintf(buf, "SPA=%d", speed);
	tellGalil(buf);
	sprintf(buf, "SPB=%d", speed);
	tellGalil(buf);
	sprintf(buf, "PRA=%d", x);
	tellGalil(buf);
	sprintf(buf, "PRB=%d", y);
	tellGalil(buf);
	motorPower(XAXIS, ON);
	motorPower(YAXIS, ON);
	tellGalil("BGA");
	tellGalil("BGB");

	while (isMoving(XAXIS) || isMoving(YAXIS)) {
		setMode(NONBLOCKING);
		if (getKey()) {
			// Keyboard stop
			printf("Motion stopped by user\n");
			tellGalil("STA");
			tellGalil("STB");
			usleep(50000);
			motorPower(XAXIS, OFF);
			motorPower(YAXIS, OFF);
			return;
		} else {
			usleep(10000);
		}
	}
	motorPower(XAXIS, OFF);
	motorPower(YAXIS, OFF);
	return;
}

/*-------------------------------------------------------------------

	moveRel(X,y)

	moveRel moves the X-Y stage by x and y relative motor steps.

-------------------------------------------------------------------*/
void moveRelVector(x, y)
long int x, y;
{

	char buf[80];
	int i, keypress, motorState;

	printf("Hit any key to stop motion\n");

	tellGalil("VS 3000");			// Vector speed
	tellGalil("VA 8500");			// Vector acceleration
	tellGalil("VD 20000");			// Vector deceleration
	motorPower(XAXIS, ON);			// Motors on
	motorPower(YAXIS, ON);
	brake(XAXIS, OFF);
	brake(YAXIS, OFF);
	tellGalil("LMXY");			// Linear interpolation motion
	sprintf(buf, "LI %ld,%ld", x, y);	// Build vector command
	tellGalil(buf);				// Send LI command
	if (debugFlag) {
		printf("sent to Galil: %s\n", buf);
	}
	tellGalil("LE");			// End the LM sequence
	tellGalil("BGS");			// Begin motion sequence

	i = 0;
	setMode(NONBLOCKING);
	while (!getKey()) {
		motorState = askGalilForInt("TSA");
		if ((motorState>>7) & (0x01)) {	// Motor still moving?
			usleep(10000);
			i++;
		} else {
			if (debugFlag) {
				printf("%d loops in linear interpolation motion\n", i);
			}
			brake(XAXIS, ON);
			brake(YAXIS, ON);
			motorPower(XAXIS, OFF);
			motorPower(YAXIS, OFF);
			return;
		}
	}

	// Keyboard stop
	printf("Motion stopped by user\n");
	tellGalil("STS");
	motorState = askGalilForInt("TSA");
	while ((motorState>>7) & (0x01)) {
		usleep(10000);
		motorState = askGalilForInt("TSA");
	}
	brake(XAXIS, ON);
	brake(YAXIS, ON);
	motorPower(XAXIS, OFF);
	motorPower(YAXIS, OFF);
	return;

}

/* Talk to the Galil directly */
void passthru()
{

	char cmd[80], buf[80];

	printf("Galil command\n:");
	fflush(stdout);
	gets(cmd);
	askGalil(cmd, buf, 80);
	printf("%s\n", buf);
	if (buf[strlen(buf) - 1] == '?') {	// Error message from Galil?
		askGalil("TC1", buf, 80);
		printf("%s\n", buf);		// Print TC1 error message
	}
	fflush(stdout);

}

/*-------------------------------------------------------------------

	selfCheck()

	selfCheck does a few sanity checks on the guider. It moves
	the three pneumatic cyinders and notices if the GMR sensors
	are appropriately triggered and it moves the X-Y axis motors
	to see if the axis encoders report position changes of about
	the correct amount.

-------------------------------------------------------------------*/
int selfCheck()
{

	int i, testVal, retVal, motorState, a, b;
	long int oldEnc;
	float encScale;

	printf("SelfCheck ");
	fflush(stdout);

	retVal = PASS;
	for (i = 0; i < 2; i++) {
		testVal = cylinder(Y1AXIS, EXTEND);
		if (testVal != 1) {
			printf("\nY1 Extend test returns %d (should be 1)\n", testVal);
			retVal = FAIL;
		} 
		testVal = cylinder(Y1AXIS, RETRACT);
		if (testVal != 0) {
			printf("\nY1 Retract test returns %d (should be 0)\n", testVal);
			retVal = FAIL;
		}
		testVal = cylinder(Y2AXIS, EXTEND);
		if (testVal != 1) {
			printf("\nY2 Extend test returns %d (should be 1)\n", testVal);
			retVal = FAIL;
		}
		testVal = cylinder(Y2AXIS, RETRACT);
		if (testVal != 0) {
			printf("\nY2 Retract test returns %d (should be 0)\n", testVal);
			retVal = FAIL;
		}
	}


	if (limitSwitch(XAXIS) || limitSwitch(YAXIS)) {
		printf("selfCheck() says limit switches active (motor cable unplugged?); no motion test\n");
	} else {
		oldEnc = encPosition(XAXIS);
		moveRel(500,0);
		encScale = (float) (encPosition(XAXIS) - oldEnc) / 500.0;
		if (fabs(encScale - 4.0) > 0.001) {
			printf("Fail X encoder scale (%7.5f)\n", encScale);
			retVal = FAIL;
		}
		oldEnc = encPosition(YAXIS);
		moveRel(0,500);
		encScale = (float) (encPosition(YAXIS) - oldEnc) / 500.0;
		if (fabs(encScale - 4.0) > 0.001) {
			printf("Fail Y encoder scale (%7.5f)\n", encScale);
			retVal = FAIL;
		}
	}
	if (retVal == PASS) {
		printf("passed\n");
	} else {
		printf("failed\n");
	}
	fflush(stdout);
	return(retVal);
}

/*-------------------------------------------------------------------

	setMode(mode)

	setMode switches between ICANON and ~ICANON tty modes. This
	is used for the command keys. mode is either NONBLOCKING
	(for one-key command mode) or BLOCKING (for line oriented
	input).

-------------------------------------------------------------------*/
void setMode(mode)
int mode;
{

	static int firstCall = 1;
	static struct termios oldtty, newtty;

	if (firstCall) {
		tcgetattr(STDIN_FILENO, &oldtty);
		firstCall = 0;
	}
	if (mode == NONBLOCKING) {
		newtty = oldtty;
		newtty.c_lflag &= ~(ICANON | ECHO);
		tcsetattr(STDIN_FILENO, TCSANOW, &newtty);
	} else {
		tcsetattr(STDIN_FILENO, TCSANOW, &oldtty);
	}

}

/*-------------------------------------------------------------------

	resetGalil()

	resetGalil sends the RS command to the Galil. This should
	reset the controller to its power-on state.

-------------------------------------------------------------------*/
void resetGalil()
{

	printf("RESET (error message is normal)\n");
	tellGalil("RS");

}

/*-------------------------------------------------------------------

	shCam()

	shCam inserts the Shack-Hartmann lenslet array into the beam.

-------------------------------------------------------------------*/
int shCam()
{

	int status;

	printf("shack-Hartmann lenslets ");
	fflush(stdout);
	status = cylinder(Y1AXIS, EXTEND);
	if (status != EXTEND) {
		printf("error: Galil reports Y1 insertion error\n");
		return(UNKNOWN);
	}
	printf("in\n");
	fflush(stdout);
	return(IN);

}

/*-------------------------------------------------------------------

	smallAp()

	smallAp inserts the small aperture into the camera beam. It
	is the opposite action of fieldLens().

-------------------------------------------------------------------*/
int smallAp()
{

	int status;

	printf("small aperture ");
	fflush(stdout);
	status = cylinder(Y2AXIS, EXTEND);
	if (status != EXTEND) {
		printf("error: Galil reports small aperture insertion error\n");
		return(UNKNOWN);
	} else {
		printf("in\n");
		return(IN);
	}
}

/*-------------------------------------------------------------------

	statusPrint()

	statusPrint prints status information. Most values are
	from direct queries to the Galil controller, although some
	items (SAXIS position, for example) no not have a sensor.

-------------------------------------------------------------------*/
void statusPrint()
{

	int sensors, motorState;

	printf("status:\n");
	printf("homeTime (local, remote): %ld %ld\n", homeTime, askGalilForLong("MG homeTime"));

	printf("motors homed? ");
	if (isHomed()) {
		printf("YES\n");
	} else {
		printf("NO\n");
	}

	printf("X-brake ");
	if (brake(XAXIS, STATUS) == ON) {
		printf("ON\n");
	} else if (brake(XAXIS, STATUS) == OFF) {
		printf("OFF\n");
	} else {
		printf("UNKNOWN\n");
	}

	printf("Y-brake ");
	if (brake(YAXIS, STATUS) == ON) {
		printf("ON\n");
	} else if (brake(YAXIS, STATUS) == OFF) {
		printf("OFF\n");
	} else {
		printf("UNKNOWN\n");
	}

	// Print the motor step position (RP)
	printf("Motors (X,Y,Z) = (%ld, %ld, %ld)\n", stepPosition(XAXIS), stepPosition(YAXIS), stepPosition(ZAXIS));

	// Print the encoder counts (TP)
	printf("Encoder: (X,Y) = (%ld, %ld)\n", encPosition(XAXIS), encPosition(YAXIS));

	// Print encoder offsets
	printf("Encoder offsets: (X,Y) = (%ld, %ld)\n", xEncOffset, yEncOffset);

	// Print encoder minvals
	printf("Encoder minvals: (X,Y) = (%ld, %ld)\n", xEncMin, yEncMin);

	if (isCalibrated) {
		printf("Stage position (x,y) %7.3f %7.3f (mm)\n", mmPosition(XAXIS), mmPosition(YAXIS));
	}

	// Print the sensor states
	printf("Cylinders:\n");
	if (cylinder(Y1AXIS, STATUS) == EXTEND) {
		printf("Y1 EXTENDED\n");
	} else if (cylinder(Y1AXIS, STATUS) == RETRACT) {
		printf("Y1 RETRACTED\n");
	} else {
		printf("Y1 UNKNOWN\n");
	}
	if (cylinder(Y2AXIS, STATUS) == EXTEND) {
		printf("Y2 EXTENDED\n");
	} else if (cylinder(Y1AXIS, STATUS) == RETRACT) {
		printf("Y2 RETRACTED\n");
	} else {
		printf("Y2 UNKNOWN\n");
	}
	if (cylinder(SAXIS, STATUS) == EXTEND) {
		printf("S EXTENDED\n");
	} else if (cylinder(SAXIS, STATUS) == RETRACT) {
		printf("S RETRACTED\n");
	} else {
		printf("S UNKNOWN\n");
	}

	// Print the limit switch states
	printf("X Limits: ");
	if (limitSwitch(XAXIS)) {
		if (limitSwitch(XAXIS) & 0x01) {
			printf("Reverse ");
		}
		if ((limitSwitch(XAXIS)>>1) & 0x01) {
			printf("Forward");
		}
	} else {
		printf("Neither active");
	}
	printf("\n");

	printf("Y Limits: ");
	if (limitSwitch(YAXIS)) {
		if (limitSwitch(YAXIS) & 0x01) {
			printf("Reverse ");
		}
		if ((limitSwitch(YAXIS)>>1) & 0x01) {
			printf("Forward ");
		}
	} else {
		printf("Neither active");
	}
	printf("\n");
}


/*-------------------------------------------------------------------

	stepPosition(axis)

	stepPosition returns the motor step position of the selected
	axis. Axis may be [XAXIS|YAXIS|ZAXIS].

	The Galil runs stepper motors open-loop so this number is
	just a count of the number of steps and may not reflect the
	true position if motor steps were lost. This number can be
	reset with the Galil DP command, which is done on a homing
	operation.

-------------------------------------------------------------------*/
long int stepPosition(axis)
int axis;

{

	switch (axis) {
		case XAXIS:
			return(askGalilForLong("RPA"));
		case YAXIS:
			return(askGalilForLong("RPB"));
		case ZAXIS:
			return(askGalilForLong("RPC"));
		default:
			printf("stepPosition(%d) passed bad axis value\n", axis);
			return(UNKNOWN);
	}
}
	

/*-------------------------------------------------------------------

	tellGalil(cmd)

	Sends a command to the Galil controller. cmd is a NUL
	terminated string containing the Galil command. If the
	Galil returns a '?' instead of a ':' then this function
	prints the TC1 error message.

-------------------------------------------------------------------*/
int tellGalil(cmd)
char *cmd;
{

	char buf[80];

	if (debugFlag) {
		printf("tellGalil(%s)\n", cmd);
	}
	askGalil(cmd, buf, 80);
	if (buf[0] == ':') {
		return(1);
	} else if (buf[0] == '?') {
		askGalil("TC1", buf, 80);
		printf("?Error, TC1 says: %s\n", buf);
		return(0);
	} else {
		printf("Unexpected response from Galil (hex=%x)\n", buf[0]);
		return(0);
	}
}

/*-------------------------------------------------------------------

	telnetToGalil(ipaddress)

	Opens a socket to the Galil controller at ipaddress.
	ipaddress is a pointer to a string containing the IP address
	of the Galil controller. It returns the file descriptor
	of the socket or prints an error message.

	The ipaddress string must be an IPv4 quartet, not a host name
	(e.g., 192.168.1.2).

-------------------------------------------------------------------*/
int telnetToGalil(ipaddress)
char *ipaddress;
{

	char buf[80];
	int i, fd;
	struct sockaddr_in sockGalil;

	memset(buf, 0, 80);
	memset((char *) &sockGalil, 0, sizeof(sockGalil));
	sockGalil.sin_family = AF_INET;
	sockGalil.sin_port = htons(8079);	// Galil is OK with this port (behaves like telnet)
	if (inet_pton(AF_INET, ipaddress, &(sockGalil.sin_addr)) <= 0) {
		errmsg(INETPTONERROR);
	}

	if ((fd = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
		errmsg(SOCKETERROR);
	}

	if (connect(fd, (struct sockaddr *) &sockGalil, sizeof(sockGalil))) {
		errmsg(CONNECTERROR);
	} else {
		printf("connected to %s\n", ipaddress);
		fflush(stdout);
	}

	// Clear the Galil output buffer (it's always been empty when I've looked)
	i = write(fd, "\r", 1);
	i = read(fd, buf, 255);
	if (buf[0] != ':') {
		printf("Galil response to <cr> was '%c'\n", buf[0]);
		fflush(stdout);
	}

	return(fd);

}


void testFunction1()
{


	int i, status;

	printf("\n");



	moveOneAxis(YAXIS, -200, 1024);
/*

	status = led(ON);
	printf("LED on?\n");
	sleep(2);
	status = led(OFF);
	printf("LED off?\n");
	sleep(2);
	status = led(STATUS);
	printf("led() reports for status %d\n", status);


	status = motorPower(XAXIS,STATUS);
	if (status == OFF) {
		printf("motorPower reports xmotor power OFF\n");
	} else if (status == ON) {
		printf("motorPower reports xmotor power ON\n");
	}
	status = askGalilForInt("TSA");
	printf("galil reports %d\n", ((status>>5) & 0x01));

	status = motorPower(YAXIS,STATUS);
	if (status == OFF) {
		printf("motorPower reports ymotor power OFF\n");
	} else if (status == ON) {
		printf("motorPower reports ymotor power ON\n");
	}
	status = askGalilForInt("TSB");
	printf("galil reports %d\n", ((status>>5) & 0x01));

	status = motorPower(ZAXIS,STATUS);
	if (status == OFF) {
		printf("motorPower reports zmotor power OFF\n");
	} else if (status == ON) {
		printf("motorPower reports zmotor power ON\n");
	}
	status = askGalilForInt("TSC");
	printf("galil reports %d\n", ((status>>5) & 0x01));
	

	if (motorPower(XAXIS,ON) == ON) {
		printf("motor x on\n");
	} else {
		printf("error\n");
	}
	if (motorPower(YAXIS,ON) == ON) {
		printf("motor y on\n");
	} else {
		printf("y error\n");
	}
	if (motorPower(ZAXIS,ON) == ON) {
		printf("motor z on\n");
	} else {
		printf("z error\n");
	}
/////////////
	if (brake(XAXIS,STATUS) == ON) {
		printf("brake routine claims x is ON\n");
	} else if (brake(XAXIS,STATUS) == OFF) {
		printf("brake routine claims x OFF\n");
	} else {
		printf("brake routine error\n");
	}
	if (brake(YAXIS,STATUS) == ON) {
		printf("brake routine claims y is ON\n");
	} else if (brake(YAXIS,STATUS) == OFF) {
		printf("brake routine claims y OFF\n");
	} else {
		printf("brake routine error\n");
	}

	status = ~askGalilForInt("TI0");
	for (i = 2; i < 6; i++) {
		printf("%d bit is %d\n", i, ((status>>i) & 0x01));

	}

	sleep(1);
	printf("Y1 Status returns %d (should be 0)\n", cylinder(Y1AXIS,STATUS));
	sleep(1);
	printf("Y2 Status returns %d (should be 0)\n", cylinder(Y2AXIS,STATUS));
	sleep(1);
	printf("S Status returns %d (should be 0)\n", cylinder(SAXIS,STATUS));
	sleep(1);
	printf("Y1 Extend returns %d (should be 1)\n", cylinder(Y1AXIS,EXTEND));
	sleep(1);
	printf("Y1 Retract returns %d (should be 0)\n", cylinder(Y1AXIS,RETRACT));
	sleep(1);
	printf("Y2 Extend returns %d (should be 1)\n", cylinder(Y2AXIS,EXTEND));
	sleep(1);
	printf("Y2 Retract returns %d (should be 0)\n", cylinder(Y2AXIS,RETRACT));
	sleep(1);
	printf("S Extend returns %d (should be 1)\n", cylinder(SAXIS,EXTEND));
	sleep(1);
	printf("S Retract returns %d (should be 0)\n", cylinder(SAXIS,RETRACT));
//UNKNOWN
	sleep(1);
	printf("brake status:\n");
	printf("brake X = %d Y = %d Z = %d\n", brake(XAXIS, STATUS), brake(YAXIS, STATUS), brake(ZAXIS, STATUS));

//X-OFF
	sleep(1);
	printf("xbrake off\n");
	brake(XAXIS, OFF);
	sleep(1);
	printf("brake X = %d Y = %d\n", brake(XAXIS, STATUS), brake(YAXIS, STATUS));

//Y-OFF
	sleep(1);
	printf("ybrake off\n");
	brake(YAXIS, OFF);
	sleep(1);
	printf("brake X = %d Y = %d\n", brake(XAXIS, STATUS), brake(YAXIS, STATUS));

//X-ON
	sleep(1);
	printf("xbrake on\n");
	brake(XAXIS, ON);
	sleep(1);
	printf("brake X = %d Y = %d\n", brake(XAXIS, STATUS), brake(YAXIS, STATUS));


//Y-ON
	sleep(1);
	printf("ybrake on\n");
	brake(YAXIS, ON);
	sleep(1);
	printf("brake X = %d Y = %d\n", brake(XAXIS, STATUS), brake(YAXIS, STATUS));

	sleep(1);
	printf("both brakes off\n");
	brake(YAXIS, OFF);
	brake(XAXIS, OFF);
	sleep(1);
	printf("brake X = %d Y = %d\n", brake(XAXIS, STATUS), brake(YAXIS, STATUS));
*/
}

void testFunction2()

{

	int lim, forward, reverse;

	lim = limitSwitch(XAXIS);
	forward = reverse = 0;
	if (lim & 0x01) {
		reverse = 1;
	}
	if ((lim>>1) & 0x01) {
		forward = 1;
	}
	printf("X limits: Rev=%d, For=%d\n", reverse, forward);

	lim = limitSwitch(YAXIS);
	forward = reverse = 0;
	if (lim & 0x01) {
		reverse = 1;
	}
	if ((lim>>1) & 0x01) {
		forward = 1;
	}
	printf("Y limits: Rev=%d, For=%d\n", reverse, forward);

	lim = limitSwitch(ZAXIS);
	forward = reverse = 0;
	if (lim & 0x01) {
		reverse = 1;
	}
	if ((lim>>1) & 0x01) {
		forward = 1;
	}
	printf("Z limits: Rev=%d, For=%d\n", reverse, forward);

}

void testFunction7()
{


	static int direction = 1;
	moveOneAxis(YAXIS, 200*direction, DEFSPEED);
	motorPower(YAXIS, OFF);
	direction = -direction;

}

void testFunction4()

{

	creepToLimits(YAXIS, -5000, 500);

}

void testFunction5()
{

	printf("\n");
	printf("X=%ld\n", stepPosition(XAXIS));
	printf("Y=%ld\n", stepPosition(YAXIS));
	printf("Z=%ld\n", stepPosition(ZAXIS));
	printf("X=%ld\n", encPosition(XAXIS));
	printf("Y=%ld\n", encPosition(YAXIS));


}

void testFunction6()
{

	backOff();

}

void testFunction8()
{

	creepToLimits(YAXIS, 1, 20);

}

void testFunction()
{

	moveAbs(50.0, 20.0);

}

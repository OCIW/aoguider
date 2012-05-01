/* aoguider

	2012-04-30 au - calibrations for new lead screw & focus motor
	2012-04-12 au - added calibration routine
	2012-04-06 au - added homing routine
	2012-04-05 au - start

	Test program demonstrating the Magellan AO guider using
	a Galil DMC 4060 controller. Compiles under Windows/Cygwin.

*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>
#include <arpa/inet.h>

#define CYGWIN
#ifdef CYGWIN
#include <cygwin/in.h>
#else
#include <netinet/in.h>
#endif

//#define GALILIP	"192.168.1.2"		// Generic private IP address
#define GALILIP 	"192.91.178.197"	// Jorge's sodium
#define GALILPORT	8079

#define XAXIS		1
#define YAXIS		2
#define ZAXIS		3
#define Y1AXIS		4
#define Y2AXIS		5
#define	SAXIS		6

#define BLOCKING	0
#define NONBLOCKING	1

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

// air cylinder sensors
#define RETRACT		0
#define EXTEND		1
#define UNKNOWN		-1
#define BOTHSENSORS	-2
#define BADAXIS		-999999

#define	IN		1
#define OUT		0

#define XMAXSTEPS	20847		// Maximum x-motor steps after homing
#define YMAXSTEPS	64596		// Maximum y-motor steps after homing
#define	ZMAXSTEPS	87424		// Maximum z-motor steps after homing
#define XSTEPSPERTURN	500		// Motor steps in 360 degrees
#define YSTEPSPERTURN	500
#define ZSTEPSPERTURN	4000
#define XENCPULSPERTURN	2000
#define YENCPULSPERTURN	2000
#define XSCREWPITCH	0.0625
#define YSCREWPITCH	0.0625
#define ZSCREWPITCH	0.050

//X and Y axis defaults
#define XYSPEED		3800		// Speed
#define XYACCEL		9000		// Acceleration
#define XYDECEL		20000		// Deceleration
#define XYLIMITHYSTER	150		// Limit switch hysteresis
#define ZSPEED		4000
#define ZACCEL		128000
#define ZDECEL		128000
#define ZLIMITHYSTER	2700

// Function prototypes
void	backOff();
void	demo();
void	fakeHome(void);
void	askGalil(char *, char *, int);
int	askGalilForInt(char *);
long int askGalilForLong(char *);
int	limitSwitch(int);
int	brake(int, int);
void	calibrate(void);
void	cmdLoop(void);
int	creepToLimits(int, int, int);
int	cylinder(int, int);
void	debug(void);
long int encPosition(int);
int	fieldCam(void);
int	fieldLens(void);
void	focus(int);
void	focusRel(long int);
int	getKey(void);
void	help(void);
void	homeAxes(void);
float	inchPosition(int);
void	initGuider(void);
int	isHomed(void);
int	isMoving(int);
int	led(int);
int	ledInOut(int);
int	motorPower(int, int);
void	move(int);
int	moveAbs(float, float);
void	moveOneAxis(int, int, int);
void	moveRel(long int, long int);
void	passthru(void);
void	resetGalil(void);
int	shCam(void);
int	selfCheck();
void	setMode(int);
int	smallAp(void);
void	statusPrint(void);
long int stepPosition(int);
char	*tellGalil(char *);
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

/*=================================================================*/
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
	if (galilfd < 0) {
		printf("Connection to %s failed (return code %d)\n", GALILIP, galilfd);
		return(0);
	}
	for (;;) {
		cmdLoop();
	}

}

/*-------------------------------------------------------------------

	askGalil(cmd, buf, n);  (LIBRARY)

	askGalil sends a command string (cmd) to the Galil controller
	and returns the controller's reply in buf. n is the available
	space in buf.  The global variable galilfd must already be a
	valid socket to the Galil controller.

	cmd is a pointer to a NUL terminated string containing the
	Galil command. For example, "TPA" (Galil command "Tell
	Position, A axis. askGalil adds a carriage return ('\r') to
	the command sent to the Galil. The reply string from the Galil
	is returned in buf.

Checked 2012-04-30
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
	read(galilfd, buf, n);

}

/*-------------------------------------------------------------------

	int askGalilForInt(cmd); (LIBRARY)

	askGalilForInt sends the command string (cmd) to the Galil
	controller and returns the Galil reply as an integer.

	cmd is a pointer to a NUL terminated string containing a
	Galil command that returns an integer (as do most commands).

Checked 2012-04-30
-------------------------------------------------------------------*/

int askGalilForInt(cmd)
char *cmd;
{

	char buf[80];

	askGalil(cmd, buf, 80);
	return(atoi(buf));

}

/*-------------------------------------------------------------------

	long int askGalilForLong(cmd); (LIBRARY)

	askGalilForLong sends the command string (cmd) to the Galil
	controller and returns the Galil reply as a long integer.

	cmd is a pointer to a NUL terminated string containing a
	Galil command that returns an integer (as do most commands).

Checked 2012-04-30
-------------------------------------------------------------------*/
long int askGalilForLong(cmd)
char *cmd;
{

	char buf[80];

	askGalil(cmd, buf, 80);
	return(atol(buf));

}

/*-------------------------------------------------------------------

	int limitSwitch(axis) (LIBRARY)

	limitSwitch returns an integer with bits 0 and 1 indicating
	the reverse and forward limit switch states respectively.
	If the limit is engaged, the bit is 1 and motion is diabled
	in that direction. If the limit is not engaged the bit is 0.

Checked 2012-04-26
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
			return(BADAXIS);
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

	void backOff(); (LIBRARY)

	backOff clears a limit switch situation by searching for
	engaged limits and backing away a few steps.

Checked 2012-04-30
-------------------------------------------------------------------*/
void backOff()
{

	int testVal;

	if ((testVal = limitSwitch(XAXIS))) {
		if (testVal & 0x01) {
			creepToLimits(XAXIS, 20, XYSPEED/4);
		} else if (testVal & 0x02) {
			creepToLimits(XAXIS, -20, XYSPEED/4);
		}
	}

	if ((testVal = limitSwitch(YAXIS))) {
		if (testVal & 0x01) {
			creepToLimits(YAXIS, 20, XYSPEED/4);
		} else if (testVal & 0x02) {
			creepToLimits(YAXIS, -20, XYSPEED/4);
		}
	}
	if ((testVal = limitSwitch(ZAXIS))) {
		if (testVal & 0x01) {
			creepToLimits(ZAXIS, 200, ZSPEED/4);
		} else if (testVal & 0x02) {
			creepToLimits(ZAXIS, -200, ZSPEED/4);
		}
	}

}

/*-------------------------------------------------------------------

	int brake(axis, onOffStatus);

	brake(axis, onOffStatus) sets or releases a motor brake or
	returns the current brake state for the selected axis. Only
	the X and Y axes on the Magellan AO guider have brakes, so
	axis can only be one of XAXIS, YAXIS. onOffStatus can be
	ON, OFF, or STATUS. Values returned may be ON, OFF,
	UNKNOWN, or BADAXIS.

Checked 2012-04-30
-------------------------------------------------------------------*/
int brake(axis, onOffStatus)
int axis, onOffStatus;
{

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
				return(BADAXIS);
		}
	}

	if (onOffStatus == ON) {
		switch (axis) {
			case XAXIS:
				tellGalil("CB1");
				if (askGalilForInt("MG@OUT[1]") == 0) {
					return(ON);
				} else {
					return(UNKNOWN);
				}
			case YAXIS:
				tellGalil("CB2");
				if (askGalilForInt("MG@OUT[2]") == 0) {
					return(ON);
				} else {
					return(UNKNOWN);
				}
			default:
				return(BADAXIS);
		}
	} else if (onOffStatus == OFF) {
		switch (axis) {
			case XAXIS:
				tellGalil("SB1");
				if (askGalilForInt("MG@OUT[1]")) {
					return(OFF);
				} else {
					return(UNKNOWN);
				}
			case YAXIS:
				tellGalil("SB2");
				if (askGalilForInt("MG@OUT[2]")) {
					return(OFF);
				} else {
					return(UNKNOWN);
				}
			default:
				return(BADAXIS);
		}
	} else {
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

Checked 2012-04-30
-------------------------------------------------------------------*/
void calibrate()
{

	homeAxes();

	// go to the reverse limit
	creepToLimits(XAXIS, -25000, XYSPEED);
	moveOneAxis(XAXIS, 150, XYSPEED/2);
	while (isMoving(XAXIS)) {
		}
	creepToLimits(XAXIS, 5, XYSPEED/2);
	moveOneAxis(XAXIS, XSTEPSPERTURN, XYSPEED/2);
	motorPower(XAXIS, OFF);

	creepToLimits(YAXIS, -25000, XYSPEED);
	moveOneAxis(YAXIS, 150, XYSPEED/2);
	while (isMoving(YAXIS)) {
		}
	creepToLimits(YAXIS, 5, XYSPEED/2);
	moveOneAxis(YAXIS, YSTEPSPERTURN, XYSPEED/2);
	motorPower(YAXIS, OFF);

	// Set global xEncPerStep, yEncPerStep
	xEncPerStep = (float) (encPosition(XAXIS) - xEncOffset) / (float) stepPosition(XAXIS);
	yEncPerStep = (float) (encPosition(YAXIS) - yEncOffset) / (float) stepPosition(YAXIS);

	// Set global xEncMin, yEncMin
	xEncMin = encPosition(XAXIS);
	yEncMin = encPosition(YAXIS);

	isCalibrated = 1;

}

/*-------------------------------------------------------------------

	void cmdLoop()
	
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
		printf("aperture, small");
		fflush(stdout);
		smallAp();
		printf(".\n");
		fflush(stdout);
	} else if (cmd == 'A') {	// Insert the field lens
		printf("field lens");
		fflush(stdout);
		fieldLens();
		printf(".\n");
		fflush(stdout);
	} else if (cmd == 'B') {	// Back off limits
		printf("Backoff limits");
		fflush(stdout);
		backOff();
		printf(".\n");
		fflush(stdout);
	} else if (cmd == 'C') {	// Calibrate
		printf("Calibrate");
		fflush(stdout);
		calibrate();
		printf(".\n");
		fflush(stdout);
	} else if (cmd == 'D') {	// Demo mode 
		printf("Demo");
		fflush(stdout);
		demo();
		printf(".\n");
		fflush(stdout);
	} else if (cmd == 'd') {	// Toggle debug flag
		printf("debug flag");
		fflush(stdout);
		debug();
		printf(".\n");
		fflush(stdout);
	} else if (cmd == 'f') {	// focus relative
		focus(FOCUSREL);
	} else if (cmd == 'F') {	// Focus to absolute position
		focus(FOCUSABS);
	} else if (cmd == 'h') {	// list commands help
		help();
	} else if (cmd == 'H') {	// Home the X and Y axes
		printf("Home");
		fflush(stdout);
		homeAxes();
		printf(".\n");
		fflush(stdout);
	} else if (cmd == 'i') {	// Initialize
		printf("initializing");
		fflush(stdout);
		initGuider();
		printf(".\n");
		fflush(stdout);
	} else if (cmd == 'l') {	// Set up LED with S-H
		printf("led ");
		if (ledInOut(STATUS) == IN) {
			ledInOut(OUT);
			printf("out.\n");
		} else {
			ledInOut(IN);
			printf("in.\n");
		}
		fflush(stdout);
	} else if (cmd == 'm') {	// relative position move
		move(RELATIVE);
	} else if (cmd == 'M') {	// absolute position move
		move(ABSOLUTE);
	} else if (cmd == 'q') {	// quit
		exit(0);
	} else if (cmd == 'R') {	// Reset
		printf("Reset");
		fflush(stdout);
		resetGalil();
		printf(".\n");
		fflush(stdout);
	} else if (cmd == 'S') {	// Self check
		printf("Self check ");
		fflush(stdout);
		selfCheck();
		printf(".\n");
		fflush(stdout);
	} else if (cmd == 's') {	// Set up for Shack-Hartmann
		printf("shack-Hartmann lenslets in");
		fflush(stdout);
		shCam();
		printf(".\n");
		fflush(stdout);
	} else if (cmd == 'T') {	// Run the Test function
		printf("Test function");
		fflush(stdout);
		testFunction();
		printf(".\n");
		fflush(stdout);
	} else if (cmd == 'w') {	// Set up for wide field viewing
		printf("wide field camera");
		fflush(stdout);
		fieldCam();
		printf(".\n");
		fflush(stdout);
	} else if (cmd == '?') {	// status
		statusPrint();
	} else if (cmd == ':') {	// Talk directly to the Galil
		passthru();
	} else {
		printf("%c? Type \"h\" for command list help\n", cmd);
		fflush(stdout);
	}

}


/*-------------------------------------------------------------------

	int creepToLimits(axis, steps, speed); (LIBRARY)

	creepToLimits moves the selected motor axis in increments of
	"steps" at the selected "speed" until that axis's limit switch
	changes state. Note that the limit switch may alread be engaged
	so a change of state could be from engaged to not engaged.

	Pay attention the step direction since you don't want to travel
	the full length of the stage at slow speed.

	Returns 1 for success, 0 if you used a bad axis value or
	the step size was small and the process timed out before
	reaching the limit.

Checked 2012-04-30
-------------------------------------------------------------------*/
int creepToLimits(axis, steps, speed)
int axis, steps, speed;
{

	char axischar;
	int i, oldLimits, maxLoops;

	maxLoops = 200;

	switch (axis) {

		case XAXIS:
			axischar = 'A';
			i = 0;
			oldLimits = limitSwitch(XAXIS);
			while (oldLimits == limitSwitch(XAXIS) && i < maxLoops) {	// wait until the switch changes state
				moveOneAxis(XAXIS, steps, speed);
				while (isMoving(XAXIS)) {
					}
				i++;
			}
			motorPower(XAXIS, OFF);
			break;

		case YAXIS:
			axischar = 'B';
			i = 0;
			oldLimits = limitSwitch(YAXIS);
			while (oldLimits == limitSwitch(YAXIS) && i < maxLoops) {
				moveOneAxis(YAXIS, steps, speed);
				while (isMoving(YAXIS)) {
					}
				i++;
			}
			motorPower(YAXIS, OFF);
			break;

		case ZAXIS:
			axischar = 'C';
			i = 0;
			oldLimits = limitSwitch(ZAXIS);
			while (oldLimits == limitSwitch(ZAXIS) && i < maxLoops) {
				moveOneAxis(ZAXIS, steps, speed);
				while (isMoving(ZAXIS)) {
					}
				i++;
			}
			motorPower(ZAXIS, OFF);
			break;

		default:
			return(0);

	}
	if (i <= maxLoops) {
		return(1);
	} else {
		return(0);
	}
}

/*-------------------------------------------------------------------

	int cylinder(axis, exRetStatus); (LIBRARY)
	
	cylinder sets or returns information on the three pneumatic
	cylinders. "axis" should be one of Y1AXIS, Y2AXIS, or SAXIS
	and "exRetStatus" should be one of EXTEND, RETRACT, STATUS.
	
	EXTEND and RETRACT cause that action to be performed. STATUS
	returns one of EXTEND, RETRACT, UNKNOWN, BOTHSENSORS, or
	BADAXIS. UNKNOWN and BOTHSENSORS probably indicate a
	timeout situation due to low air pressure. BADAXIS means
	that you called the routine with an invalid axis.

Checked 2012-04-30
-------------------------------------------------------------------*/
int cylinder(axis, extRetStatus)
int axis, extRetStatus;
{

	int i, status, y1e, y1r, y2e, y2r;
	static int sAxisStatus = UNKNOWN;

	if (extRetStatus == STATUS) {

		status = ~askGalilForInt("TI0");
		y1e = ((status>>4) & 0x01);
		y1r = ((status>>5) & 0x01);
		y2e = ((status>>2) & 0x01);
		y2r = ((status>>3) & 0x01);

		if ((y1e == y1r) || (y2e == y2r)) {
			return(BOTHSENSORS);
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
				return(BADAXIS);

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
				return(UNKNOWN);

			case SAXIS:
				tellGalil("CB3");
				sAxisStatus = RETRACT;
				sleep(1);
				return(RETRACT);

			default:
				return(BADAXIS);
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
				return(UNKNOWN);

			case SAXIS:
				tellGalil("SB3");
				sAxisStatus = EXTEND;
				sleep(1);
				return(EXTEND);

			default:
				return(BADAXIS);
		} 
	} else {
		return(BADAXIS);
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

	long int encPosition(axis)
	
	encPosition returns the current encoder position of the
	selected axis. "axis" may be either XAXIS or YAXIS (the
	focus axis does not have an encoder).

Checked 2012-04-30
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
			return(BADAXIS);

	}

}

/*-------------------------------------------------------------------

	inchPosition(axis)
	
	inchPosition returns the current X-Y position in inches away
	from the home position of the selected axis based on the
	rotary encoders. "axis" may be either XAXIS or YAXIS. This
	position is computed from the manufacturer's specs on the
	pitch of the lead screw and the measured number of pulses
	per turn sent by the encoder (computed with the calibrate()
	routine).

	The name should be changed to inchPosition since it now
	reports inches (this change was made after swapping out
	the old lead screws with new inch-based ones).

Checked 2012-04-30
-------------------------------------------------------------------*/
float inchPosition(axis)
int axis;
{

	switch(axis) {
		case XAXIS:
			return((float) ((xEncOffset - encPosition(XAXIS)) * XSCREWPITCH) / (float) (XENCPULSPERTURN));

		case YAXIS:
			return((float) ((yEncOffset - encPosition(YAXIS)) * YSCREWPITCH) / (float) (YENCPULSPERTURN));

		default:
			return(BADAXIS);
	}
}

/*-------------------------------------------------------------------

	fieldCam()
	
	fieldCam retracts the Y1 cylinder to put Derek Kopon's wide
	field camera into the beam ahead of the CCD. Returns UNKNOWN
	if the cylinder() routine does not detect the retract sensor.

Checked 2012-04-30
-------------------------------------------------------------------*/
int fieldCam()
{

	int status;

	status = cylinder(Y1AXIS, RETRACT);

	if (status != RETRACT) {
		return(UNKNOWN);
	} else {
		return(IN);
	}

}

/*-------------------------------------------------------------------

	void focus(int type) (USER)
	
	focus asks the user for a new focus position and sets the
	focus stage position. "type" may be either FOCUSREL or
	FOCUSABS for relative or absolute motions. Relative motions
	are in motor steps, absolute motions are in thousandths of
	an inch from the home position.

Checked 2012-04-30
-------------------------------------------------------------------*/
void focus(type)
int type;
{

	char buf[20];
	long int x, currentFocus;

	if (type == FOCUSREL) {
		printf("focus offset (steps): ");
		fflush(stdout);
		x = atol(gets(buf));
		focusRel(x);
		return;
	} else if (type == FOCUSABS) {
		if (!isHomed()) {
			printf("not homed\n");
			return;
		}
		currentFocus = stepPosition(ZAXIS);
		printf("New absolute focus value (mils): ");
		x = atol(gets(buf));
		x *= (int) ((float) ZSTEPSPERTURN / (1000.0 * (float) ZSCREWPITCH));
		focusRel(-x - currentFocus);
		return;
	}


}

/*-------------------------------------------------------------------

	void focusRel(long int z); (LIBRARY)
	
	focusRel moves the focus motor by x steps, positive or negative.

Checked 2012-04-30
-------------------------------------------------------------------*/
void focusRel(z)
long int z;
{

	moveOneAxis(ZAXIS, z, ZSPEED);
	motorPower(ZAXIS, OFF);
}


/*-------------------------------------------------------------------

	getKey()
	
	getKey returns a keyboard character hit (from stdin).

Checked 2012-04-30
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

Checked 2012-04-30
-------------------------------------------------------------------*/
void help()
{

	printf("commands:\n\n");
	printf("\ta - aperture, insert small\n");
	printf("\tA - field lens, insert\n");
	printf("\tB - Back off limits\n");
	printf("\tC - Calibrate\n");
	printf("\td - debug flag toggle (diagnostic prints)\n");
	printf("\tD - Demo mode\n");
	printf("\tf - focus, relative motion\n");
	printf("\tF - Focus, absolute position\n");
	printf("\th - this help listing\n");
	printf("\tH - Home the axes\n");
	printf("\ti - initialize\n");
	printf("\tl - led in or out (toggle)\n");
	printf("\tm - move relative\n");
	printf("\tM - Move absolute\n");
	printf("\tq - quit\n");
	printf("\tR - Reset Galil\n");
	printf("\ts - Shack-Hartmann lenslets in\n");
	printf("\tS - Self check\n");
	printf("\tT - Test function execute\n");
	printf("\tw - wide field camera in\n");
	printf("\t? - print status\n");
	printf("\t: - send commands directly to Galil\n");
	fflush(stdout);

}


/*-------------------------------------------------------------------

	int fieldLens(void); (LIBRARY)
	
	fieldLens retracts the Y2 cylinder to put the field lens
	into the CCD beam. The field lens alternates with the small
	aperture.

Checked 2012-04-30
-------------------------------------------------------------------*/
int fieldLens()
{

	int status;

	status = cylinder(Y2AXIS, RETRACT);
	if (status != RETRACT) {
		return(UNKNOWN);
	} else {
		return(IN);
	}
}

/*-------------------------------------------------------------------

	homeAxes() (LIBRARY)
	
	homeAxes drives the stage to the reverse limits, backs off
	to a point point where the limit switch still allows motion,
	then moves one motor turn back from there to define the home
	position.
	
	The  motor positions are zeroed out and the X-Y encoder positions
	are noted (they cannot be zeroed out).

Checked 2012-04-30
-------------------------------------------------------------------*/
void homeAxes()
{

	if (limitSwitch(XAXIS) + limitSwitch(YAXIS) + limitSwitch(ZAXIS)) {
		backOff();
	}

	creepToLimits(XAXIS, 25000, XYSPEED);
	moveOneAxis(XAXIS, -50, XYSPEED/2);
	while (isMoving(XAXIS)) {
		}
	creepToLimits(XAXIS, -5, XYSPEED/2);
	moveOneAxis(XAXIS, -XSTEPSPERTURN, XYSPEED/2);
	motorPower(XAXIS, OFF);

	creepToLimits(YAXIS, 25000, XYSPEED);
	moveOneAxis(YAXIS, -50, XYSPEED/2);
	while (isMoving(YAXIS)) {
		}
	creepToLimits(YAXIS, -5, XYSPEED/2);
	moveOneAxis(YAXIS, -YSTEPSPERTURN, XYSPEED/2);
	motorPower(YAXIS, OFF);

	creepToLimits(ZAXIS, 25000, ZSPEED);
	moveOneAxis(ZAXIS, -1000, ZSPEED/2);
	while (isMoving(ZAXIS)) {
		}
	creepToLimits(ZAXIS, -10, ZSPEED/2);
	moveOneAxis(ZAXIS, -ZSTEPSPERTURN, ZSPEED/2);
	motorPower(ZAXIS, OFF);

	tellGalil("DP 0,0,0");			// zero out the steppers
	xEncOffset = askGalilForLong("TPA");	// Save global variables
	yEncOffset = askGalilForLong("TPB");

	tellGalil("homeTime=TIME");
	homeTime = askGalilForLong("MG homeTime");
	if (debugFlag) {
		printf("homeTime = %ld", homeTime);
	}
}

/*-------------------------------------------------------------------

	initGuider()
	
	initGuider sets up the guider so the pneumatic actuators are
	all retracted, motor brakes are turned on, and various Galil
	parameters are set to reasonable values.

Checked 2012-04-30
-------------------------------------------------------------------*/
void initGuider()
{

	tellGalil("MT -2,-2,-2");		// Tell Galil they're stepper motors
	motorPower(XAXIS, OFF);			// Power down the motors
	motorPower(YAXIS, OFF);
	motorPower(ZAXIS, OFF);
	ledInOut(OUT);
	tellGalil("CN1");			// Limit switch configuration

	cylinder(Y1AXIS, RETRACT);
	cylinder(Y2AXIS, RETRACT);
	cylinder(SAXIS, RETRACT);

	tellGalil("SP 200,200,1000");		// Slow speed
	tellGalil("AC 256000,256000,256000");	// These are defaults from Galil
	tellGalil("DC 256000,256000,256000");
	tellGalil("SD 256000,256000,256000");	// Deceleration after hitting a limit switch
	tellGalil("VS 200");			// Slow vector speed
	tellGalil("VA 256000");			// Default vector values
	tellGalil("VD 256000");
	tellGalil("KS 3,3,3");			// Step motor smoothing (not too sensitive)
	tellGalil("CAS");			// S coordinate system for vector motion

}

/*-------------------------------------------------------------------

	int isHomed() (LIBRARY)
	
	Returns 1 if the guider was homed, 0 if not.

	PROBABLY WANT TO REPLACE THIS WITH "CALIBRATED?"

Checked 2012-04-26
-------------------------------------------------------------------*/
int isHomed()
{

	if (homeTime != askGalilForLong("MG homeTime")) {
		return(0);
	} else {
		return(1);
	}
}

/*-------------------------------------------------------------------

	int isMoving(axis)
	
	axis is one of XAXIS, YAXIS, or ZAXIS.  Returns 1 if the
	axis is moving, 0 if not, BADAXIS if called incorrectly

Checked 2012-04-26
-------------------------------------------------------------------*/
int isMoving(axis)
int axis;
{

	int status;
	char buf[10];

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
			return(BADAXIS);
	}
	status = askGalilForInt(buf);
	return((status>>7) & 0x01);

}

/*-------------------------------------------------------------------

	int led(onOffStatus) (LIBRARY)
	
	Turns the led on or off, or asks for status. onOffStatus
	should be one of ON, OFF, or STATUS. This function returns
	one of ON, OFF, or UNKNOWN. UNKNOWN is returned if the
	Galil controller returns an inconsistent answer or if you
	called the routine with something other than ON, OFF, or
	STATUS.

Checked 2012-04-26
-------------------------------------------------------------------*/
int led(onOffStatus)
{

	static int status;

	switch (onOffStatus) {
		case ON:
			tellGalil("SB4");
			if (askGalilForInt("MG@OUT[4]") == 0) {
				status = UNKNOWN;
			} else {
				status = ON;
			}
			break;

		case OFF:
			tellGalil("CB4");
			if (askGalilForInt("MG@OUT[4]") != 0) {
				status = UNKNOWN;
			} else {
				status = OFF;
			}
			break;

		case STATUS:
			if (askGalilForInt("MG@OUT[4]") == 0) {
				status = OFF;
			} else {
				status = ON;
			}
			break;

		default:
			status = UNKNOWN;
	}
	return(status);
}

/*-------------------------------------------------------------------

	int ledInOut(inOutStatus) (LIBRARY)
	
	Turns the led on or off, or asks for status. inOutStatus
	is one of IN, OUT, or STATUS. This function returns one
	of IN, OUT, UNKNOWN.

Checked 2012-04-29
-------------------------------------------------------------------*/
int ledInOut(inOutStatus)
int inOutStatus;
{

	static int status = UNKNOWN;

	if (inOutStatus == IN) {
		if (cylinder(SAXIS, EXTEND) == EXTEND) {
			led(ON);
			status = IN;
		} else {
			status = UNKNOWN;
		}
	} else if (inOutStatus == OUT) {
		led(OFF);
		if (cylinder(SAXIS, RETRACT) == RETRACT) {
			status = OUT;
		} else {
			status = UNKNOWN;
		}
	}
	return(status);

}

/*-------------------------------------------------------------------

	int motorPower(int axis, int onOffStatus) (LIBRARY)
	
	Controls power to the motors. axis is XAXIS, YAXIS, or ZAXIS.
	onOffStatus is one of ON, OFF, or STATUS.

	This routine returns ON, OFF, BADAXIS (if you supplied an
	incorrect axis value), and UNKNOWN if you supplied an 
	inccorrect onOffStatus value.

Checked 2012-04-26
-------------------------------------------------------------------*/
int motorPower(axis, onOffStatus)
int axis, onOffStatus;
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
			return(BADAXIS);
	}	

	if (onOffStatus == ON) {
		sprintf(buf, "SH%c", axischar);
		tellGalil(buf);
		if (axis != ZAXIS) {
			usleep(250000);
			brake(axis, OFF);
			usleep(250000);
		}
		return(ON);

	} else if (onOffStatus == OFF) {
		while (isMoving(axis)) {
			}
		if (axis != ZAXIS) {
			usleep(250000);
			brake(axis, ON);
			usleep(250000);
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
		return(UNKNOWN);
	}

}


/*-------------------------------------------------------------------

	move(type) (USER)

	Moves the X-Y stage. type is ABSOLUTE or RELATIVE. For
	absolute moves, the user is requested for the position in
	inches. Relative moves are in motor steps.

-------------------------------------------------------------------*/
void move(type)
int type;
{

	char buf[80];
	long int xoff, yoff;
	float x, y;

	if (type == ABSOLUTE) {
		if (!isHomed()) {
			printf("Not homed\n");
			return;
		}
		printf("Absolute postion X-Y move\n");
		printf("New X position (inches): ");
		fflush(stdout);
		x = atof(gets(buf));
		printf("New Y position (inches): ");
		fflush(stdout);
		y = atof(gets(buf));
		moveAbs(x, y);
		return;

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
		printf("move(%d) type not ABSOLUTE or RELATIVE\n", type);
	}
}

/*-------------------------------------------------------------------

	int moveAbs(float, float) (LIBRARY)

	moveAbs positions the guider stage at coordinates (x,y) where
	x and y are in inches.

	The coordinates are positive numbers referenced to the
	upper right hand corner of the guider patrol space (the
	home position).

	A calibrate() operation must be done before this command.

Minor changes; check it 2012-04-26
-------------------------------------------------------------------*/
int moveAbs(x, y)
float x, y;
{

	long xEncOld, yEncOld, xEncNew, yEncNew, xSteps, ySteps, temp;
	float xPulsPerStep, yPulsPerStep;

	xPulsPerStep = (float) XENCPULSPERTURN / (float) XSTEPSPERTURN;
	yPulsPerStep = (float) YENCPULSPERTURN / (float) YSTEPSPERTURN;

	// save current encoder position
	xEncOld = encPosition(XAXIS);
	yEncOld = encPosition(YAXIS);

	// compute target encoder position
	xEncNew = xEncOffset - (long int) (x * ((float) XENCPULSPERTURN)/XSCREWPITCH);
	yEncNew = yEncOffset - (long int) (y * ((float) YENCPULSPERTURN)/YSCREWPITCH);

	if ((xEncNew > xEncOffset) || (xEncNew < xEncMin)) {
		if (debugFlag) {
			printf("xEncNew out of range (%ld)\n", yEncNew);
		}
		return(0);
	}
	if ((yEncNew > yEncOffset) || (yEncNew < yEncMin)) {
		if (debugFlag) {
			printf("yEncNew out of range (%ld)\n", yEncNew);
		}
		return(0);
	}

	// compute motor steps
	temp = (xEncNew - xEncOld);
	xSteps = (temp >= 0) ? (long int) (((double) (temp + 0.5)) / xPulsPerStep) : (long int) (((double) (temp - 0.5)) / xPulsPerStep);
	temp = (yEncNew - yEncOld);
	ySteps = (temp >= 0) ? (long int) (((double) (temp + 0.5)) / yPulsPerStep) : (long int) (((double) (temp - 0.5)) / yPulsPerStep);

	moveRel(xSteps, ySteps);
	return(1);

}

/*-------------------------------------------------------------------

	void moveOneAxis(int, int, int) (LIBRARY)

	moveOneAxis commands a single-axis motion with the selected
	number of motor steps and speed. Acceleration and deceleration
	values are defaults.

	IMPORTANT NOTE: moveOneAxis turns on the motor power and
	releases the brakes (if any) but does not turn off the motor
	or set the brakes. This must be done after with a call to
	motorPower(axis, onOffStatus), which waits for motion to
	stop, then turns on the brake (if any) and turns off the
	motor.

Checked 2012-04-30
-------------------------------------------------------------------*/
void moveOneAxis(axis, steps, speed)
int axis, steps, speed;
{

	char axischar, buf[20];
	long int acceleration, deceleration;

	acceleration = XYACCEL;
	deceleration = XYDECEL;

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
			acceleration = ZACCEL;
			deceleration = ZDECEL;
			motorPower(ZAXIS, ON);
			break;

		default:
			return;
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

}

/*-------------------------------------------------------------------

	void moveRel(long int, long int) (LIBRARY)

	Moves the X-Y stage to the new position, relative motor
	steps. This could be more elegant, but it works.

Checked 2012-04-30
-------------------------------------------------------------------*/
void moveRel(x, y)
long int x, y;
{

	if (x) {
		moveOneAxis(XAXIS, x, XYSPEED);
	}
	if (y) {
		moveOneAxis(YAXIS, y, XYSPEED);
	}
	motorPower(XAXIS, OFF);
	motorPower(YAXIS, OFF);
}


/*-------------------------------------------------------------------

	void passthru() (USER)

	Talk directly to the Galil controller.

Checked 2012-04-26 (increased string sizes)
-------------------------------------------------------------------*/
void passthru()
{

	char cmd[128], buf[128];

	printf("Galil command\n:");
	fflush(stdout);
	gets(cmd);
	askGalil(cmd, buf, 128);
	printf("%s\n", buf);
	if (buf[strlen(buf) - 1] == '?') {	// Error message from Galil?
		askGalil("TC1", buf, 128);
		printf("%s\n", buf);		// Print TC1 error message
	}
	fflush(stdout);

}

/*-------------------------------------------------------------------

	int selfCheck() (USER)

	selfCheck does a few sanity checks on the guider. It moves
	the three pneumatic cyinders and notices if the GMR sensors
	are correctly triggered and it moves the X-Y axis motors
	to see if the axis encoders report position changes of about
	the correct amount.

Checked 2012-04-26
-------------------------------------------------------------------*/
int selfCheck()
{

	int i, testVal, retVal;
	long int oldEnc;
	float encScale;

	retVal = PASS;
	for (i = 0; i < 2; i++) {
		testVal = cylinder(Y1AXIS, EXTEND);
		if (testVal != 1) {
			retVal = FAIL;
		} 
		testVal = cylinder(Y1AXIS, RETRACT);
		if (testVal != 0) {
			retVal = FAIL;
		}
		testVal = cylinder(Y2AXIS, EXTEND);
		if (testVal != 1) {
			retVal = FAIL;
		}
		testVal = cylinder(Y2AXIS, RETRACT);
		if (testVal != 0) {
			retVal = FAIL;
		}
	}


	if (limitSwitch(XAXIS) || limitSwitch(YAXIS)) {
		printf("selfCheck() says limit switches active (motor cable unplugged?); no motion test");
		fflush(stdout);
	} else {
		oldEnc = encPosition(XAXIS);
		moveRel(500,0);
		encScale = (float) (encPosition(XAXIS) - oldEnc) / 500.0;
		if (fabs(encScale - 4.0) > 0.01) {
			printf("Fail X encoder scale (%7.5f)\n", encScale);
			retVal = FAIL;
		}
		oldEnc = encPosition(YAXIS);
		moveRel(0,500);
		encScale = (float) (encPosition(YAXIS) - oldEnc) / 500.0;
		if (fabs(encScale - 4.0) > 0.01) {
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

	setMode(mode) (USER)

	setMode switches between ICANON and ~ICANON tty modes. This
	is used for the command keys. mode is either NONBLOCKING
	(for one-key command mode) or BLOCKING (for line oriented
	input).

Checked 2012-04-26
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

	resetGalil() (LIBRARY)

	resetGalil sends the RS command to the Galil. This resets
	the controller to its power-on state.
	The Galil sends a character that's not a ':' or '?' so
	the tellGalil() function will return an error message.
	This is normal.

Checked 2012-04-26
-------------------------------------------------------------------*/
void resetGalil()
{

	tellGalil("RS");

}

/*-------------------------------------------------------------------

	int shCam(void) (LIBRARY)

	shCam inserts the Shack-Hartmann lenslet array into the beam.
	shCam inserts the Shack-Hartmann lenslet array into the
	camera beam by extending the Y1 cylinder. It has the opposite
	action of fieldCam().

	On success, this function returns IN. Otherwise, it returns
	what cylinder() reports.

Checked 2012-04-30
-------------------------------------------------------------------*/
int shCam()
{

	int status;

	status = cylinder(Y1AXIS, EXTEND);
	if (status == EXTEND) {
		return(IN);
	} else {
		return(status);
	}

}

/*-------------------------------------------------------------------

	int smallAp(void) (LIBRARY)

	smallAp inserts the small aperture into the camera beam by
	extending the Y2 cylinder. It has the opposite action of
	fieldLens().

	On success, this function returns IN. Otherwise, it returns
	what cylinder() reports.

Checked 2012-04-30
-------------------------------------------------------------------*/
int smallAp()
{

	int status;

	status = cylinder(Y2AXIS, EXTEND);
	if (status == EXTEND) {
		return(IN);
	} else {
		return(status);
	}

}

/*-------------------------------------------------------------------

	void statusPrint(void) (USER)

	statusPrint prints status information. Most values are
	from direct queries to the Galil controller, although some
	items (SAXIS position, for example) no not have a sensor.

-------------------------------------------------------------------*/
void statusPrint()
{

	printf("Status:\n");
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
		printf("Stage position (x,y) %7.3f %7.3f (mm)\n", inchPosition(XAXIS), inchPosition(YAXIS));
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

	printf("Z Limits: ");
	if (limitSwitch(ZAXIS)) {
		if (limitSwitch(ZAXIS) & 0x01) {
			printf("Reverse ");
		}
		if ((limitSwitch(ZAXIS)>>1) & 0x01) {
			printf("Forward ");
		}
	} else {
		printf("Neither active");
	}
	printf("\n");
}


/*-------------------------------------------------------------------

	stepPosition(axis) (LIBRARY)

	stepPosition returns the motor step position of the selected
	axis. Axis may be XAXIS, YAXIS, or ZAXIS.

	The Galil runs stepper motors open-loop so this number is
	just a count of the number of steps accumulated and may not
	reflect the true position if motor steps were lost. This
	number can be reset with the Galil DP command, which is
	done on a homing operation.

	Returns the value of the axis position or BADAXIS.

Checked 2012-04-26
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
			return(BADAXIS);
	}
}
	

/*-------------------------------------------------------------------

	char *tellGalil(char *cmd) (LIBRARY)

	Send a command to the Galil controller. cmd is a NUL
	terminated string containing the Galil command. If the
	Galil returns a '?' instead of a ':' as the first character,
	then this function returns a pointer to a string containing
	the TC1 error message. If it returns neither a '?' or a ':'
	then it returns an error message with the character showing.
	Otherwise, it returns a pointer to a zero length string
	(first byte is '\0').

Checked 2012-04-30
-------------------------------------------------------------------*/
char *tellGalil(cmd)
char *cmd;
{

	uint8_t code;
	static char buf[512];

	askGalil(cmd, buf, 512);
	if (buf[0] == ':') {
		memset(buf, 0, 512);
		return(buf);
	} else if (buf[0] == '?') {
		askGalil("TC1", buf, 512);
		return(buf);
	} else {
		code = (uint8_t) buf[0];
		sprintf(buf, "Unexpected response from Galil (first char = %X)\n", code);
		return(buf);
	}
}

/*-------------------------------------------------------------------

	int telnetToGalil(char *ipaddress) (LIBRARY)

	Opens a socket to the Galil controller at ipaddress.
	ipaddress is a pointer to a string containing the IP
	address of the Galil controller (e.g. "192.168.1.2").
	It returns the file descriptor of the socket or prints
	an error code.

	The ipaddress string must be an IPv4 quartet, not a host name
	(e.g., "192.168.1.2").

	The Galil controller doesn't care about port numbers.
	This routine uses the #define GALILPORT value. The standard
	telnet port is 23 (this is often blocked by routers) and
	the standard Modbus port is 502. You should probably not
	use the Modbus port number even if you know you're not going
	to ask the Galil to communicate on Modbus.

	If telnetToGalil can't open a socket, it returns one of
	the following:

	-1 inet_pton() failed.
	-2 socket() failed.
	-3 connect() failed.

Checked 2012-04-26
-------------------------------------------------------------------*/
int telnetToGalil(ipaddress)
char *ipaddress;
{

	char buf[256];
	int i, fd;
	struct sockaddr_in sockGalil;

	memset(buf, 0, 80);
	memset((char *) &sockGalil, 0, sizeof(sockGalil));
	sockGalil.sin_family = AF_INET;
	sockGalil.sin_port = htons(GALILPORT);
	if (inet_pton(AF_INET, ipaddress, &(sockGalil.sin_addr)) <= 0) {
		return(-1);
	}

	if ((fd = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
		return(-2);
	}

	if (connect(fd, (struct sockaddr *) &sockGalil, sizeof(sockGalil))) {
		return(-3);
	}

	// Clear the Galil output buffer (it's always been empty when I've looked)
	i = write(fd, "\r", 1);
	i = read(fd, buf, 255);
	return(fd);

}

void testFunction()
{


}

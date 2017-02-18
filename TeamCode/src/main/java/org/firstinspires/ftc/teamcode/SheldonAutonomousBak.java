/*
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Sheldon: Two Beacon Autonomous", group = "Robot Position 1")
@Disabled
public class SheldonAutonomousBak extends LinearOpMode {

    /********************************************
     * Declare OpMode Member Variables     *
     ********************************************/

    Robot sheldon = new Robot();                      // Use a Sheldon's hardware

    ModernRoboticsI2cGyro gyro = null;                // Additional Gyro device

    private ElapsedTime runtime = new ElapsedTime();  //Set an instance of a timer

    //Calculate Encder Ticks for Motors
    static final double COUNTS_PER_MOTOR_REV = 1680;  // eg: AndyMark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;   // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;  // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;      // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;       // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;         // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;       // Larger is more responsive, but also less stable


    //These variables are used to determine what team the robot is currently assigned to.
    String TEAM_COLOR = "Red";
    double teamColorCoefficient = 1.0;

    //This variable determines if the beacon was activated
    String beaconActivated = "Not Activated";

    //These variables are used when using the Optical Distance Sensors to find white lines
    double ods1WhiteLineThreshold = 0.36;           // Measure the optical distance sensor 1 value above
    // grey mat then over white line, add together
    // and divide by 2 to get this value

    double ods2WhiteLineThreshold = 0.33;           // Measure the optical distance sensor 2 value above
    // grey mat then over white line, add together
    // and divide by 2 to get this value
    double whiteLineCorrection;

    /***********************
     * End of OpMode Member Variables
     ***************************************/

    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */

        //We had to create a new class to initialize the robot in Autonomous mode because it
        //drives backwards from our normal mode
        sheldon.initializeRobotAutonomous(hardwareMap);

        //Locally assign gyro instead of using sensor map
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("mr_gyro");

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        sheldon.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sheldon.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // Don't move until gyro is calibrated
        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        sheldon.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sheldon.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {

            //Use gamepad1 to assign which team color robot is on x button for blue, b button for red
            if (gamepad1.x == true) {
                TEAM_COLOR = "Blue";
                teamColorCoefficient = -1.0;
            } else if (gamepad1.b == true) {
                TEAM_COLOR = "Red";
                teamColorCoefficient = 1.0;
            }

            //Add telemetry so we can see if our hardware is working properly
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.addData("Current Team Color Selected", TEAM_COLOR);
            telemetry.addData("Left Beacon Servo", sheldon.getLeftBeaconServoStatus());
            telemetry.addData("Right Beacon Servo", sheldon.getRightBeaconServoStatus());
            telemetry.addData("Optical Distance Sensor 1", sheldon.getOpticalDistanceSensor1Status());
            telemetry.addData("Optical Distance Sensor 2", sheldon.getOpticalDistanceSensor2Status());
            telemetry.addData("Optical Distance 1 Normal Light", "%.2f", sheldon.getOpticalDistanceSensor1NormalLightDetected());
            telemetry.addData("Optical Distance 2 Normal Light", "%.2f", sheldon.getOpticalDistanceSensor2NormalLightDetected());
            telemetry.addData("Right Beacon Servo Position", "%.2f", sheldon.rightBeaconServo.getPosition());
            telemetry.addData("Touch Sensor", sheldon.getTouchSensorStatus());
            telemetry.addData("Color Sensor 1", sheldon.getColorSensor1Status());
            telemetry.addData("Color Sensor 2", sheldon.getColorSensor2Status());
            telemetry.update();
            idle();
        }
        gyro.resetZAxisIntegrator();

        /*************************Here is where our Opmode Begins *********************************/

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn

        //When init is pressed, the gyro will calibrate, then the driver coach must assign
        //gamepad1 to the autonomous mode and select the x button if the team color is blue
        //or b if the team color is red (those are the colors of the buttons on the gamepad)
        //This will set the TEAM_COLOR variable and the teamColorCoefficient which will determine
        //turns made regardless of which team is selected.

        gyroDrive(DRIVE_SPEED, 15.0, (teamColorCoefficient * 0.0));   // Step A - Drive 15" on heading of 0 degrees
        gyroTurn(TURN_SPEED, (teamColorCoefficient * 45.0));          // Step B - Turn to a heading of 45 degrees (left or right depending on color)
        gyroHold(TURN_SPEED, (teamColorCoefficient * 45.0), 0.5);     // Step B Continued - Hold the heading and rest for .5 seconds for gyro to settle
        gyroDrive(DRIVE_SPEED, 51.0, (teamColorCoefficient * 45.0));  // Step C -Drive 51" on heading of 45 degrees (left or right depending on color)
        gyroTurn(TURN_SPEED, (teamColorCoefficient * 90.0));          // Step D - Turn to a heading of 90 degrees (left or right depending on color)
        gyroHold(TURN_SPEED, (teamColorCoefficient * 90.0), 0.5);     // Step D Continued - Hold the heading and rest for .5 seconds for gyro to settle
        lookForColorBeacon();                                       // Step E - Drive forward slowly looking for color beacon


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*************************************End of RunOpMode ****************************************/

    /************************************Start of Our Methods *************************************/

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);

            newLeftTarget = sheldon.leftFrontMotor.getCurrentPosition() + moveCounts;
            newRightTarget = sheldon.rightFrontMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            sheldon.leftFrontMotor.setTargetPosition(newLeftTarget);
            sheldon.rightFrontMotor.setTargetPosition(newRightTarget);

            sheldon.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sheldon.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start Motors
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            sheldon.leftFrontMotor.setPower(speed);
            sheldon.leftRearMotor.setPower(speed);
            sheldon.rightFrontMotor.setPower(speed);
            sheldon.rightRearMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (sheldon.leftFrontMotor.isBusy() && sheldon.rightFrontMotor.isBusy())) {

                // Adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;//orig
                rightSpeed = speed + steer;//orig

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                sheldon.leftFrontMotor.setPower(leftSpeed);
                sheldon.rightFrontMotor.setPower(rightSpeed);
                sheldon.leftRearMotor.setPower(leftSpeed);
                sheldon.rightRearMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", sheldon.leftFrontMotor.getCurrentPosition(),
                        sheldon.rightFrontMotor.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all Motors
            sheldon.leftFrontMotor.setPower(0);
            sheldon.leftRearMotor.setPower(0);
            sheldon.rightFrontMotor.setPower(0);
            sheldon.rightRearMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            sheldon.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sheldon.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        sheldon.leftFrontMotor.setPower(0);
        sheldon.leftRearMotor.setPower(0);
        sheldon.rightFrontMotor.setPower(0);
        sheldon.rightRearMotor.setPower(0);
    }

    /**
     * Method to begin turning and try to locate the edge of the white line
     */
    public void findWhiteLine() {
        double drivePower = 0.2;
        if (TEAM_COLOR == "Blue") {
            while (sheldon.getOpticalDistanceSensor1NormalLightDetected() < ods1WhiteLineThreshold) {
                sheldon.driveRobot(drivePower, -drivePower);
            }
        }

        if (TEAM_COLOR == "Red") {
            while (sheldon.getOpticalDistanceSensor2NormalLightDetected() < ods2WhiteLineThreshold) {
                sheldon.driveRobot(-drivePower, drivePower);
            }
        }
    }

    public void followWhiteLine() {
        double minimumSpeed, leftPower, rightPower;
        minimumSpeed = .2;
        while (true) {
            if (TEAM_COLOR == "Blue") {
                whiteLineCorrection = (ods1WhiteLineThreshold - sheldon.getOpticalDistanceSensor1NormalLightDetected());
                if (whiteLineCorrection <= 0) {
                    leftPower = minimumSpeed + whiteLineCorrection;
                    rightPower = minimumSpeed;
                } else {
                    leftPower = minimumSpeed;
                    rightPower = minimumSpeed - whiteLineCorrection;
                }

                sheldon.driveRobot(leftPower, rightPower);

            }
        }
    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        sheldon.leftFrontMotor.setPower(leftSpeed);
        sheldon.leftRearMotor.setPower(leftSpeed);
        sheldon.rightFrontMotor.setPower(rightSpeed);
        sheldon.rightRearMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /**
     * Purpose:  Method to slowly approach beacons and deploy appropriate beacon pusher servo
     * when successful, it will stop.
     */
    public void lookForColorBeacon() {
        double leftApproachSpeed = .2;
        double rightApproachSpeed = .15;

        while (beaconActivated == "Not Activated") {
            if (sheldon.getColorSensor1ColorDetected() == TEAM_COLOR && sheldon.getColorSensor2ColorDetected() != TEAM_COLOR) {
                if (sheldon.leftBeaconServo.getPosition() <= sheldon.leftBeaconServoCalibratedMin) {  //Assume left beacon servo motor is retracted
                    while (sheldon.getColorSensor1ColorDetected() == TEAM_COLOR && sheldon.getColorSensor2ColorDetected() != TEAM_COLOR) {
                        sheldon.deployLeftBeaconServo();
                    }

                    sheldon.retractLeftBeaconServo();
                }

            } else if (sheldon.getColorSensor1ColorDetected() != TEAM_COLOR && sheldon.getColorSensor2ColorDetected() == TEAM_COLOR) {
                if (sheldon.rightBeaconServo.getPosition() <= .16) {  //Assume left beacon servo motor is retracted

                    while (sheldon.getColorSensor1ColorDetected() != TEAM_COLOR && sheldon.getColorSensor2ColorDetected() == TEAM_COLOR) {
                        sheldon.deployRightBeaconServo();
                    }

                    sheldon.retractRightBeaconServo();
                }

            } else if (sheldon.getColorSensor1ColorDetected() == TEAM_COLOR && sheldon.getColorSensor2ColorDetected() == TEAM_COLOR) {
                //If we have made it here, we have successfully tripped the beacon so stop motors
                sheldon.driveRobot(0, 0);
                sheldon.retractLeftBeaconServo();
                sheldon.retractRightBeaconServo();
                beaconActivated = "Successfully Activated!";

            } else {
                //Add code to slowly drive toward beacons continuing to look for beacons
                sheldon.driveRobot(leftApproachSpeed, rightApproachSpeed);
                sheldon.retractLeftBeaconServo();
                sheldon.retractRightBeaconServo();
            }

        }
    }

}// End of Linear Op Mode

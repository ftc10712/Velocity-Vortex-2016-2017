package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * Created by Austin Ford & Tristan Sorenson on 12/15/2016.
 *
 * Purpose: To map and initialize all DC motors on the robot.  The class also contains all of the
 * methods to drive the motors.
 *
 */

class DcMotorHardware {

    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;
    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor leftParticleMotor;
    private DcMotor rightParticleMotor;
    private DcMotor leftMastLiftMotor;
    private DcMotor rightMastLiftMotor;

    //Telemetry Variables
    String leftFrontMotorStatus = "null";
    String leftRearMotorStatus = "null";
    String rightFrontMotorStatus = "null";
    String rightRearMotorStatus = "null";
    String leftParticleMotorStatus = "null";
    String rightParticleMotorStatus = "null";
    String leftMastLiftMotorStatus = "null";
    String rightMastLiftMotorStatus = "null";

    //DCMotorHardware Class Constructor
    DcMotorHardware() {

    }

    /**
     * Called by Robot Class and passed the Hardware Map reference to initialize DC Motors
     *
     * @param hwMap - This is a reference to the hardeware map
     */
    void initializeDCMotors(HardwareMap hwMap) {

        /**
         * Try to map the drive DC motors to the mapped devices on the android phone
         * if they fail, create an entry in the logcat file on the android phone
         * to be viewed later and send set telemetry variable.
         */
        try {
            rightFrontMotor = hwMap.dcMotor.get("right_front_motor");
            rightFrontMotor.setDirection(REVERSE);
            rightFrontMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            rightFrontMotor = null;
            rightFrontMotorStatus = "Failed to Initialize";
        }

        try {
            rightRearMotor = hwMap.dcMotor.get("right_rear_motor");
            rightRearMotor.setDirection(REVERSE);
            rightRearMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            rightRearMotor = null;
            rightRearMotorStatus = "Failed to Initialize";
        }

        try {
            leftFrontMotor = hwMap.dcMotor.get("left_front_motor");
            leftFrontMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            leftFrontMotor = null;
            leftFrontMotorStatus = "Failed to Initialize";
        }

        try {
            leftRearMotor = hwMap.dcMotor.get("left_rear_motor");
            leftRearMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            leftRearMotor = null;
            leftRearMotorStatus = "Failed to Initialize";
        }

        try {
            leftParticleMotor = hwMap.dcMotor.get("left_particle_motor");
            leftParticleMotor.setDirection(REVERSE);
            leftParticleMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            leftParticleMotor = null;
            leftParticleMotorStatus = "Failed to Initialize";
        }

        try {
            rightParticleMotor = hwMap.dcMotor.get("right_particle_motor");
            rightParticleMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            rightParticleMotor = null;
            rightParticleMotorStatus = "Failed to Initialize";
        }

        try {
            leftMastLiftMotor = hwMap.dcMotor.get("left_mast_lift_motor");
            leftMastLiftMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            leftMastLiftMotor = null;
            leftMastLiftMotorStatus = "Failed to Initialize";
        }

        try {
            rightMastLiftMotor = hwMap.dcMotor.get("right_mast_lift_motor");
            rightMastLiftMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            rightMastLiftMotor = null;
            rightMastLiftMotorStatus = "Failed to Initialize";
        }
    }


    void driveRobot(double leftDCMotorPower, double rightDCMotorPower) {
        leftRearMotor.setPower(leftDCMotorPower);
        rightRearMotor.setPower(rightDCMotorPower);
        leftFrontMotor.setPower(leftDCMotorPower);
        rightFrontMotor.setPower(rightDCMotorPower);
    }

    /**
     * Purpose: Toggle the particle shooter motors on or off (To spin the 4" particle shooter wheels
     * <p>
     * Paramter values are currelnt statically passed as the particle shooter is operated with a toggle
     * button.  The particle shooter is either on or off but not a variable speed.
     *
     * @param leftParticleMotorPower  - Variable type double - Can contain values 0.0 - 1.0
     * @param rightParticleMotorPower - Variable type double - Can contain values 0.0 - 1.0
     */
    void toggleParticleShooterMotors(double leftParticleMotorPower, double rightParticleMotorPower) {
        leftParticleMotor.setPower(leftParticleMotorPower);
        rightParticleMotor.setPower(rightParticleMotorPower);
    }

    /**
     * Purpose:  Raise and lower the forklift mast
     *
     * @param mastLiftDCMotorPower - Uses the value of the triggers on gamepad 1 - Can be 0.0 - 1.0
     *
     * Since parameter values are passed by the triggers, the value of the parameter can vary
     *between 0.0 - 1.0, it is determined on how far the trigger is pressed.
     */
    void driveMastLift(float mastLiftDCMotorPower) {
        leftMastLiftMotor.setPower(mastLiftDCMotorPower);
        rightMastLiftMotor.setPower(mastLiftDCMotorPower);
    }

}

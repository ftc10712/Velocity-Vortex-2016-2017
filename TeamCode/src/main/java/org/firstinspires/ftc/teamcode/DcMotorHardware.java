package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * Created by Austin Ford & Tristan Sorenson on 12/15/2016.
 *
 */


class DcMotorHardware {

    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;
    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor leftParticleMotor;
    private DcMotor rightParticleMotor;
    private DcMotor mastLiftMotor;

    //Telemetry Variables
    String leftFrontMotorStatus = "null";
    String leftRearMotorStatus = "null";
    String rightFrontMotorStatus = "null";
    String rightRearMotorStatus = "null";
    String leftParticleMotorStatus = "null";
    String rightParticleMotorStatus = "null";
    String mastLiftMotorStatus = "null";

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
            mastLiftMotor = hwMap.dcMotor.get("mast_lift_motor");
            mastLiftMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            mastLiftMotor = null;
            mastLiftMotorStatus = "Failed to Initialize";
        }
    }


    void driveRobot(double leftDCMotorPower, double rightDCMotorPower) {
        leftRearMotor.setPower(leftDCMotorPower);
        rightRearMotor.setPower(rightDCMotorPower);
        leftFrontMotor.setPower(leftDCMotorPower);
        rightFrontMotor.setPower(rightDCMotorPower);
    }

    void toggleParticleShooterMotors(double leftParticleMotorPower, double rightParticleMotorPower) {
        leftParticleMotor.setPower(leftParticleMotorPower);
        rightParticleMotor.setPower(rightParticleMotorPower);
    }

    void driveMastLift(float mastLiftDCMotorPower) {
        mastLiftMotor.setPower(mastLiftDCMotorPower);
    }

}

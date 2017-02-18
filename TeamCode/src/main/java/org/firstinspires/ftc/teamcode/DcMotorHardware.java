package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * Created by Austin Ford & Tristan Sorenson on 12/15/2016.
 * <p>
 * Purpose: To map and initialize all DC motors on the robot.  The class also contains all of the
 * methods to drive the motors.
 */

class DcMotorHardware {

    protected DcMotor rightFrontMotor;
    protected DcMotor rightRearMotor;
    protected DcMotor leftFrontMotor;
    protected DcMotor leftRearMotor;
    protected DcMotor leftForkGripperMotor;
    protected DcMotor rightForkGripperMotor;
    private DcMotor leftMastLiftMotor;
    private DcMotor rightMastLiftMotor;

    //Telemetry Variables
    String leftFrontMotorStatus = "null";
    String leftRearMotorStatus = "null";
    String rightFrontMotorStatus = "null";
    String rightRearMotorStatus = "null";
    String leftForkGripperMotorStatus = "null";
    String rightForkGripperMotorStatus = "null";
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
            leftForkGripperMotor = hwMap.dcMotor.get("left_fork_gripper_motor");
            leftForkGripperMotor.setDirection(REVERSE);
            leftForkGripperMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            leftForkGripperMotor = null;
            leftForkGripperMotorStatus = "Failed to Initialize";
        }

        try {
            rightForkGripperMotor = hwMap.dcMotor.get("right_fork_gripper_motor");
            rightForkGripperMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            rightForkGripperMotor = null;
            rightForkGripperMotorStatus = "Failed to Initialize";
        }

        try {
            leftMastLiftMotor = hwMap.dcMotor.get("left_mast_lift_motor");
            //leftMastLiftMotor.setDirection(REVERSE);
            leftMastLiftMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            leftMastLiftMotor = null;
            leftMastLiftMotorStatus = "Failed to Initialize";
        }

        try {
            rightMastLiftMotor = hwMap.dcMotor.get("right_mast_lift_motor");
            //rightMastLiftMotor.setDirection(REVERSE);
            rightMastLiftMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            rightMastLiftMotor = null;
            rightMastLiftMotorStatus = "Failed to Initialize";
        }
    }

    //This changes the motors for Autonomous Mode
    void initializeDCMotorsAutonomous(HardwareMap hwMap) {

        /**
         * Try to map the drive DC motors to the mapped devices on the android phone
         * if they fail, create an entry in the logcat file on the android phone
         * to be viewed later and send set telemetry variable.
         */
        try {
            rightFrontMotor = hwMap.dcMotor.get("left_rear_motor");
            rightFrontMotor.setDirection(REVERSE);
            rightFrontMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            rightFrontMotor = null;
            rightFrontMotorStatus = "Failed to Initialize";
        }

        try {
            rightRearMotor = hwMap.dcMotor.get("left_front_motor");
            rightRearMotor.setDirection(REVERSE);
            rightRearMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            rightRearMotor = null;
            rightRearMotorStatus = "Failed to Initialize";
        }

        try {
            leftFrontMotor = hwMap.dcMotor.get("right_rear_motor");
            leftFrontMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            leftFrontMotor = null;
            leftFrontMotorStatus = "Failed to Initialize";
        }

        try {
            leftRearMotor = hwMap.dcMotor.get("right_front_motor");
            leftRearMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            leftRearMotor = null;
            leftRearMotorStatus = "Failed to Initialize";
        }

        try {
            leftForkGripperMotor = hwMap.dcMotor.get("left_fork_gripper_motor");
            leftForkGripperMotor.setDirection(REVERSE);
            leftForkGripperMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            leftForkGripperMotor = null;
            leftForkGripperMotorStatus = "Failed to Initialize";
        }

        try {
            rightForkGripperMotor = hwMap.dcMotor.get("right_fork_gripper_motor");
            rightForkGripperMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            rightForkGripperMotor = null;
            rightForkGripperMotorStatus = "Failed to Initialize";
        }

        try {
            leftMastLiftMotor = hwMap.dcMotor.get("left_mast_lift_motor");
            //leftMastLiftMotor.setDirection(REVERSE);
            leftMastLiftMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            leftMastLiftMotor = null;
            leftMastLiftMotorStatus = "Failed to Initialize";
        }

        try {
            rightMastLiftMotor = hwMap.dcMotor.get("right_mast_lift_motor");
            //rightMastLiftMotor.setDirection(REVERSE);
            rightMastLiftMotorStatus = "Initialized";
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            rightMastLiftMotor = null;
            rightMastLiftMotorStatus = "Failed to Initialize";
        }
    }
    //Method to Drive Robot
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
     * @param forkGripperMotorPower  - Variable type double - Can contain values 0.0 - 1.0
     * @param forkGripperMotorPower - Variable type double - Can contain values 0.0 - 1.0
     */
    void driveForkGripperMotors(float forkGripperMotorPower) {
        leftForkGripperMotor.setPower(forkGripperMotorPower);
        rightForkGripperMotor.setPower(forkGripperMotorPower);
    }

    /**
     * Purpose:  Raise and lower the forklift mast
     *
     * @param mastLiftDCMotorPowerUp - Uses the value of the triggers on gamepad 1 - Can be 0.0 - 1.0
     *                               <p>
     *                               Since parameter values are passed by the triggers, the value of the parameter can vary
     *                               between 0.0 - 1.0, it is determined on how far the trigger is pressed.
     */
    void driveMastLiftPower(float mastLiftDCMotorPowerUp) {
        leftMastLiftMotor.setPower(mastLiftDCMotorPowerUp);
        rightMastLiftMotor.setPower(mastLiftDCMotorPowerUp);
    }

    double convertInchesToEncoderTicks(double numberOfInches) {
        final double COUNTS_PER_MOTOR_REV = 1680;    // eg: AndyMark Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        return COUNTS_PER_INCH * numberOfInches;
    }

    /**
     * Purpose: Reset Encoders on Rear Motors
     */
    void resetEncoders() {
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Purpose: Set Motors to Run With Encoders
     */
    void runUsingEncoders() {
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Scale Motor Power so robot speed is more manageable
    float scalePower(float power) {

        float l_scale = 0.0f;

        float l_power = Range.clip(power, -1, 1);

        float[] l_array =
                {0.00f, 0.03f, 0.06f, 0.9f, 0.12f
                        , 0.15f, 0.18f, 0.21f, 0.24f, 0.27f
                        , 0.30f, 0.35f, 0.50f, 0.65f, 0.85f
                        , 1.00f, 1.00f
                };

        int l_index = (int) (l_power * 16.0);
        if (l_index < 0) {
            l_index = -l_index;
        } else if (l_index > 16) {
            l_index = 16;
        }

        if (l_power < 0) {
            l_scale = -l_array[l_index];
        } else {
            l_scale = l_array[l_index];
        }

        return l_scale;

    } // scale_motor_power
}

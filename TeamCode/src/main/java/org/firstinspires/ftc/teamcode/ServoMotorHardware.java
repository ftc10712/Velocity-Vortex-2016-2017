package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * ServoMotorHardware class maps and initializes the robot servos and provides methods
 * to drive the servos.
 */
class ServoMotorHardware extends DcMotorHardware {

    //Member Variables for Servos on Servo Controller AI02QSMX
    private CRServo leftForkGripperServo;
    private CRServo rightForkGripperServo;

    private Servo leftForkServo;
    float leftForkServoCalibratedMin = 0f;
    float leftForkServoCalibratedMax = .94f;     //Modern Robotics updated firmware to go beyond
    //130 degrees.  This calibrates the servo to turn
    //180 degrees. Change min and max to suit servos

    private Servo rightForkServo;
    float rightForkServoCalibratedMin = .96f;
    float rightForkServoCalibratedMax = 0f;    //Modern Robotics updated firmware to go beyond
    //130 degrees.  This calibrates the servo to turn
    //180 degrees.  Change min and max to suit servos

    //private CRServo leftMastHorizontalServo;
    //private CRServo rightMastHorizontalServo;



    //Member Variables for Servos on Servo Controller A104ORQT

    protected Servo leftBeaconServo;
    float leftBeaconServoCalibratedMax = 0f;
    float leftBeaconServoCalibratedMin = .67f;


    protected Servo rightBeaconServo;
    float rightBeaconServoCalibratedMin = .14f;
    float rightBeaconServoCalibratedMax = .85f;


    //ServoMotorHardware Class Constructor
    ServoMotorHardware() {

    }

    /**
     * Try to map the servo motors to the mapped devices on the android phone
     * if they fail, create an entry in the logcat file on the android phone
     * to be viewed later.
     *
     * @param hwMap - a reference to the hardware map
     */
    void initializeServoMotors(HardwareMap hwMap) {


        //------------------Servos Assigned to Servo Controller AI02QSMX-------------------------//

        //Gripper Servo Motor Initialization - These are Continuous Rotation Servos
        try {
            leftForkGripperServo = hwMap.crservo.get("left_fork_gripper_servo");
            leftForkGripperServo.setDirection(FORWARD);
        } catch (Exception errorMessage) {
            leftForkGripperServo = null;
        }

        try {
            rightForkGripperServo = hwMap.crservo.get("right_fork_gripper_servo");
            rightForkGripperServo.setDirection(REVERSE);
        } catch (Exception errorMessage) {
            rightForkGripperServo = null;
        }

        //Fork Deploy Servo Motor Initialization - These are standard 180 degree Servos
        try {
            leftForkServo = hwMap.servo.get("left_fork_deploy_servo");
            leftForkServo.setPosition(leftForkServoCalibratedMin);
        } catch (Exception errorMessage) {
            leftForkServo = null;
        }

        try {
            rightForkServo = hwMap.servo.get("right_fork_deploy_servo");
            rightForkServo.setPosition(rightForkServoCalibratedMin);
        } catch (Exception errorMessage) {
            rightForkServo = null;
        }

        //Mast Servo Motor Initialization - These are Continuous Rotation Servos
//        try {
//            leftMastHorizontalServo = hwMap.crservo.get("left_mast_horizontal_servo");
//            leftMastHorizontalServo.setDirection(FORWARD);
//        } catch (Exception errorMessage) {
//            leftMastHorizontalServo = null;
//        }
//
//        try {
//            rightMastHorizontalServo = hwMap.crservo.get("right_mast_horizontal_servo");
//            rightMastHorizontalServo.setDirection(REVERSE);
//        } catch (Exception errorMessage) {
//            rightMastHorizontalServo = null;
//        }

        //---------------End Servos Assigned to Servo Controller AI02QSMX-------------------------//

        //------------------Servos Assigned to Servo Controller A104ORQT_-------------------------//
        try {
            leftBeaconServo = hwMap.servo.get("left_beacon_servo");
            leftBeaconServo.setPosition(leftBeaconServoCalibratedMin);
        } catch (Exception errorMessage) {
            leftBeaconServo = null;
        }

        try {
            rightBeaconServo = hwMap.servo.get("right_beacon_servo");
            rightBeaconServo.setPosition(rightBeaconServoCalibratedMin);
        } catch (Exception errorMessage) {
            rightBeaconServo = null;
        }


        //---------------End Servos Assigned to Servo Controller A104ORQT-------------------------//


    } // end of initializeServoMotors() Method


    /**
     * forkGripperOpen Method opens the forks when called
     *
     * @param gripperServoSpeed      - Speed at which to move the gripper servos
     * @param directionToMoveGripper - Given as either OPEN or CLOSE
     */

    void forkGripperMove(String directionToMoveGripper, double gripperServoSpeed) {

        if (directionToMoveGripper.equals("OPEN")) {

            leftForkGripperServo.setPower(gripperServoSpeed);
            rightForkGripperServo.setPower(gripperServoSpeed);

        } else if (directionToMoveGripper.equals("CLOSE")) {
            leftForkGripperServo.setPower(-gripperServoSpeed);
            rightForkGripperServo.setPower(-gripperServoSpeed);
        }
    }

    void forkGripperStop() {
        leftForkGripperServo.setPower(0);
        rightForkGripperServo.setPower(0);
    }

//    //Method to move the mast servos forward and reverse
//    void mastServoMove(String directionToMove, double mastServoSpeed) {
//
//        if (directionToMove.equals("FORWARD")) {
//
//            leftMastHorizontalServo.setPower(mastServoSpeed);
//            rightMastHorizontalServo.setPower(mastServoSpeed - .1);
//
//        } else if (directionToMove.equals("REVERSE")) {
//
//            leftMastHorizontalServo.setPower(-mastServoSpeed);
//            rightMastHorizontalServo.setPower(-mastServoSpeed - .1);
//        }
//    }
//
//    void mastServoStop() {
//        leftMastHorizontalServo.setPower(0);
//        rightMastHorizontalServo.setPower(0);
//    }


    //Method to check the status of left and right beacon servos
    String getLeftBeaconServoStatus() {
        String leftBeaconServoStatus;
        if (leftBeaconServo != null) {
            leftBeaconServoStatus = "Initialized";
        } else {
            leftBeaconServoStatus = "Failed to Initialize";
        }
        return leftBeaconServoStatus;
    }

    String getRightBeaconServoStatus() {
        String rightBeaconServoStatus;
        if (leftBeaconServo != null) {
            rightBeaconServoStatus = "Initialized";
        } else {
            rightBeaconServoStatus = "Failed to Initialize";
        }
        return rightBeaconServoStatus;
    }

    //Methods to Deploy Beacon Servos
    void deployLeftBeaconServo() {
        leftBeaconServo.setPosition(leftBeaconServoCalibratedMax);
    }

    void deployRightBeaconServo() {
        rightBeaconServo.setPosition(rightBeaconServoCalibratedMax);
    }

    //Methods to Retract Servo Beacons
    void retractLeftBeaconServo() {
        leftBeaconServo.setPosition(leftBeaconServoCalibratedMin);
    }

    void retractRightBeaconServo() {
        rightBeaconServo.setPosition(rightBeaconServoCalibratedMin);
    }

    //Methods to Deploy Fork Blade Servos
    void deployLeftForkBladeServo() {
        leftForkServo.setPosition(leftForkServoCalibratedMax);
    }

    void deployRightForkBladeServo() {
        rightForkServo.setPosition(rightForkServoCalibratedMin);
    }

    //Methods to Retract Fork Blade Servos
    void retractLeftForkBladeServo() {
        leftForkServo.setPosition(leftForkServoCalibratedMin);
    }

    void retractRightForkBladeServo() {
        rightForkServo.setPosition(leftForkServoCalibratedMax);
    }

}


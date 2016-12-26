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
    private Servo leftForkDeployServo;
    private Servo rightForkDeployServo;

    //Member Variables for Servos on Servo Controller A104ORQT
    protected Servo leftBeaconServo;
    protected Servo rightBeaconServo;

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
            leftForkDeployServo = hwMap.servo.get("left_fork_deploy_servo");
        } catch (Exception errorMessage) {
            leftForkDeployServo = null;
        }

        try {
            rightForkDeployServo = hwMap.servo.get("right_fork_deploy_servo");
        } catch (Exception errorMessage) {
           rightForkDeployServo = null;
        }

        //---------------End Servos Assigned to Servo Controller AI02QSMX-------------------------//

        //------------------Servos Assigned to Servo Controller A104ORQT_-------------------------//
        try {
            leftBeaconServo = hwMap.servo.get("left_beacon_servo");
            leftBeaconServo.setPosition(0);
        } catch (Exception errorMessage) {
            leftBeaconServo = null;
        }

        try {
            rightBeaconServo = hwMap.servo.get("right_beacon_servo");
            rightBeaconServo.setPosition(0);
        } catch (Exception errorMessage) {
            rightBeaconServo = null;
        }


        //---------------End Servos Assigned to Servo Controller A104ORQT-------------------------//


    } // end of initializeServoMotors() Method


    /**
     * forkGripperOpen Method opens the forks when called
     *
     * @param gripperSpeed    - Speed at which to move the gripper servos
     * @param directionToMove - Given as either OPEN or CLOSE
     */

    void forkGripperMove(String directionToMove, double gripperSpeed) {

        if (directionToMove.equals("OPEN")) {

            leftForkGripperServo.setPower(gripperSpeed);
            rightForkGripperServo.setPower(-gripperSpeed);

        } else if (directionToMove.equals("CLOSE")) {
            leftForkGripperServo.setPower(-gripperSpeed);
            rightForkGripperServo.setPower(gripperSpeed);
        }
    }

    void forkGripperStop() {
        leftForkGripperServo.setPower(0);
        rightForkGripperServo.setPower(0);
    }


    /*


    public void forkGripperOpen(double speed) {

        double GRIPPER_SPEED = speed;

        if (leftForkGripperServo != null) {
            leftForkGripperServo.setDirection(FORWARD);
            leftForkGripperServo.setPower(GRIPPER_SPEED);
        }

        if (rightForkGripperServo != null) {
            rightForkGripperServo.setDirection(REVERSE);
            rightForkGripperServo.setPower(GRIPPER_SPEED);
        }

    }

    /**
     * forkGripperClose Method closes the forks when called
     * @param speed
     */

    /*
    public void forkGripperClose(double speed) {

        double GRIPPER_SPEED = speed;

        if (leftForkGripperServo != null) {
            leftForkGripperServo.setDirection(FORWARD);
            leftForkGripperServo.setPower(GRIPPER_SPEED);
        }

        if (rightForkGripperServo != null) {
            rightForkGripperServo.setDirection(REVERSE);
            rightForkGripperServo.setPower(GRIPPER_SPEED);
        }

    }


    */

    //Method to check the status of left and right beacon servos
    String getLeftBeaconServoStatus(){
        String leftBeaconServoStatus;
        if(leftBeaconServo != null){
            leftBeaconServoStatus = "Initialized";
        } else {
            leftBeaconServoStatus = "Failed to Initialize";
        }
        return leftBeaconServoStatus;
    }

    String getRightBeaconServoStatus(){
        String rightBeaconServoStatus;
        if(leftBeaconServo != null){
            rightBeaconServoStatus = "Initialized";
        } else {
            rightBeaconServoStatus = "Failed to Initialize";
        }
        return rightBeaconServoStatus;
    }

    //Sample Test Code not production
    void deployLeftBeaconServo(){
        leftBeaconServo.setPosition(.9);
    }

    void retractLeftBeaconServo(){
        leftBeaconServo.setPosition(0);
    }

    void deployRightBeaconServo(){
        rightBeaconServo.setPosition(.92);
    }

    void retractRightBeaconServo(){
        rightBeaconServo.setPosition(0);
    }

}


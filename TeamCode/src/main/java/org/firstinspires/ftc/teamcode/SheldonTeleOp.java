package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Austin Ford & Tristan Sorenson on 12/19/2016.
 *
 * Description: FTC10712 TeleOP Program to control Velocity Vortex Robot Sheldon
 */

@TeleOp(name = "TeleOp: Sheldon", group = "TeleOP")

public class SheldonTeleOp extends OpMode {

    Robot sheldon = new Robot();            //New instance of our Robot Class we call Sheldon

    //Drive Direction Variables
    double leftDCMotorPower = 0;            //Power Value for Left DC Drive Motors
    double rightDCMotorPower = 0;           //Power Value for Right DC Drive Motors
    boolean toggleDriveDirection = false;   //Toggle which side of the robot will drive forward
    boolean driveDirection = false;         //Toggle direction of motor power when direction toggled
    String frontSideOfRobot = "Forklift";   //Default front side of robot is Forklift Side

    //Mast Servo Variables
    String mastDirectionToMove = "Not Initialized";
    double mastServoSpeed = 1.0;            //Set the speed at which you want the mast servos to move
    //can be 0.0-1.0 range

    //Gripper Servo Variables
    String gripperDirectionToMove = "Not Initialized";
    double gripperServoSpeed = 1.0;         //Set the speed at which you want the mast servos to move
    //can be 0.0-1.0 range

    //Fork Servo Variables
    boolean toggleForkDeploy = false;        //Holds current state of fork deploy status
    boolean forkDeployStatus = false;    //Checks to see if a button on joystick was pressed
    String positionOfForks = "Retracted";   //Default front side of robot is Forklift Side

    @Override
    public void init() {
        sheldon.initializeRobot(hardwareMap);

        //telemetry.addData("Left Beacon Servo ", sheldon.getLeftBeaconServoStatus());
        //telemetry.addData("Right Beacon Servo ", sheldon.getRightBeaconServoStatus());
        //telemetry.addData("Touch Sensor ", sheldon.getTouchSensorStatus());
        //telemetry.addData("Color Sensor 1 ", sheldon.getColorSensor1Status());
        updateTelemetry(telemetry);
    }

    @Override
    public void loop() {

        /********************** Start of Driving Routine******************************/

     /* This routine toggles the side of the robot that is considered forward facing.  When
        operating the forklift side of the robot, we want that side to be forward facing.
        When we are shooting balls, we want the particle shooter side of the robot to be forward
        facing.
     */
        if (gamepad1.left_stick_button && !toggleDriveDirection) {

            //  If left joystick button is pressed, change direction of the front of the robot from
            //  it currently was set.  The inital side considered forward is the forklift side of
            //  the robot.

            driveDirection = !driveDirection;

            if (driveDirection) {
                //Set the front of the robot to be the Particle Shooter side of the robot.
                frontSideOfRobot = "Particle Shooter";  //Telemetry data variable

            } else {
                //Set the front of the robot to be the Forklift side of the robot.
                frontSideOfRobot = "Forklift";      //Telemetry data variable
            }

            toggleDriveDirection = true;

        } else if (!gamepad1.left_stick_button) {
            toggleDriveDirection = false;

        }
        //Drive the robot in the direction selected
        if (frontSideOfRobot == "Particle Shooter") {

            leftDCMotorPower = gamepad1.left_stick_y;
            rightDCMotorPower = gamepad1.right_stick_y;
            sheldon.driveRobot(leftDCMotorPower, rightDCMotorPower);

        } else if (frontSideOfRobot == "Forklift") {

            leftDCMotorPower = -gamepad1.left_stick_y;
            rightDCMotorPower = -gamepad1.right_stick_y;
            sheldon.driveRobot(leftDCMotorPower, rightDCMotorPower);
        }
        /**********************End of Driving Routine************************/


        /**********************Start of Gripper Moving Routine***************/
        if (gamepad1.right_bumper) {
            gripperDirectionToMove = "OPEN";
            sheldon.forkGripperMove(gripperDirectionToMove, gripperServoSpeed);
        } else if (gamepad1.left_bumper) {
            gripperDirectionToMove = "CLOSE";
            sheldon.forkGripperMove(gripperDirectionToMove, gripperServoSpeed);
        } else {
            sheldon.forkGripperStop();
        }
        /**********************End of Gripper Moving Routine*****************/

        /**********************Start of Mast Moving Forward or Reverse Routine******************/
        if (gamepad1.dpad_up) {
            mastDirectionToMove = "FORWARD";
            sheldon.mastServoMove(mastDirectionToMove, mastServoSpeed);
        } else if (gamepad1.dpad_down) {
            mastDirectionToMove = "REVERSE";
            sheldon.mastServoMove(mastDirectionToMove, mastServoSpeed);
        } else {
            sheldon.mastServoStop();
        }
        /**********************End of Mast Moving Forward or Reverse Routine********************/

        /**********************Fork Deploy & Retract Routine************************************/
        if (gamepad1.a && !toggleForkDeploy) {

            //  If left joystick button is pressed, change direction of the front of the robot from
            //  it currently was set.  The inital side considered forward is the forklift side of
            //  the robot.

            forkDeployStatus = !forkDeployStatus;

            if (forkDeployStatus) {
                //Set the front of the robot to be the Particle Shooter side of the robot.
                positionOfForks = "Retracted";  //Telemetry data variable

            } else {
                //Set the front of the robot to be the Forklift side of the robot.
                positionOfForks = "Deployed";      //Telemetry data variable
            }

            toggleForkDeploy = true;

        } else if (!gamepad1.a) {
            toggleForkDeploy = false;

        }
        //Drive the robot in the direction selected
        if (positionOfForks == "Deployed") {
            sheldon.deployLeftForkBladeServo();
            sheldon.deployRightForkBladeServo();

        } else if (positionOfForks == "Retracted") {
            sheldon.retractLeftForkBladeServo();
            sheldon.retractRightForkBladeServo();
        }
        /**********************End Fork Deploy & Retract Routine********************************/

        telemetry.addData("Left Joystick Button", toggleDriveDirection);
        telemetry.addData("Motor Direction", driveDirection);
        telemetry.addData("Front Side Facing", frontSideOfRobot);
        updateTelemetry(telemetry);
    }
}

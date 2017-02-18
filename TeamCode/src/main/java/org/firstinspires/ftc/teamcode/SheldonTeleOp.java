package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Austin Ford & Tristan Sorenson on 12/19/2016.
 *
 * Description: FTC10712 TeleOP Program to control Velocity Vortex Robot Sheldon
 */

@TeleOp(name = "TeleOp: Sheldon", group = "TeleOP")
//@Disabled

public class SheldonTeleOp extends OpMode {

    private Robot sheldon = new Robot();            //New instance of our Robot Class we call Sheldon
    private boolean toggleLeftJoystickButtonStatus = false;
    private boolean toggleLeftJoystickButton = false;
    private String frontSideOfRobot = "Forklift";   //Default front side of robot is Forklift Side
    private String frontSideOfRobotStatus = "Forklift"; //Telemetry Variable

    //Power variables for lifting the mast
    private float mastLiftPowerUp = 1.0f;
    private float mastLiftPowerDown = -1.0f;
    private float mastLiftPowerStop = 0.0f;


    @Override
    public void init() {
        sheldon.initializeRobot(hardwareMap);

        //Telemetry to report if all of our hardware was recognized and initialized
        //when the init button is pressed.
        telemetry.addData("Left Beacon Servo", sheldon.getLeftBeaconServoStatus());
        telemetry.addData("Right Beacon Servo", sheldon.getRightBeaconServoStatus());
        telemetry.addData("Left Front DC Motor", sheldon.leftFrontMotorStatus);
        telemetry.addData("Left Rear DC Motor", sheldon.leftRearMotorStatus);
        telemetry.addData("Right Front DC Motor", sheldon.rightFrontMotorStatus);
        telemetry.addData("Right Rear DC Motor", sheldon.rightRearMotorStatus);
        telemetry.addData("Left Particle DC Motor", sheldon.leftForkGripperMotorStatus);
        telemetry.addData("Right Particle DC Motor", sheldon.rightForkGripperMotorStatus);
        telemetry.addData("Left Mast Lift Motor", sheldon.leftMastLiftMotorStatus);
        telemetry.addData("Right Mast Lift Motor", sheldon.rightMastLiftMotorStatus);

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

        double leftDCMotorPower;            //Power Value for Left DC Drive Motors
        double rightDCMotorPower;           //Power Value for Right DC Drive Motors

        //Code to toggle which side of Robot is Facing Forward
        if (gamepad1.left_stick_button && !toggleLeftJoystickButton) {
            toggleLeftJoystickButton = true;
            toggleLeftJoystickButtonStatus = !toggleLeftJoystickButtonStatus;

            if (toggleLeftJoystickButtonStatus) {

                frontSideOfRobot = "Particle Shooter";  //Telemetry data variable

            } else {

                frontSideOfRobot = "Forklift";      //Telemetry data variable
            }

        } else if (!gamepad1.left_stick_button) {

            toggleLeftJoystickButton = false;

        }

        //Drive the robot in the direction selected either Particle Shooter forward or Forklift Forward
        if (frontSideOfRobot == "Particle Shooter") {
            frontSideOfRobotStatus = "Paricle Shooter";
            leftDCMotorPower = sheldon.scalePower(gamepad1.left_stick_y);
            rightDCMotorPower = sheldon.scalePower(gamepad1.right_stick_y);
            sheldon.driveRobot(leftDCMotorPower, rightDCMotorPower);

        } else if (frontSideOfRobot == "Forklift") {
            frontSideOfRobotStatus = "Forklift";
            leftDCMotorPower = sheldon.scalePower(-gamepad1.left_stick_y);
            rightDCMotorPower = sheldon.scalePower(-gamepad1.right_stick_y);
            sheldon.driveRobot(leftDCMotorPower, rightDCMotorPower);
        }
        /**********************End of Driving Routine************************/


        /**********************Start of Gripper Moving Routine***************/
        //Gripper Motor Power Variables
        //String gripperDirectionToMove = "Not Initialized";
        float forkGripperOpenPower = 0.1f; //Set open power
        float forkGripperClosePower = -0.1f; //Set open power
        float forkGripperStopPower = 0; //Set Power to Stop

        if (gamepad1.right_bumper) {
            sheldon.driveForkGripperMotors(forkGripperOpenPower);
        } else if (gamepad1.left_bumper) {
            //Close Grippers
            sheldon.driveForkGripperMotors(forkGripperClosePower);
        } else {
            sheldon.driveForkGripperMotors(forkGripperStopPower);
        }
        /**********************End of Gripper Moving Routine*****************/

        /**********************Fork Deploy & Retract Routine************************************/

        //Fork Servo Variables
        String statusOfForks = "Retracted";

        if (gamepad1.b) {
            sheldon.deployLeftForkBladeServo();
            sheldon.deployRightForkBladeServo();
            statusOfForks = "Deployed";     //Sets Telemetry Variable

        } else if (gamepad1.x) {
            sheldon.retractLeftForkBladeServo();
            sheldon.retractRightForkBladeServo();
            statusOfForks = "Retracted";    //Sets Telemetry Variable
        }

        /**********************End Fork Deploy & Retract Routine********************************/

        /********************* Start Raise & Lower Mast Routine ********************************/

        if (gamepad1.y) {
            sheldon.driveMastLiftPower(mastLiftPowerUp);
            //sheldon.driveForkGripperMotors(.1f);
        } else if (gamepad1.a) {
            sheldon.driveMastLiftPower(mastLiftPowerDown);
        } else {
            sheldon.driveMastLiftPower(mastLiftPowerStop);
        }
        /**************************** End of Raise Mast Routine ***********************************/

        /**********************Beacon Servo Deploy & Retract Routine*******************************/

        //Fork Servo Variables
        //String statusOfForks = "Retracted";

        if (gamepad2.right_bumper) {
            sheldon.deployLeftBeaconServo();
        } else {
            sheldon.retractLeftBeaconServo();
        }

        if (gamepad2.left_bumper) {
            sheldon.deployRightBeaconServo();
        } else {
            sheldon.retractRightBeaconServo();
        }

        /********************End of Beacon Servo Deploy & Retract Routine**************************/

        /*******************Telemetry for TeleOp Mode while Running ******************************/
        telemetry.addData("Front Side Facing", frontSideOfRobotStatus);
        telemetry.addData("Position of Forks", statusOfForks);
        //telemetry.addData("Direction Mast Moving", mastDirectionToMove);
        //telemetry.addData("Particle Shooter Motors", particleShooterStatus);
        updateTelemetry(telemetry);
        /***************End of Telemetry for TeleOp Mode while Running ***************************/

    }
}

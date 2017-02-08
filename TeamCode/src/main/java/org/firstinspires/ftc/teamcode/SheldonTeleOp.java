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
@Disabled

public class SheldonTeleOp extends OpMode {

    private Robot sheldon = new Robot();            //New instance of our Robot Class we call Sheldon
    private String frontSideOfRobot = "Forklift";   //Default front side of robot is Forklift Side
    private String frontSideOfRobotStatus = "Forklift"; //Telemetry Variable
    private String particleShooterStatus = "Off";
    private String particleShooterToggle = "Off";


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
        telemetry.addData("Left Particle DC Motor", sheldon.leftParticleMotorStatus);
        telemetry.addData("Right Particle DC Motor", sheldon.rightParticleMotorStatus);
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

        //Call method in JoystickHardware Class to toggle a button on joystick
        frontSideOfRobot = sheldon.toggleJoystickButton(gamepad1.left_stick_button, "Particle Shooter", "Forklift");

        //Drive the robot in the direction selected either Particle Shooter forward or Forklift Forward
        if (frontSideOfRobot == "Particle Shooter") {
            frontSideOfRobotStatus = "Paricle Shooter";
            leftDCMotorPower = gamepad1.left_stick_y;
            rightDCMotorPower = gamepad1.right_stick_y;
            sheldon.driveRobot(leftDCMotorPower, rightDCMotorPower);

        } else if (frontSideOfRobot == "Forklift") {
            frontSideOfRobotStatus = "Forklift";
            leftDCMotorPower = -gamepad1.left_stick_y;
            rightDCMotorPower = -gamepad1.right_stick_y;
            sheldon.driveRobot(leftDCMotorPower, rightDCMotorPower);
        }
        /**********************End of Driving Routine************************/


        /**********************Start of Gripper Moving Routine***************/

        //Gripper Servo Variables
        String gripperDirectionToMove = "Not Initialized";
        double gripperServoSpeed = 1.0; //Set Gripper Servo Speed 0.0-1.0

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

        //Mast Servo Variables
        String mastDirectionToMove = "Not Initialized";
        double mastServoSpeed = 1.0;            //Set Mast Servo Speed 0.0-1.0

        if (gamepad1.dpad_up) {
            mastDirectionToMove = "FORWARD";    //Telemetry variable
            //sheldon.mastServoMove(mastDirectionToMove, mastServoSpeed);
        } else if (gamepad1.dpad_down) {
            mastDirectionToMove = "REVERSE";    //Telemetry variable
            //sheldon.mastServoMove(mastDirectionToMove, mastServoSpeed);
        } else {
            //sheldon.mastServoStop();
            mastDirectionToMove = "STOPPED";    //Telemetry variable
        }
        /**********************End of Mast Moving Forward or Reverse Routine********************/

        /**********************Fork Deploy & Retract Routine************************************/

        //Fork Servo Variables
        String positionOfForks = "Retracted";   //Default front side of robot is Forklift Side
        String statusOfForks = "Retracted";

        positionOfForks = sheldon.toggleJoystickButton(gamepad1.a, "Deployed", "Retracted");

        if (positionOfForks == "Deployed") {
            sheldon.deployLeftForkBladeServo();
            sheldon.deployRightForkBladeServo();
            statusOfForks = "Deployed";     //Sets Telemetry Variable

        } else if (positionOfForks == "Retracted") {
            sheldon.retractLeftForkBladeServo();
            sheldon.retractRightForkBladeServo();
            statusOfForks = "Retracted";    //Sets Telemetry Variable
        }

        /**********************End Fork Deploy & Retract Routine********************************/

        /******************** Toggle Particle Shooter Motors On/Off ************************/
        double leftParticleMotorPower;            //Power Value for Left DC Drive Motors
        double rightParticleMotorPower;           //Power Value for Right DC Drive Motors

        //Call method in JoystickHardware Class to toggle a button on joystick
        particleShooterToggle = sheldon.toggleJoystickButton(gamepad2.a, "On", "Off");

        //Drive the robot in the direction selected either Particle Shooter forward or Forklift Forward
        if (particleShooterToggle == "On") {
            particleShooterStatus = "On";
            leftParticleMotorPower = 1.0;
            rightParticleMotorPower = 1.0;
            sheldon.toggleParticleShooterMotors(leftParticleMotorPower, rightParticleMotorPower);

        } else if (particleShooterToggle == "Off") {
            particleShooterStatus = "Off";
            leftParticleMotorPower = 0.0;
            rightParticleMotorPower = 0.0;
            sheldon.toggleParticleShooterMotors(leftParticleMotorPower, rightParticleMotorPower);
        }
        /******************** End of Toggle Particle Shooter Motors On/Off ************************/

        /**************************** Start Raise Mast Routine ************************************/
        sheldon.driveMastLiftUp(gamepad1.right_trigger);  //Raise Mast Lift
        sheldon.driveMastLiftDown(gamepad1.left_trigger);   //Lower Mast Lift

        sheldon.driveMastLiftUp(-gamepad2.right_trigger);  //Raise Mast Lift
        sheldon.driveMastLiftDown(-gamepad2.left_trigger);   //Lower Mast Lift
        //sheldon.driveMastLift(gamepad1.left_stick_x);
        /**************************** End of Raise Mast Routine ***********************************/

        /*******************Telemetry for TeleOp Mode while Running ******************************/
        telemetry.addData("Front Side Facing", frontSideOfRobotStatus);
        telemetry.addData("Position of Forks", statusOfForks);
        telemetry.addData("Direction Mast Moving", mastDirectionToMove);
        telemetry.addData("Particle Shooter Motors", particleShooterStatus);
        updateTelemetry(telemetry);
        /***************End of Telemetry for TeleOp Mode while Running ***************************/

    }
}

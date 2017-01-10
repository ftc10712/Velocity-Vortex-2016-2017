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

    double leftDCMotorPower = 0;            //Power Value for Left DC Drive Motors
    double rightDCMotorPower = 0;           //Power Value for Right DC Drive Motors
    boolean toggleDriveDirection = false;   //Toggle which side of the robot will drive forward
    boolean driveDirection = false;         //Toggle direction of motor power when direction toggled
    String frontSideOfRobot = "Forklift";   //Default front side of robot is Forklift Side

    float rightServoRetractedPosition = .1f; //Threshold sre


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

        // Good Code

//        if(sheldon.getColorSensor1ColorDetected() == "Red"){
//            if(sheldon.leftBeaconServo.getPosition() < .1){  //Assume left beacon servo motor is retracted
//                sheldon.deployLeftBeaconServo();
//            }
//
//        } else if (sheldon.getColorSensor1ColorDetected() == "Blue" ){
//            if(sheldon.rightBeaconServo.getPosition() < .1) {  //Assume left beacon servo motor is retracted
//                sheldon.deployRightBeaconServo();
//            }
//        } else {
//                sheldon.retractLeftBeaconServo();
//                sheldon.retractRightBeaconServo();
//            }
        //End Sample Code

        //telemetry.addData("Toch Sensor Button is currently ", sheldon.isTouchSensorPressed());
        //telemetry.addData("Color Sensor 1 Detected ", sheldon.getColorSensor1ColorDetected());

        telemetry.addData("Left Joystick Button", toggleDriveDirection);
        telemetry.addData("Motor Direction", driveDirection);
        telemetry.addData("Front Side Facing", frontSideOfRobot);
        updateTelemetry(telemetry);
    }
}

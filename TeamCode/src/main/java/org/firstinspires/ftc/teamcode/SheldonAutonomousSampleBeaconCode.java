package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Austin Ford & Tristan Sorenson on 12/19/2016.
 */

@Autonomous(name = "Sheldon Sample Beacon Code", group = "Autonomous")
@Disabled

public class SheldonAutonomousSampleBeaconCode extends OpMode {

    /*------------Member Variables--------------------*/

    // New instance of our robot class we will call "Sheldon"
    Robot sheldon = new Robot();

    String TEAM_COLOR = "Red";

    String beaconActivated = "Not Activated...";

    ElapsedTime autonomousRunTime;

    enum State {stepA, stepB, stepC, stepD, stepE, stepF, stepG, stepH, stepI, stepJ, stepK, stepL, stepM, done}

    State state;

    @Override
    public void init() {

        sheldon.initializeRobot(hardwareMap);
        autonomousRunTime = new ElapsedTime();
        state = State.stepA;
        sheldon.resetEncoders();
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        sheldon.gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (sheldon.gyro.isCalibrating()) {

        }
        sheldon.runUsingEncoders();

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();
    }

    public void init_loop() {
        //Press Game Pad 1 - x button for blue team and Game Pad 1 - b button for Red Team

        if (gamepad1.x == true) {
            TEAM_COLOR = "Blue";
        } else if (gamepad1.b == true) {
            TEAM_COLOR = "Red";
        }

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
        updateTelemetry(telemetry);
    }


    @Override
    public void loop() {
        double currentTime = autonomousRunTime.time();
        switch (state) {

            case stepA:
                //Drive forward 24"

                state = State.stepB;
                break;

            case stepB:
                //Turn Right 45 Degrees if Blue and Turn Left 45 Degrees if Red
                state = State.stepC;
                break;

            case stepC:
                //Drive forward 51"
                state = State.stepD;
                break;

            case stepD:
                //Start Turning Right if Blue & Look for White Line - Start Turning Left if Red and
                //look for white line
                state = State.stepE;
                break;

            case stepE:
                //Follow white line and keep looking for beacon color
                if (sheldon.getColorSensor1ColorDetected() == TEAM_COLOR && sheldon.getColorSensor2ColorDetected() != TEAM_COLOR) {
                    if (sheldon.leftBeaconServo.getPosition() <= sheldon.leftBeaconServoCalibratedMin) {  //Assume left beacon servo motor is retracted
                        while (sheldon.getColorSensor1ColorDetected() == TEAM_COLOR && sheldon.getColorSensor2ColorDetected() != TEAM_COLOR) {
                            sheldon.deployLeftBeaconServo();
                            //Add code to drive motors here
                        }
                        //Add code to stop the motors because the beacon should have been successfully activated
                        sheldon.retractLeftBeaconServo();
                    }

                } else if (sheldon.getColorSensor1ColorDetected() != TEAM_COLOR && sheldon.getColorSensor2ColorDetected() == TEAM_COLOR) {
                    if (sheldon.rightBeaconServo.getPosition() <= .16) {  //Assume left beacon servo motor is retracted
                        //sheldon.deployRightBeaconServo();
                        while (sheldon.getColorSensor1ColorDetected() != TEAM_COLOR && sheldon.getColorSensor2ColorDetected() == TEAM_COLOR) {
                            sheldon.deployRightBeaconServo();
                            //Add code to drive motors here
                        }
                        //Add code to stop the motors because the beacon should have been successfully activated
                        sheldon.retractRightBeaconServo();
                    }

                } else if (sheldon.getColorSensor1ColorDetected() == TEAM_COLOR && sheldon.getColorSensor2ColorDetected() == TEAM_COLOR) {
                    //Add code to slowly drive toward beacons

                    beaconActivated = "Successfully Activated!";

                } else {
                    //Add code to slowly drive toward beacons
                    sheldon.retractLeftBeaconServo();
                    sheldon.retractRightBeaconServo();
                }
                state = State.stepF;
                break;

            case stepF:
                //Once beacon activated backup 5"
                state = State.stepG;
                break;

            case stepG:
                //Turn Left 90 degrees if blue and right 90 degrees if red
                state = State.stepH;
                break;

            case stepH:
                //Drive forward 48"
                state = State.stepI;
                break;

            case stepI:
                //Start Turning Right if Blue & Look for White Line - Start Turning Left if Red and
                //look for white line
                state = State.stepJ;
                break;

            case stepJ:
                //Follow white line and trp beacon
                state = State.stepK;
                break;

            case stepK:
                //Backup 6"
                state = State.stepL;
                break;

            case stepL:
                //Start Turning Left if Blue & Look for White Line - Start Turning Right if Red 90
                //degrees
                state = State.stepM;
                break;

            case stepM:
                //Drive in reverse 78"
                state = State.done;
                break;

            case done:
                break;
        }// End of Switch Statement


        //End Sample Code

        telemetry.addData("Toch Sensor Button is currently ", sheldon.isTouchSensorPressed());
        telemetry.addData("Color Sensor 1 Detected ", sheldon.getColorSensor1ColorDetected());
        telemetry.addData("Color Sensor 2 Detected ", sheldon.getColorSensor2ColorDetected());
        telemetry.addData("Optical Distance 1 Raw Light", "%.2f", sheldon.getOpticalDistanceSensor1RawLightDetected());
        telemetry.addData("Optical Distance 1 Normal Light", "%.2f", sheldon.getOpticalDistanceSensor1NormalLightDetected());
        telemetry.addData("Optical Distance 2 Raw Light", "%.2f", sheldon.getOpticalDistanceSensor2RawLightDetected());
        telemetry.addData("Optical Distance 2 Normal Light", "%.2f", sheldon.getOpticalDistanceSensor2NormalLightDetected());
        telemetry.addData("Right Beacon Servo Position", "%.2f", sheldon.rightBeaconServo.getPosition());
        telemetry.addData("Color Beacon: ", beaconActivated);
        updateTelemetry(telemetry);
    }
}

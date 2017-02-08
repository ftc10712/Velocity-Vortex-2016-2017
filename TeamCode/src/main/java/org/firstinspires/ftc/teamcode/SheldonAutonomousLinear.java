package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Darrell on 2/7/2017.
 */

@Autonomous(name = "Sheldon Linear", group = "Autonomous")
//@Disabled
public class SheldonAutonomousLinear extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //Start of our robot initialization//
        /*------------Member Variables--------------------*/

        // New instance of our robot class we will call "Sheldon"
        Robot sheldon = new Robot();

        //Setup toggle variables to choose whether to run the "Red" Team Autonomous or "Blue"
        //Team Autonomous mode...
        boolean teamColorToggle = false;
        boolean teamColorToggleCurrentState = false;
        String TEAM_COLOR = "Red";

        String beaconActivated = "Not Activated...";
        ElapsedTime autonomousRunTime = new ElapsedTime();

        sheldon.initializeRobot(hardwareMap);

        if (gamepad1.x) {
            TEAM_COLOR = "Blue";
        } else if (gamepad1.b) {
            TEAM_COLOR = "Red";
        }

        telemetry.addData("Current Team Color Selected", TEAM_COLOR);
        telemetry.addData("Left Beacon Servo", sheldon.getLeftBeaconServoStatus());
        telemetry.addData("Right Beacon Servo", sheldon.getRightBeaconServoStatus());
        telemetry.addData("Optical Distance Sensor 1", sheldon.getOpticalDistanceSensor1Status());
        telemetry.addData("Optical Distance Sensor 2", sheldon.getOpticalDistanceSensor2Status());
        telemetry.addData("Touch Sensor", sheldon.getTouchSensorStatus());
        telemetry.addData("Color Sensor 1", sheldon.getColorSensor1Status());
        telemetry.addData("Color Sensor 2", sheldon.getColorSensor2Status());
        updateTelemetry(telemetry);

        waitForStart();

        //Start of our Autonomous Steps
        while (opModeIsActive()) {

            if (sheldon.getColorSensor1ColorDetected() == TEAM_COLOR && sheldon.getColorSensor2ColorDetected() != TEAM_COLOR) {
                if (sheldon.leftBeaconServo.getPosition() < .1) {  //Assume left beacon servo motor is retracted
                    while (sheldon.getColorSensor1ColorDetected() == TEAM_COLOR && sheldon.getColorSensor2ColorDetected() != TEAM_COLOR) {
                        sheldon.deployLeftBeaconServo();
                        //Add code to drive motors here
                    }
                    //Add code to stop the motors because the beacon should have been successfully activated
                    sheldon.retractLeftBeaconServo();
                }

            } else if (sheldon.getColorSensor1ColorDetected() != TEAM_COLOR && sheldon.getColorSensor2ColorDetected() == TEAM_COLOR) {
                if (sheldon.rightBeaconServo.getPosition() < .1) {  //Assume left beacon servo motor is retracted
                    sheldon.deployRightBeaconServo();
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

            telemetry.addData("Toch Sensor Button is currently ", sheldon.isTouchSensorPressed());
            telemetry.addData("Color Sensor 1 Detected ", sheldon.getColorSensor1ColorDetected());
            telemetry.addData("Color Sensor 2 Detected ", sheldon.getColorSensor2ColorDetected());
            telemetry.addData("Optical Distance 1 Raw Light", "%.2f", sheldon.getOpticalDistanceSensor1RawLightDetected());
            telemetry.addData("Optical Distance 1 Normal Light", "%.2f", sheldon.getOpticalDistanceSensor1NormalLightDetected());
            telemetry.addData("Color Beacon: ", beaconActivated);
            updateTelemetry(telemetry);

            idle(); //Always put at end of loop
        }

        //Termination Code goes here//

    }
}

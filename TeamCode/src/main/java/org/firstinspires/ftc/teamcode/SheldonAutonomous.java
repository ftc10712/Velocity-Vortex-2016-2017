package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Austin Ford & Tristan Sorenson on 12/19/2016.
 */

@Autonomous(name = "Autonomous: Sheldon", group = "Autonomous")

public class SheldonAutonomous extends OpMode {

    Robot sheldon = new Robot();
    final String TEAM_COLOR = "Red";


    @Override
    public void init() {
        sheldon.initializeRobot(hardwareMap);

        telemetry.addData("Left Beacon Servo", sheldon.getLeftBeaconServoStatus());
        telemetry.addData("Right Beacon Servo", sheldon.getRightBeaconServoStatus());
        telemetry.addData("Optical Distance Sensor 1", sheldon.getOpticalDistanceSensor1Status());
        telemetry.addData("Optical Distance Sensor 2", sheldon.getOpticalDistanceSensor2Status());
        telemetry.addData("Touch Sensor", sheldon.getTouchSensorStatus());
        telemetry.addData("Color Sensor 1", sheldon.getColorSensor1Status());
        telemetry.addData("Color Sensor 2", sheldon.getColorSensor2Status());
        updateTelemetry(telemetry);
    }

    @Override
    public void loop() {
        // Sample Code Not Production
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
            //boolean beaconActivated;
            telemetry.addData("Beacon Status: ", "Successfully Activated!");

        } else {
            //Add code to slowly drive toward beacons
            sheldon.retractLeftBeaconServo();
            sheldon.retractRightBeaconServo();
        }

        //End Sample Code

        telemetry.addData("Toch Sensor Button is currently ", sheldon.isTouchSensorPressed());
        telemetry.addData("Color Sensor 1 Detected ", sheldon.getColorSensor1ColorDetected());
        telemetry.addData("Color Sensor 2 Detected ", sheldon.getColorSensor2ColorDetected());
        telemetry.addData("Optical Distance 1 Raw Light", sheldon.getOpticalDistanceSensor1RawLightDetected());
        telemetry.addData("Optical Distance 1 Normal Light", sheldon.getOpticalDistanceSensor1NormalLightDettected());
        updateTelemetry(telemetry);
    }
}

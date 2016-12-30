package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Austin Ford & Tristan Sorenson on 12/19/2016.
 */

@TeleOp(name = "TeleOp: Sheldon", group = "TeleOP")

public class SheldonTeleOp extends OpMode {

    Robot sheldon = new Robot();
    final String TEAM_COLOR = "RED";
    float rightServoRetractedPosition = .1f; //Threshold sre


    @Override
    public void init() {
        sheldon.initializeRobot(hardwareMap);

        telemetry.addData("Left Beacon Servo ", sheldon.getLeftBeaconServoStatus());
        telemetry.addData("Right Beacon Servo ", sheldon.getRightBeaconServoStatus());
        telemetry.addData("Touch Sensor ", sheldon.getTouchSensorStatus());
        telemetry.addData("Color Sensor 1 ", sheldon.getColorSensor1Status());
        updateTelemetry(telemetry);
    }

    @Override
    public void loop() {


        // Sample Code Not Production
        if(sheldon.getColorSensor1ColorDetected() == "Red"){
            if(sheldon.leftBeaconServo.getPosition() < .1){  //Assume left beacon servo motor is retracted
                sheldon.deployLeftBeaconServo();
            }

        } else if (sheldon.getColorSensor1ColorDetected() == "Blue" ){
            if(sheldon.rightBeaconServo.getPosition() < .1) {  //Assume left beacon servo motor is retracted
                sheldon.deployRightBeaconServo();
            }
        } else {
                sheldon.retractLeftBeaconServo();
                sheldon.retractRightBeaconServo();
            }
        //End Sample Code
        telemetry.addData("Toch Sensor Button is currently ", sheldon.isTouchSensorPressed());
        telemetry.addData("Color Sensor 1 Detected ", sheldon.getColorSensor1ColorDetected());
        updateTelemetry(telemetry);
    }
}

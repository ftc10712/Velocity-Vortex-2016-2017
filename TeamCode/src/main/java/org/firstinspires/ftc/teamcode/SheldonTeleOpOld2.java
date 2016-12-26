package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Sheldon: Teleop", group="Sheldon")
@Disabled
public class SheldonTeleOpOld2 extends OpMode {

    //Member Variables

    //Create a new instance of our extended Robot Class and name it "Sheldon"
    Robot sheldon = new Robot();


    //Define variables we will use
    /*final double GRIPPER_SPEED = 1.0;
    boolean bPrevState = false;
    boolean bCurrState = false;
    boolean positionForks = false;
    String directionToMove = "Not Initialized";*/

    //Declare the variables for the DC Drive Motors on Sheldon
    @Override
    public void init() {
        //sheldon.initializeRobot();
    }

    @Override
    public void loop() {

        /*if (gamepad1.right_bumper) {
            directionToMove = "OPEN";
            sheldon.forkGripperMove(directionToMove, GRIPPER_SPEED);
        } else if (gamepad1.left_bumper) {
            String directionToMove = "CLOSE";
            sheldon.forkGripperMove(directionToMove, GRIPPER_SPEED);
        } else {
            sheldon.forkGripperStop();
        }*/

        telemetry.addData("Clear", sheldon.colorSensor1.alpha());
        telemetry.addData("Red  ", sheldon.colorSensor1.red());
        telemetry.addData("Green", sheldon.colorSensor1.green());
        telemetry.addData("Blue ", sheldon.colorSensor1.blue());
        //telemetry.addData("Fork Gripper  ", directionToMove+"ing");
        updateTelemetry(telemetry);
    }

}

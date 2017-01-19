package org.firstinspires.ftc.teamcode;

/**
 * Created by Darrell on 1/19/2017.
 */

public class JoystickHardware extends SensorHardware {

    boolean toggleButtonStatus = false;
    boolean toggleButton = false;
    String buttonStatus = "";

    JoystickHardware() {

    }

    /**************************
     * Toggle Joystick Button Method
     ******************************/

    public String toggleJoystickButton(boolean joystickButton, String onStatus, String offStatus) {
        if (joystickButton && !toggleButton) {

            toggleButtonStatus = !toggleButtonStatus;

            if (toggleButtonStatus) {

                buttonStatus = onStatus;  //Telemetry data variable

            } else {

                buttonStatus = offStatus;      //Telemetry data variable
            }

            toggleButton = true;

        } else if (!joystickButton) {
            toggleButton = false;

        }
        return buttonStatus;

    }
    /********************* End Toggle Button on Joystick Method*******************************/
}

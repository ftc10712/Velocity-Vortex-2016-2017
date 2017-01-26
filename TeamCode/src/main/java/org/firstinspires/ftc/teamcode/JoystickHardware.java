package org.firstinspires.ftc.teamcode;

/**
 * Created by Austin Ford on January 19, 2017
 */

class JoystickHardware extends SensorHardware {

    private boolean toggleButtonStatus = false;
    private boolean toggleButton = false;
    private String buttonStatus = "";

    JoystickHardware() {

    }

    /**************************
     * Toggle Joystick Button Method
     ******************************/

    String toggleJoystickButton(boolean joystickButton, String onStatus, String offStatus) {
        if (joystickButton && !toggleButton) {
            toggleButton = true;
            toggleButtonStatus = !toggleButtonStatus;

            if (toggleButtonStatus) {

                buttonStatus = onStatus;  //Telemetry data variable

            } else {

                buttonStatus = offStatus;      //Telemetry data variable
            }


        } else if (!joystickButton) {
            toggleButton = false;

        }
        return buttonStatus;

    }

}

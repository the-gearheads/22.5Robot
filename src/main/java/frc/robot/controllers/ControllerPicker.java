package frc.robot.controllers;

import edu.wpi.first.wpilibj.DriverStation;

public class ControllerPicker {
    private static String[] lastControllerNames = new String[6];

    /** Returns true if the connected controllers have changed since last called. */
    public static boolean didControllersChange() {
        boolean hasChanged = false;

        for (int i=0; i<DriverStation.kJoystickPorts; i++) {
            String name = DriverStation.getJoystickName(i);
            if(!name.equals(lastControllerNames[i])) {
                hasChanged = true;
                lastControllerNames[i] = name;
            }
        }

        return hasChanged;
    }

    
}

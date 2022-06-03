package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;

public class SingleXboxController implements ControllerInterface {

    XboxController controller;

    public SingleXboxController(int port) {
        controller = new XboxController(port);
    }

    public double getMoveAxis() {
        return controller.getLeftY();
    }

    public double getRotateAxis() {
        return controller.getRightX();
    }
    
}

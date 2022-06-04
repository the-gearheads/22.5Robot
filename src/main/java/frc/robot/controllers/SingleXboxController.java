package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class SingleXboxController implements ControllerInterface {

    XboxController controller;

    public SingleXboxController(int port) {
        controller = new XboxController(port);
    }

    public double getMoveAxis() {
        return MathUtil.applyDeadband(controller.getLeftY(), Constants.Controllers.DRIVE_DEADBAND);
    }

    public double getRotateAxis() {
        return MathUtil.applyDeadband(controller.getRightX(), Constants.Controllers.ROTATE_DEADBAND);
    }

}

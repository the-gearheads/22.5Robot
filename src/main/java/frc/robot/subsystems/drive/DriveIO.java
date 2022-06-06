package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface DriveIO {
    public static class DriveIOInputs implements LoggableInputs {

        public void toLog(LogTable table) {

        }

        public void fromLog(LogTable table) {
            
        }
    }

    public default void updateInputs(DriveIOInputs inputs) {

    }

    public default void setVoltages(double leftVolage, double rightVoltage) {

    }

    public default void setBrakeMode(boolean isBraked) {

    }
}

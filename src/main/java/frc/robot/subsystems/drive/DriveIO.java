package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;

public interface DriveIO {
    public static class DriveIOInputs implements LoggableInputs {

        public double rightPos;
        public double rightVelocity;

        public double leftPos;
        public double leftVelocity;

        public double appliedLeftVolts;
        public double appliedRightVolts;
        
        public Rotation2d heading;

        public void toLog(LogTable table) {
            table.put("RightPos", rightPos);
            table.put("RightVelocity", rightVelocity);

            table.put("LeftPos", leftPos);
            table.put("LeftVelocity", leftVelocity);

            table.put("LeftVolts", appliedLeftVolts);
            table.put("RightVolts", appliedRightVolts);

            table.put("HeadingDegrees", heading.getDegrees());
        }

        public void fromLog(LogTable table) {
            rightPos = table.getDouble("RightPos", rightPos);
            rightVelocity = table.getDouble("RightVelocity", rightVelocity);

            leftPos = table.getDouble("LeftPos", leftPos);
            leftVelocity = table.getDouble("LeftVelocity", leftVelocity);

            appliedLeftVolts = table.getDouble("LeftVolts", appliedLeftVolts);
            appliedRightVolts = table.getDouble("RightVolts", appliedRightVolts);

            heading = Rotation2d.fromDegrees(table.getDouble("HeadingDegrees", 0));
        }
    }

    /** Update the provided {@link DriveIOInputs} class with inputs */
    public default void updateInputs(DriveIOInputs inputs) {

    }

    /** Sets voltages of motors to specified values */
    public default void setVoltages(double leftVolage, double rightVoltage) {

    }

    /** Sets whether the robot brakes or coasts (true means brake) */
    public default void setBrakeMode(boolean doBraking) {

    }

    /** Zeros all motor encoders and resets gyro */
    public default void zeroEncoders() {

    }
}

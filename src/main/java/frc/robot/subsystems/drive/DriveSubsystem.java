package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveIO.DriveIOInputs;

public class DriveSubsystem extends SubsystemBase {

    Field2d field;
    
    DriveIO drive;
    DriveIOInputs inputs = new DriveIOInputs();

    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TRACK_WIDTH);
    private DifferentialDriveOdometry odometry; 

    private SimpleMotorFeedforward rFeedforward = new SimpleMotorFeedforward(Constants.Drivetrain.RIGHT_kS, Constants.Drivetrain.RIGHT_kV);
    private SimpleMotorFeedforward lFeedforward = new SimpleMotorFeedforward(Constants.Drivetrain.LEFT_kS, Constants.Drivetrain.LEFT_kV);


    public DriveSubsystem(DriveIO io) {
        drive = io;
        // Call updateInputs once before we zero encoders so the gyro heading is set
        drive.updateInputs(inputs); 

        zeroEncoders();

        /* Push field to dashboard */
        SmartDashboard.putData("Field", field);
    }

    public void zeroEncoders() {
        drive.zeroEncoders();

        /* Reset odometry */
        odometry = new DifferentialDriveOdometry(inputs.heading, new Pose2d(0, 0, new Rotation2d(0)));
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        double leftSpeed = lFeedforward.calculate(speeds.leftMetersPerSecond);
        double rightSpeed = rFeedforward.calculate(speeds.rightMetersPerSecond);

        drive.setVoltages(leftSpeed, rightSpeed);
    }

    public void drive(double xSpeed, double rot) {
        var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot));
        setSpeeds(wheelSpeeds);
    }

    public void setBrakeMode(boolean doBraking) {
        drive.setBrakeMode(doBraking);
    }

    @Override
    public void periodic() {
        drive.updateInputs(inputs);
        Logger.getInstance().processInputs("Drivetrain", inputs);

        /* Do stuff here */ 
        odometry.update(inputs.heading, inputs.leftPos, inputs.rightPos);
        field.setRobotPose(odometry.getPoseMeters());
    }
}

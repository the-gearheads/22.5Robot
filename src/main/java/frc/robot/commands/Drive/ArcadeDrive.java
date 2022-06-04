package frc.robot.commands.Drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDrive extends CommandBase {
    private DriveSubsystem drive;
    public double moveSpeed = Constants.Controllers.DEFAULT_DRIVE_SPEED;
    public double rotateSpeed = Constants.Controllers.DEFAULT_ROT_SPEED;
    
    public ArcadeDrive(DriveSubsystem driveSubsystem) {
        drive = driveSubsystem;
        addRequirements(drive);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drive.setNeutralMode(NeutralMode.Brake);
        drive.zeroEncoders();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        moveSpeed = NetworkTableInstance.getDefault().getTable("/SmartDashboard/ArcadeDrive").getEntry("moveSpeed").getNumber(Constants.Controllers.DEFAULT_DRIVE_SPEED).doubleValue();
        rotateSpeed = NetworkTableInstance.getDefault().getTable("/SmartDashboard/ArcadeDrive").getEntry("rotateSpeed").getNumber(Constants.Controllers.DEFAULT_ROT_SPEED).doubleValue();

        SmartDashboard.putNumber("ArcadeDrive/moveSpeed", moveSpeed);
        SmartDashboard.putNumber("ArcadeDrive/rotateSpeed", rotateSpeed);



        drive.drive(-Controllers.activeController.getMoveAxis() * moveSpeed, -Controllers.activeController.getRotateAxis() * rotateSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

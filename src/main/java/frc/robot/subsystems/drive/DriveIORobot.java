package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

public class DriveIORobot implements DriveIO {
    
        WPI_TalonFX rFrontMotor = new WPI_TalonFX(Constants.Drivetrain.RIGHT_FRONT_MOTOR);
        WPI_TalonFX rRearMotor = new WPI_TalonFX(Constants.Drivetrain.RIGHT_REAR_MOTOR);
        WPI_TalonFX lFrontMotor = new WPI_TalonFX(Constants.Drivetrain.LEFT_FRONT_MOTOR);
        WPI_TalonFX lRearMotor = new WPI_TalonFX(Constants.Drivetrain.LEFT_REAR_MOTOR);

        AHRS gyro = new AHRS(SPI.Port.kMXP);

        public DriveIORobot() {
            /* Setup motors */
            rFrontMotor.configFactoryDefault();
            rRearMotor.configFactoryDefault();
            lFrontMotor.configFactoryDefault();
            lRearMotor.configFactoryDefault();

            rFrontMotor.setInverted(true);
            rRearMotor.setInverted(InvertType.FollowMaster);

            rRearMotor.follow(rFrontMotor);
            lRearMotor.follow(lFrontMotor);

            gyro.reset();
        }

        /** Converts native encoder units to meters moved, accounting for gearing and wheels */
        private double nativeToMeters(double nativeUnits) {
            double rotations = nativeUnits / Constants.Drivetrain.TALON_UNITS_PER_ROTATION;         // convert native units to rotations
            double wheelRotations = rotations / Constants.Drivetrain.SHAFT_TO_WHEEL_GEAR_RATIO;     // convert rotations of motor shaft to rotations of wheel
            double linearDisplacement = wheelRotations * Constants.Drivetrain.WHEEL_CIRCUMFERENCE;  // convert wheel rotations to linear displacement
            return linearDisplacement;
        }

        /* Converts native velocity units to m/s */
        private double nativeVelocityToMeters(double velocity) {
            return nativeToMeters(velocity) * 10; // converts m/100ms to m/s
        }

        public void updateInputs(DriveIOInputs inputs) {
            inputs.rightPos = nativeToMeters(rFrontMotor.getSelectedSensorPosition());
            inputs.leftPos = nativeToMeters(lFrontMotor.getSelectedSensorPosition());

            inputs.rightVelocity = nativeVelocityToMeters(rFrontMotor.getSelectedSensorVelocity());
            inputs.leftVelocity = nativeVelocityToMeters(lFrontMotor.getSelectedSensorVelocity());

            inputs.heading = gyro.getRotation2d();
            
            inputs.appliedRightVolts = rFrontMotor.getMotorOutputVoltage();
            inputs.appliedLeftVolts = lFrontMotor.getMotorOutputVoltage();
        }

        public void setVoltages(double leftVoltage, double rightVoltage) {
            lFrontMotor.setVoltage(MathUtil.clamp(leftVoltage, -12, 12));
            rFrontMotor.setVoltage(MathUtil.clamp(rightVoltage, -12, 12));
        }

        public void zeroEncoders() {
            rFrontMotor.setSelectedSensorPosition(0);
            rRearMotor.setSelectedSensorPosition(0);
            lFrontMotor.setSelectedSensorPosition(0);
            lRearMotor.setSelectedSensorPosition(0);
    
            gyro.reset();
        }

        public void setBrakeMode(boolean doBraking) {
            NeutralMode mode = doBraking ? NeutralMode.Brake : NeutralMode.Coast;

            rFrontMotor.setNeutralMode(mode);
            rRearMotor.setNeutralMode(mode);
            lFrontMotor.setNeutralMode(mode);
            lRearMotor.setNeutralMode(mode);
        }
    
    
}

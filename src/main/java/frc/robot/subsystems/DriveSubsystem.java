package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;

import javax.swing.GrayFilter;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.commands.Drive.ArcadeDrive;

public class DriveSubsystem extends SubsystemBase {

    private WPI_TalonFX lFrontMotor = new WPI_TalonFX(Constants.Drivetrain.LEFT_FRONT_MOTOR);
    private WPI_TalonFX lRearMotor = new WPI_TalonFX(Constants.Drivetrain.LEFT_REAR_MOTOR);
    private TalonFXSimCollection lSim = lFrontMotor.getSimCollection();

    private WPI_TalonFX rFrontMotor = new WPI_TalonFX(Constants.Drivetrain.RIGHT_FRONT_MOTOR);
    private WPI_TalonFX rRearMotor = new WPI_TalonFX(Constants.Drivetrain.RIGHT_REAR_MOTOR);
    private TalonFXSimCollection rSim = rFrontMotor.getSimCollection();

    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    private SimDouble simAngle;

    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TRACK_WIDTH);

    private DifferentialDrivetrainSim sim;

    private DifferentialDriveOdometry odometry; 

    private SimpleMotorFeedforward rFeedforward = new SimpleMotorFeedforward(Constants.Drivetrain.RIGHT_kS, Constants.Drivetrain.RIGHT_kV);
    private SimpleMotorFeedforward lFeedforward = new SimpleMotorFeedforward(Constants.Drivetrain.LEFT_kS, Constants.Drivetrain.LEFT_kV);

    private Field2d field = new Field2d();

    public DriveSubsystem() {

        /* Reset motors */
        lFrontMotor.configFactoryDefault();
        lRearMotor.configFactoryDefault();
        rFrontMotor.configFactoryDefault();
        lRearMotor.configFactoryDefault();


        /* Right motors are inverted because they're facing the other way, fix that */
        lFrontMotor.setInverted(false);
        lRearMotor.setInverted(InvertType.FollowMaster);
        rFrontMotor.setInverted(true);
        rRearMotor.setInverted(InvertType.FollowMaster);

        lRearMotor.follow(lFrontMotor);
        rRearMotor.follow(rFrontMotor);

        
        /* Set up gyro simulation */
        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));

        zeroEncoders();
        setDefaultCommand(new ArcadeDrive(this));
        
        SmartDashboard.putData("Field", field); // Put game field on dahsboard
    }

    /**
     * Zeros everything
     */
    public void zeroEncoders() {
        rFrontMotor.setSelectedSensorPosition(0);
        rRearMotor.setSelectedSensorPosition(0);
        lFrontMotor.setSelectedSensorPosition(0);
        lRearMotor.setSelectedSensorPosition(0);

        gyro.reset();
        /* Reset our position */
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), new Pose2d(0, 0, new Rotation2d(0)));

        /* Reset simulation */
        sim = new DifferentialDrivetrainSim(DCMotor.getFalcon500(2), Constants.Drivetrain.SHAFT_TO_WHEEL_GEAR_RATIO, 
            Constants.Drivetrain.Sim.JKG_M2, Constants.Drivetrain.Sim.ROBOT_MASS, Constants.Drivetrain.WHEEL_CIRCUMFERENCE, Constants.Drivetrain.TRACK_WIDTH, null);
    }

    /**
     * Sets neutral mode for all motors (e.g. brake or coast)
     * @param mode 
     */
    public void setNeutralMode(NeutralMode mode) {
        rFrontMotor.setNeutralMode(mode);
        rRearMotor.setNeutralMode(mode);
        lFrontMotor.setNeutralMode(mode);
        lRearMotor.setNeutralMode(mode);
    }


    /* Encoder related functions */
    public double getRightPos() {
        double encoderVal = rFrontMotor.getSelectedSensorPosition();
        double rotations = encoderVal / Constants.Drivetrain.TALON_UNITS_PER_ROTATION;                           // convert native units to rotations
        double wheelRotations = rotations / Constants.Drivetrain.SHAFT_TO_WHEEL_GEAR_RATIO;                      // convert rotations of motor shaft to rotations of wheel
        double linearDisplacement = wheelRotations * Constants.Drivetrain.WHEEL_CIRCUMFERENCE;                   // convert wheel rotations to linear displacement
        return linearDisplacement;
    }

    public double getLeftPos(){
        double encoderVal = lFrontMotor.getSelectedSensorPosition();
        double rotations = encoderVal / Constants.Drivetrain.TALON_UNITS_PER_ROTATION;                           // convert native units to rotations
        double wheelRotations = rotations / Constants.Drivetrain.SHAFT_TO_WHEEL_GEAR_RATIO;                      // convert rotations of motor shaft to rotations of wheel
        double linearDisplacement = wheelRotations * Constants.Drivetrain.WHEEL_CIRCUMFERENCE;                   // convert wheel rotations to linear displacement
        return linearDisplacement;
    }

    public double getRightVel(){
        double encoderVal = rFrontMotor.getSelectedSensorPosition();
        double rotations = encoderVal / Constants.Drivetrain.TALON_UNITS_PER_ROTATION;                           // convert native units to rotations
        double wheelRotations = rotations / Constants.Drivetrain.SHAFT_TO_WHEEL_GEAR_RATIO;                      // convert rotations of motor shaft to rotations of wheel
        double linearDisplacement = wheelRotations * Constants.Drivetrain.WHEEL_CIRCUMFERENCE;                   // convert wheel rotations to linear displacement
        return linearDisplacement * 10;                                                                          // convert m/100ms to m/s
    }

    public double getLeftVel(){
        double encoderVal = lFrontMotor.getSelectedSensorPosition();
        double rotations = encoderVal / Constants.Drivetrain.TALON_UNITS_PER_ROTATION;                           // convert native units to rotations
        double wheelRotations = rotations / Constants.Drivetrain.SHAFT_TO_WHEEL_GEAR_RATIO;                      // convert rotations of motor shaft to rotations of wheel
        double linearDisplacement = wheelRotations * Constants.Drivetrain.WHEEL_CIRCUMFERENCE;                   // convert wheel rotations to linear displacement
        return linearDisplacement * 10;                                                                          // convert m/100ms to m/s
    }

    /** Sets motor speeds */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        double leftSpeed = lFeedforward.calculate(speeds.leftMetersPerSecond);
        double rightSpeed = rFeedforward.calculate(speeds.rightMetersPerSecond);

        lFrontMotor.setVoltage(MathUtil.clamp(leftSpeed, -12, 12));
        rFrontMotor.setVoltage(MathUtil.clamp(rightSpeed, -12, 12));
    }

    public void drive(double xSpeed, double rot) {
        var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        setSpeeds(wheelSpeeds);
    }

    /**
     * Runs every 20ms by default
     */
    @Override
    public void periodic() {

        SmartDashboard.putNumber("Drivetrain/REncoderPos", rFrontMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Drivetrain/REncoderVel", rFrontMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Drivetrain/RPos", getRightPos());

        SmartDashboard.putNumber("Drivetrain/LEncoderPos", lFrontMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Drivetrain/LEncoderVel", lFrontMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Drivetrain/LPos", getLeftPos());

        SmartDashboard.putNumber("Drivetrain/Heading", gyro.getRotation2d().getDegrees());

        odometry.update(gyro.getRotation2d(), getLeftPos(), getRightPos());
        field.setRobotPose(odometry.getPoseMeters());
    }

    private int distanceToNativeUnits(double position) {
        double wheelRotations = position / Constants.Drivetrain.WHEEL_CIRCUMFERENCE;
        double motorRotations = wheelRotations * Constants.Drivetrain.SHAFT_TO_WHEEL_GEAR_RATIO;
        double nativeUnits = motorRotations * Constants.Drivetrain.TALON_UNITS_PER_ROTATION;
        return (int)(nativeUnits);
    }

    private int velocityToNativeUnits(double velocity) {
        // i think this should work
        return (int)(distanceToNativeUnits(velocity) / 10);
    }

    /**
     * Simulation specific periodic code (e.g. physics simulations)
     */
    @Override
    public void simulationPeriodic() {

        /* Pass the robot battery voltage to the simulated Talon FXs */
        lSim.setBusVoltage(RobotController.getBatteryVoltage());
        rSim.setBusVoltage(RobotController.getBatteryVoltage());

        // Set outputs, motor inputs, inverting the right (setInverted doesn't work here).
        SmartDashboard.putNumber("Drivetrain/Sim/LMotorOutputLeadVoltage", lSim.getMotorOutputLeadVoltage());
        SmartDashboard.putNumber("Drivetrain/Sim/RMotorOutputLeadVoltage", rSim.getMotorOutputLeadVoltage());

        sim.setInputs(lSim.getMotorOutputLeadVoltage(),
            -rSim.getMotorOutputLeadVoltage());

        /* Advance the model by 20ms */
        sim.update(0.02);

        /* Set odometry for position and velocity based on simulation. */
        lSim.setIntegratedSensorRawPosition(
            distanceToNativeUnits(
                sim.getLeftPositionMeters()
        ));
        lSim.setIntegratedSensorVelocity(
            velocityToNativeUnits(
                sim.getLeftVelocityMetersPerSecond()
        ));
        rSim.setIntegratedSensorRawPosition(
            distanceToNativeUnits(
                -sim.getRightPositionMeters()
        ));
        rSim.setIntegratedSensorVelocity(
            velocityToNativeUnits(
                -sim.getRightVelocityMetersPerSecond()
        ));

        simAngle.set(-sim.getHeading().getDegrees());

    }
}

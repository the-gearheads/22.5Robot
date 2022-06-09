package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.Constants;

public class DriveIOSim extends DriveIORobot {
    private TalonFXSimCollection lSim = lFrontMotor.getSimCollection();
    private TalonFXSimCollection rSim = rFrontMotor.getSimCollection();
    private SimDouble simAngle;

    private DifferentialDrivetrainSim sim;
    
    private void initSim() {
        var drivetrain = LinearSystemId.identifyDrivetrainSystem(Constants.Drivetrain.Sim.LINEAR_KV,
            Constants.Drivetrain.Sim.LINEAR_KA, Constants.Drivetrain.Sim.ANGULAR_KV,
            Constants.Drivetrain.Sim.ANGULAR_KA
        );

        sim = new DifferentialDrivetrainSim(drivetrain, DCMotor.getFalcon500(2),
            Constants.Drivetrain.SHAFT_TO_WHEEL_GEAR_RATIO,
            Constants.Drivetrain.TRACK_WIDTH, Constants.Drivetrain.WHEEL_RADIUS, null
        );
    }

    public DriveIOSim() {
        super();

        /* Set up gyro simulation */
        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));

        /* Init drivetrain simulation */
        initSim();
    }

    @Override
    public void zeroEncoders() {

        /* Also reset the simulation so the physics don't carry over in a reset.
         * Just for convienence, probably makes it less accurate in some edge case.
         */
        initSim();

        updateInputs(new DriveIOInputs());

        super.zeroEncoders();
    }

    /** Converts meters to native units */
    private int metersToNative(double position) {
        double wheelRotations = position / Constants.Drivetrain.WHEEL_CIRCUMFERENCE;
        double motorRotations = wheelRotations * Constants.Drivetrain.SHAFT_TO_WHEEL_GEAR_RATIO;
        double nativeUnits = motorRotations * Constants.Drivetrain.TALON_UNITS_PER_ROTATION;
        return (int)(nativeUnits);
    }

    /** Converts m/s velocity to native units */
    private int velocityToNativeVelocity(double velocity) {
        return (int)(metersToNative(velocity) / 10); // Convert m/s to m/100ms
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        super.updateInputs(inputs);

        /* Pass the robot battery voltage to the simulated Talon FXs */
        lSim.setBusVoltage(RobotController.getBatteryVoltage());
        rSim.setBusVoltage(RobotController.getBatteryVoltage());
        
        // Set outputs, motor inputs, inverting the right (setInverted doesn't work here).        
        sim.setInputs(lSim.getMotorOutputLeadVoltage(),
            -rSim.getMotorOutputLeadVoltage()
        );
        
        /* Advance the model by 20ms */
        sim.update(0.02);
        
        /* Set odometry for position and velocity based on simulation. */
        lSim.setIntegratedSensorRawPosition(
            metersToNative(
                sim.getLeftPositionMeters()
        ));
        lSim.setIntegratedSensorVelocity(
            velocityToNativeVelocity(
                sim.getLeftVelocityMetersPerSecond()
        ));

        rSim.setIntegratedSensorRawPosition(
            metersToNative(
                -sim.getRightPositionMeters()
        ));
        rSim.setIntegratedSensorVelocity(
            velocityToNativeVelocity(
                -sim.getRightVelocityMetersPerSecond()
        ));
        
        simAngle.set(-sim.getHeading().getDegrees());
        
    }

}

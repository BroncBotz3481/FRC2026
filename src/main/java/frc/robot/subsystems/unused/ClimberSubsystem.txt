
package frc.robot.subsystems.unused;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDConstants;
import frc.robot.Constants.ClimberConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

// TODO: Example with absolute encoders

/**
 * Exponentially profiled elevator subsystem. The elevator represented by this class does NOT have
 * an absolute encoder! This subsystem has a "self-homing" command, more details in the function
 * description.
 */
public class ClimberSubsystem extends SubsystemBase {
    /**
     * {@link SmartMotorControllerConfig} for the elevator motor.
     */
   /* */ private final SmartMotorControllerConfig smcConfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    // Mechanism Circumference is the distance traveled by each mechanism rotation converting
                    // rotations to meters.
                    .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
                    // Feedback Constants (PID Constants)
                    .withClosedLoopController(
                            ClimberConstants.kP,
                            ClimberConstants.kI,
                            ClimberConstants.kD,
                            DegreesPerSecond.of(90),
                            DegreesPerSecondPerSecond.of(45))
                    .withSimClosedLoopController(
                            ClimberConstants.kPSim,
                            ClimberConstants.kISim,
                            ClimberConstants.kDSim,
                            DegreesPerSecond.of(90),
                            DegreesPerSecondPerSecond.of(45))
                    // Feedforward Constants
                    .withFeedforward(
                            new ArmFeedforward(ClimberConstants.kS, ClimberConstants.kG, ClimberConstants.kV))
                    .withSimFeedforward(
                            new ArmFeedforward(
                                    ClimberConstants.kSSim, ClimberConstants.kGSim, ClimberConstants.kVSim))
                    // Gearing from the motor rotor to final shaft.
                    // In this example GearBox.fromReductionStages(3,4) is the same as
                    // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your
                    // motor.
                    // You could also use .withGearing(12) which does the same thing.
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
                    // Motor properties to prevent over currenting.
                    .withMotorInverted(false)
                    .withIdleMode(MotorMode.BRAKE)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25))
                    .withStartingHeight(Meters.of(0.5))
                    .withMomentOfInertia(Inches.of(4), ClimberConstants.mass);


    private SparkMax spark = new SparkMax(CanIDConstants.climberCanID, MotorType.kBrushless);
    private SmartMotorController sparkSmartMotorController =
            new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);
    private ElevatorConfig elevconfig =
            new ElevatorConfig()
                    .withHardLimits(ClimberConstants.hardLimitMin, ClimberConstants.hardLimitMax)
                    .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH);

    private Elevator elevator = new Elevator(elevconfig,sparkSmartMotorController);

    /**
     * Set the height of the elevator.
     *
     * @param angle Distance to go to.,
     */
    public Command setHeight(Distance height) {
        return elevator.setHeight(height);
    }

    /**
     * Move the elevator up and down.
     *
     * @param dutycycle [-1, 1] speed to set the elevator too.
     */
    public Command set(double dutycycle) {
        return elevator.set(dutycycle);
    }

    /**
     * Run sysId on the {@link Elevator}
     */
    public Command sysId() {
        return elevator.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
    }

    /**
     * Creates a new ExampleSubsystem.
     */
    public ClimberSubsystem() {
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    public Command climb() {
        return null;
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        elevator.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        elevator.simIterate();
    }
}

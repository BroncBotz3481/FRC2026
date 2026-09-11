package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDConstants;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class AgitatorSubsystem extends SubsystemBase {

    private final SparkMax agitatorMotor = new SparkMax(CanIDConstants.agitatorID, MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig =
            new SmartMotorControllerConfig(this)
                    .withClosedLoopController(0, 0, 0)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3)))
                    .withIdleMode(MotorMode.COAST)
                    .withTelemetry("AgitatorMotor", TelemetryVerbosity.LOW)
                    .withStatorCurrentLimit(Amps.of(50))//NEO motor & motor preservation
                    .withMotorInverted(false)
                    .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
                    .withSimFeedforward(new SimpleMotorFeedforward(0, 0.5, 0))
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withMomentOfInertia(Inches.of(4), Pounds.of(1));

    private final SmartMotorController motor =
            new SparkWrapper(agitatorMotor, DCMotor.getNEO(1), motorConfig);

    private final FlyWheelConfig agitatorConfig =
            new FlyWheelConfig()
                    .withTelemetry("Agitator", TelemetryVerbosity.HIGH);

    private final FlyWheel agitator = new FlyWheel(agitatorConfig,motor);

    public AgitatorSubsystem() {
    }

    public AngularVelocity getVelocity() {
        return agitator.getSpeed();
    }

    public Command setDutyCycleCommand(double dutyCycle) {
        return agitator.set(dutyCycle);
    }

    public void periodic() {
        agitator.updateTelemetry();
    }

    public void simulationPeriodic() {
        agitator.simIterate();
    }

    public void setDutyCycleSetpoint(double dutycycle) {
        agitator.setDutyCycleSetpoint(dutycycle);
    }

    public void setVelocitySetpoint(AngularVelocity velocity)
    {
        agitator.setMechanismVelocitySetpoint(velocity);
    }
    public Command stopCommand() {
        return agitator.set(0);
    }
}

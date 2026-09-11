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
import frc.robot.Constants.IndexerConstants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class KickerSubsystem extends SubsystemBase {
    private final SparkMax kickerMotor =
            new SparkMax(CanIDConstants.kickerflywheelID, MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig =
            new SmartMotorControllerConfig(this)
                    .withClosedLoopController(0, 0, 0)
                    .withGearing(IndexerConstants.gearingKicker)
                    .withIdleMode(MotorMode.COAST)
                    .withTelemetry("KickerMotor", TelemetryVerbosity.HIGH)
                    //.withStatorCurrentLimit(Amps.of(40))
                    .withStatorCurrentLimit(Amps.of(40))
                    .withMotorInverted(true)//check
                    .withFeedforward(new SimpleMotorFeedforward(0.18, 0.62, 0))
                    .withSimFeedforward(new SimpleMotorFeedforward(0, 0.5, 0)) //30 rpm
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withMomentOfInertia(Inches.of(4), Pounds.of(1));

    private final SmartMotorController motor =
            new SparkWrapper(kickerMotor, DCMotor.getNEO(1), motorConfig);

    private final FlyWheelConfig kickerConfig =
            new FlyWheelConfig()
                    .withTelemetry("Kicker", TelemetryVerbosity.HIGH);

    private final FlyWheel kicker = new FlyWheel(kickerConfig,motor);

    public Command setDutyCycleCommand(double dutyCycle) {
        return kicker.set(dutyCycle);
    }

    public void setDutycycleSetpoint(double dutycycleSetpoint) {
        kicker.setDutyCycleSetpoint(dutycycleSetpoint);
    }

    public Command setVelocityCommand(AngularVelocity rpm) {
        return kicker.run(rpm);
    }

    public void setVelocitySetpoint(AngularVelocity velocity) {
        kicker.setMechanismVelocitySetpoint(velocity);
    }

    public AngularVelocity getRPM() {
        return kicker.getSpeed();
    }

    public Command stopCommand() {
        return kicker.set(0);
    }

    public void periodic() {
        kicker.updateTelemetry();
    }

    public void simulationPeriodic() {
        kicker.simIterate();
    }

}

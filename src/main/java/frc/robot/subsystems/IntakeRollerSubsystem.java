package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
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

public class IntakeRollerSubsystem extends SubsystemBase {

    private final SparkFlex intakeRollerMotor = new SparkFlex(CanIDConstants.intakeRollerID, MotorType.kBrushless);//IntakeRoller is not NEO

    private final SmartMotorControllerConfig motorConfig =
            new SmartMotorControllerConfig(this)
                    .withClosedLoopController(0, 0, 0)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
                    .withIdleMode(MotorMode.COAST)
                    .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH)//not Intake"ARM" motor
                    .withStatorCurrentLimit(Amps.of(40))
                    .withMotorInverted(false)
                    .withFeedforward(new SimpleMotorFeedforward(0,
                     0, 0))
                    .withSimFeedforward(new SimpleMotorFeedforward(0, 0.5, 0))
                    .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorController motor =
            new SparkWrapper(intakeRollerMotor, DCMotor.getNeoVortex(1), motorConfig);

    private final FlyWheelConfig intakeRollerConfig =
            new FlyWheelConfig()
                    .withTelemetry("IntakeRoller", TelemetryVerbosity.LOW);

    private final FlyWheel intakeRoller = new FlyWheel(intakeRollerConfig,motor);

    public IntakeRollerSubsystem() {
    }

    public AngularVelocity getRPM() {
        return intakeRoller.getSpeed();
    }

    public Command setRPM(double rpm) {
        return intakeRoller.run(RPM.of(rpm));
    }

    public Command setDutyCycleCommand(double dutyCycle) {
        return intakeRoller.set(dutyCycle);
    }

    public Command stopCommand() {
        return intakeRoller.set(0);
    }

    public void periodic() {
        intakeRoller.updateTelemetry();
    }

    public void simulationPeriodic() {
        intakeRoller.simIterate();
    }

    public void setDutycycleSetpoint(double dutycycleSetpoint) { intakeRoller.setDutyCycleSetpoint(dutycycleSetpoint);}

    public void setVelocitySetpoint(AngularVelocity intakeRollerRPM) { intakeRoller.setMechanismVelocitySetpoint(intakeRollerRPM);}
}
// public Command sysId() {
//     return intakeRoller.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));
//   }

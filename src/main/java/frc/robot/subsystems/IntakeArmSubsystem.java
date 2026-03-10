package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GroundConstants;
import frc.robot.Setpoints;
import frc.robot.Setpoints.Intake;
import java.util.function.BooleanSupplier;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeArmSubsystem extends SubsystemBase
{

  private Angle absoluteEncoderZeroOffset = Setpoints.Intake.intakeArmAbsZeroOffset;

  private SparkMax                   m_masterMotor   = new SparkMax(Constants.CanIDConstants.intakeArmID,
                                                                    MotorType.kBrushless);
  private SparkAbsoluteEncoder       absoluteEncoder = m_masterMotor.getAbsoluteEncoder();
  private SparkMax                   m_slaveMotor    = new SparkMax(Constants.CanIDConstants.intakeArmFollowerID,
                                                                    MotorType.kBrushless);
  private SmartMotorControllerConfig masterConfig    = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(1.35, 0, 0)
      .withFeedforward(new ArmFeedforward(0.15, 0, 0))
      .withSimClosedLoopController(20, 0, 0)
      .withSimFeedforward(new ArmFeedforward(1, 0, 1))
      .withTelemetry("IntakeArmMotor", TelemetryVerbosity.HIGH)
      .withGearing(GroundConstants.gearing)
      .withMotorInverted(true)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(40))
      .withStartingPosition(Degrees.zero())
      .withSoftLimit(GroundConstants.softLowerLimit, GroundConstants.softUpperLimit);

  private SmartMotorControllerConfig slaveConfig = masterConfig.clone()
                                                               .withMotorInverted(false)
                                                               // Set custom FF here
                                                               .withTelemetry("IntakeArmSlaveMotor",
                                                                              TelemetryVerbosity.HIGH);


  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController slaveMotorController  = new SparkWrapper(m_slaveMotor, DCMotor.getNEO(2), slaveConfig);
  private SmartMotorController masterMotorController = new SparkWrapper(m_masterMotor, DCMotor.getNEO(2),
                                                                        masterConfig.withLooselyCoupledFollowers(
                                                                            slaveMotorController));


  private ArmConfig armCfg = new ArmConfig(masterMotorController)
      // Hard limit is applied to the simulation.
      .withHardLimit(Degrees.of(-20), GroundConstants.hardUpperLimit)
      // Length and mass of your arm for sim.
      .withLength(GroundConstants.length)
      .withMass(Pounds.of(1))
      // Telemetry name and verbosity for the arm.
      .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH)
      .withStartingPosition(Degrees.zero());
      


  // Arm Mechanism
  private Arm arm = new Arm(armCfg);

  /**
   * Creates a new ExampleSubsystem.
   */
  public IntakeArmSubsystem()
  {
    boolean useAbsoluteEncoder = false;
    // Synchronize the motor controllers to become the same angle.
    Angle armAngle = Rotations.of(absoluteEncoder.getPosition()).minus(absoluteEncoderZeroOffset);
    if (!useAbsoluteEncoder)
    {armAngle = Intake.intakeArmStartAngle;}
    masterMotorController.setEncoderPosition(armAngle);
    slaveMotorController.setEncoderPosition(armAngle);
  }

  public Angle getAngle()
  {
    return arm.getAngle();
  }

  public Command setAngleCommand(Angle angle)
  {
    return arm.setAngle(angle);
//    return run(()->setAngleSetpoint(angle)).withName("SetAngleCommand");
  }

  public void setAngleSetpoint(Angle angle)
  {
    arm.setMechanismPositionSetpoint(angle);
//    masterMotorController.setPosition(angle);
//    slaveMotorController.setPosition(angle);
  }

  public Command setDutyCycleCommand(double dutyCycle)
  {
    return run(() -> {
      masterMotorController.setDutyCycle(dutyCycle);
      slaveMotorController.setDutyCycle(dutyCycle);
    }).withName("SetDutyCycle");
  }

  public void setDutyCycleSetpoint(double dutyCycle)
  {
    arm.setDutyCycleSetpoint(dutyCycle);
    slaveMotorController.setDutyCycle(dutyCycle);
  }

  @Override
  public void periodic()
  {
    arm.updateTelemetry();
    slaveMotorController.updateTelemetry();
  }

  @Override
  public void simulationPeriodic()
  {
    arm.simIterate();
    slaveMotorController.simIterate();
  }

  public BooleanSupplier aroundAngle(Angle angle)
  {
    return arm.isNear(angle, GroundConstants.tolerationAngle);
  }

  public Command setVoltageCommand(Voltage left, Voltage right)
  {
    return run(() -> {
      masterMotorController.setVoltage(left);
      slaveMotorController.setVoltage(right);
    }).finallyDo(() -> {
      masterMotorController.setVoltage(Volts.of(0));
      slaveMotorController.setVoltage(Volts.of(0));
    }).withName("SetBothVoltage");
  }

  public Command setVoltageCommand(Voltage volts)
  {
    return setVoltageCommand(volts, volts);
  }

}

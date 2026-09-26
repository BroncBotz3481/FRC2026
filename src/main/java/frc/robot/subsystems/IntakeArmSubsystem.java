package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GroundConstants;
import frc.robot.Setpoints;
import frc.robot.Setpoints.Intake;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.units.YUnits;

public class IntakeArmSubsystem extends SubsystemBase
{
  private SparkMax             m_masterMotor   = new SparkMax(Constants.CanIDConstants.intakeArmID,
                                                              MotorType.kBrushless);
  private SparkAbsoluteEncoder m_masterAbsoluteEncoder = m_masterMotor.getAbsoluteEncoder();
  private SparkMax             m_followerMotor = new SparkMax(Constants.CanIDConstants.intakeArmFollowerID,
                                                              MotorType.kBrushless);
   private SparkAbsoluteEncoder m_followerAbsoluteEncoder = m_followerMotor.getAbsoluteEncoder();

  private SmartMotorControllerConfig followerConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(2, 0, 2) 
      .withFeedforward(new ArmFeedforward(0.01, 0.02, 0, 0)) 
      .withSimClosedLoopController(10, 0, 0)
      .withSimFeedforward(new ArmFeedforward(0.25, 0, 0.25)) //this just get it faster
      .withGearing(GroundConstants.gearing)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(35))
      .withMotorInverted(false)
      .withStartingPosition(Intake.intakeArmStartAngle)
      .withTelemetry("IntakeArmFollowerMotor", TelemetryVerbosity.HIGH)
      //.withTrapezoidalProfile(RPM.of(15000),YUnits.RPMPerSecond.of(20000))
      //.withExternalEncoder(m_followerAbsoluteEncoder)
      // .withSoftLimit(GroundConstants.softLowerLimit, GroundConstants.softUpperLimit)//5 deg to 65 deg
      //Soft limit is 2 degrees!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      // .withExternalEncoderInverted(false)
     //.withExternalEncoderZeroOffset(Degrees.of(10.68 )) // Remove if configured in REV HW Client
    //  .withUseExternalFeedbackEncoder(false)
    //.withVendorConfig(new SparkMaxConfig().apply(new AbsoluteEncoderConfig().zeroCentered(true)))
      .withResetPreviousConfig(true);


  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController       followerMotorController = new SparkWrapper(m_followerMotor,
                                                                                DCMotor.getNEO(2),
                                                                                followerConfig);

  private SmartMotorControllerConfig masterConfig            = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(2, 0, 2)
      .withFeedforward(new ArmFeedforward(0.01, 0.02, 0, 0))
      .withSimClosedLoopController(10, 0, 0)
      .withSimFeedforward(new ArmFeedforward(
      0.25, 0, 0.25))
      .withTelemetry("IntakeArmMotor", TelemetryVerbosity.LOW)
      .withGearing(GroundConstants.gearing)
      .withMotorInverted(true)
      .withIdleMode(MotorMode.BRAKE)
      .withStartingPosition(Intake.intakeArmStartAngle)
      .withStatorCurrentLimit(Amps.of(40))
      .withLooselyCoupledFollowers(followerMotorController)
      // .withTrapezoidalProfile(RPM.of(15000),YUnits.RPMPerSecond.of(20000))
      //.withExternalEncoder(m_masterAbsoluteEncoder)
      // .withSoftLimit(GroundConstants.softLowerLimit, GroundConstants.softUpperLimit)
    // .withExternalEncoderInverted(true)
    .withVendorConfig(new SparkMaxConfig().apply(new AbsoluteEncoderConfig().zeroCentered(true)))

     //.withExternalEncoderZeroOffset(Degrees.of(-19.31)) // Remove if configured in REV HW Client
      .withResetPreviousConfig(true)
      .withStartingPosition(Degrees.of(67));

  private SmartMotorController       masterMotorController   = new SparkWrapper(m_masterMotor, DCMotor.getNEO(2),
                                                                                masterConfig);


  private ArmConfig armCfg = new ArmConfig()
      // Hard limit is applied to the simulation.
      .withHardLimits(GroundConstants.hardLowerLimit, GroundConstants.hardUpperLimit)
      // Length and mass of your arm for sim.
      .withLength(GroundConstants.length)
      // Telemetry name and verbosity for the arm.
      .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH);


  // Arm Mechanism
  private Arm arm = new Arm(armCfg,masterMotorController);

  /**
   * Creates a new ExampleSubsystem.
   */
  public IntakeArmSubsystem()
  {
  //  followerMotorController.setEncoderPosition(Rotations.of(m_followerAbsoluteEncoder.getPosition()));
  //  masterMotorController.setEncoderPosition(Rotations.of(m_masterAbsoluteEncoder.getPosition()));

  }

  public Angle getAngle()
  {
    return arm.getAngle();
  }
  public Command setAngle(Angle angle)
  {
    return arm.setAngle(angle).until(()->arm.getAngle().isNear(angle, 3));
  }

  public Command setAngleCommand(Angle angle)
  {
    return run(() -> setAngleSetpoint(angle)).withName("SetAngleCommand");
  }

  public void resetEncoder(){
    masterMotorController.setEncoderPosition(Degrees.of(0));
    followerMotorController.setEncoderPosition(Degrees.of(0));
  }

  public Command resetEncoderCommand(){
    return run(()-> resetEncoder());
  }

  public void setAngleSetpoint(Angle angle)
  {
    masterMotorController.setPosition(angle);
    followerMotorController.setPosition(angle);
  }

  public Command setDutyCycleCommand(double dutyCycle)
  {
    return setDutyCycleCommand(dutyCycle, dutyCycle);
  }

  public Command setDutyCycleCommand(double left, double right)
  {

    return run(() -> {
      masterMotorController.setDutyCycle(left);
      followerMotorController.setDutyCycle(right);
    }).finallyDo(() -> {
      masterMotorController.setDutyCycle((0));
      followerMotorController.setDutyCycle((0));
    }).withName("SetBothDutyCycle");
  }

  public Command setDutyCycleCommand(Supplier<Double> left, Supplier<Double> right)
  {

    double multiplier = left.get() < 0.2 && right.get() < 0.2 ? -0.2 : -0.2;

    return run(() -> {
      masterMotorController.setDutyCycle(left.get()* multiplier);
      followerMotorController.setDutyCycle(right.get()* multiplier);
    }).withName("SetBothDutyCycle");
  }

  public void setDutyCycleSetpoint(double dutyCycle)
  {
    masterMotorController.setDutyCycle(dutyCycle);
    followerMotorController.setDutyCycle(dutyCycle);
  }

  @Override
  public void periodic()
  {
    // SmartDashboard.putNumber("Maste Absolute Encoder Value(deg)",Rotations.of(m_masterAbsoluteEncoder.getPosition()).in(Degrees));
    // SmartDashboard.putNumber("Follower Absolute Encoder(deg)",Rotations.of(m_followerAbsoluteEncoder.getPosition()).in(Degrees));
    arm.updateTelemetry();
    followerMotorController.updateTelemetry();
  }

  @Override
  public void simulationPeriodic()
  {
    arm.simIterate();
    followerMotorController.simIterate();
  }

  public BooleanSupplier aroundAngle(Angle angle)
  {
    return arm.isNear(angle, GroundConstants.tolerationAngle);
  }

  public Command setVoltageCommand(Voltage left, Voltage right)
  {
    return run(() -> {
      masterMotorController.setVoltage(left);
      followerMotorController.setVoltage(right);
    }).finallyDo(() -> {
      masterMotorController.setVoltage(Volts.of(0));
      followerMotorController.setVoltage(Volts.of(0));
    }).withName("SetBothVoltage");
  }

  public Command setVoltageCommand(Voltage volts)
  {
    return setVoltageCommand(volts, volts);
  }


}

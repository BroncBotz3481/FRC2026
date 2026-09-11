package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
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

public class IndexerSubsystem extends SubsystemBase {

  private final SparkMax indexMotor =
      new SparkMax(CanIDConstants.indexerflywheelID, MotorType.kBrushless);

    

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(
              0, 0, 0)
          .withGearing(IndexerConstants.gearingIndexer) //always ask mech whether there's extra chain or gears that need to be take in account for :)
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH)
          //.withStatorCurrentLimit(Amps.of(40))
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)//check
          // .withClosedLoopRampRate(Seconds.of(0.25))  //limits the speed, don't add it
          // .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0.18, 0.62, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0.5, 0))
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withMomentOfInertia(Inches.of(4), Pounds.of(1));

  private final SmartMotorController motor =
      new SparkWrapper(indexMotor, DCMotor.getNEO(1), motorConfig);

  private final FlyWheelConfig flywheelConfig =
      new FlyWheelConfig()//Mass & Diameter is for sim only
          .withTelemetry("Indexer", TelemetryVerbosity.HIGH);
          //.withSoftLimit(RPM.of(-5000), RPM.of(5000))
          //.withSpeedometerSimulation(RPM.of(7500));

  private final FlyWheel indexer = new FlyWheel(flywheelConfig,motor);

  public IndexerSubsystem() {}

  public Command setDutyCycleCommand(double dutyCycle) {
    return indexer.set(dutyCycle);
  }
  
  public Command setVeloctiyCommand(AngularVelocity velocity){
     return indexer.run(velocity);
  }

  public Command stopCommand(){
    return indexer.set(0);
  }

  public AngularVelocity getRPM(){
    return indexer.getSpeed();
  }

  public void periodic()
  {
    indexer.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    indexer.simIterate();
  }

  public void setVelocitySetpoint(AngularVelocity velo) {
    indexer.setMechanismVelocitySetpoint(velo);
  }

  public void setDutyCycleSetpoint(double dutyCycle) {
      indexer.setDutyCycleSetpoint(dutyCycle);
  }
}
//Besides turretFlywheel, indexer, kicker, agitator, intakeRoller are all modeled as a YAMS flywheel
//For sim, start w/ increasin kv(pid = 0)
//Add periodic() and simulationPeriodic() for sim updates

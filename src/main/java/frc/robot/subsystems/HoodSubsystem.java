package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDConstants;
import frc.robot.Constants.HoodConstants;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;

public class HoodSubsystem extends SubsystemBase {//Modeled as a pivot, since it's not really affected by gravity

  private final SparkMax hoodMotor = new SparkMax(CanIDConstants.hoodID, MotorType.kBrushless);
  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(2,0,0)
              //DegreesPerSecond.of(180),  
              //DegreesPerSecondPerSecond.of(90))  //Don't add it unless it's not increasing fast enough
          .withSimClosedLoopController(0,0,0)
              //DegreesPerSecond.of(180),
              //DegreesPerSecondPerSecond.of(90))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withVoltageCompensation(Volts.of(12))
          .withMotorInverted(false)
          .withStartingPosition(Degrees.of(0))
          //.withClosedLoopRampRate(Seconds.of(0.25)) //Don't add it unless it's too fast(limit the rate & help slow it down)->check yams docs
          //.withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0,0,0))
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withSoftLimits(HoodConstants.softLimitMin, HoodConstants.softLimitMax)
          .withStartingPosition(Degrees.of(0))
          .withMomentOfInertia(HoodConstants.MOIInKilogram);

  private final SmartMotorController motor =
      new SparkWrapper(hoodMotor, DCMotor.getNEO(1), motorConfig);

 /*  private final MechanismPositionConfig robotToMechanism =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(1.5))
          .withMaxRobotLength(Meters.of(0.75))
          .withRelativePosition(new Translation3d(Meters.of(-0.25), // back from robot center
                                                  Meters.of(0),     // centered left/right
                                                  Meters.of(0.5))); // up from the floor reference*/

  private final PivotConfig m_config =
      new PivotConfig()
          .withHardLimits(HoodConstants.hardLimitMin, HoodConstants.hardLimitMax)
          .withTelemetry("Hood", TelemetryVerbosity.HIGH);
          
          //.withMOI(Inches.of(7), Pounds.of(2))
        

  private final Pivot hood = new Pivot(m_config,motor);

  public HoodSubsystem() {}

  public Command setDegreeCommand(double degree) {
    return hood.setAngle(Degrees.of(degree));
  }

  public void setAngleSetpoint(Angle degree) {
      hood.setMechanismPositionSetpoint(degree);
  }

  public Angle getAngle() {
    return hood.getAngle();
  }

  public void setDutyCycleSetpoint(double dutyCycle) {
    hood.setDutyCycleSetpoint(dutyCycle);
  } 

  public Command setDutyCycle(double dutyCycle) {
    return hood.set(dutyCycle);
  }

  public Command stopCommand(){
    return hood.set(0);
  }
  @Override
  public void periodic() {
    hood.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    hood.simIterate();
  }


}
//Live tuning, so we don't really need sysId
//public Command sysId() {
  //   return hood.sysId(
  //       Volts.of(4.0), // maximumVoltage
  //       Volts.per(Second).of(0.5), // step
  //       Seconds.of(8.0) // duration
  //       );
  // }

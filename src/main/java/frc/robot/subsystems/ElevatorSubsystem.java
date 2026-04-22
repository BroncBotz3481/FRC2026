	// Copyright (c) FIRST and other WPILib contributors.
	// Open Source Software; you can modify and/or share it under the terms of
	// the WPILib BSD license file in the root directory of this project.
	
	package frc.robot.subsystems;
	
	import static edu.wpi.first.units.Units.Amps;
	import static edu.wpi.first.units.Units.Meters;
	import static edu.wpi.first.units.Units.MetersPerSecond;
	import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
	import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
	import static edu.wpi.first.units.Units.Second;
	import static edu.wpi.first.units.Units.Seconds;
	import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
	import com.revrobotics.spark.SparkMax;
	
	
	import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
	import edu.wpi.first.wpilibj2.command.Command;
	import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
	import yams.mechanisms.config.ElevatorConfig;
	import yams.mechanisms.positional.Elevator;
	import yams.motorcontrollers.SmartMotorController;
	import yams.motorcontrollers.SmartMotorControllerConfig;
	import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
	import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
	import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
	import yams.motorcontrollers.local.SparkWrapper;
	
	public class ElevatorSubsystem extends SubsystemBase {

	
	  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
	    .withControlMode(ControlMode.CLOSED_LOOP)
	    // Mechanism Circumference is the distance traveled by each mechanism rotation converting rotations to meters.
	    .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
	    // Feedback Constants (PID Constants)
	    .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
	    .withSimClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
	    // Feedforward Constants
	    .withFeedforward(new ElevatorFeedforward(0, 0, 0))
	    .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
	    // Telemetry name and verbosity level
	    .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
	    // Gearing from the motor rotor to final shaft.
	    // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
	    // You could also use .withGearing(12) which does the same thing.
	    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
	    // Motor properties to prevent over currenting.
	    .withMotorInverted(false)
	    .withIdleMode(MotorMode.BRAKE)
	    .withStatorCurrentLimit(Amps.of(40))
	    .withClosedLoopRampRate(Seconds.of(0.25))
	    .withOpenLoopRampRate(Seconds.of(0.25));
	
		private SparkMax             m_elevatorMotor   = new SparkMax(11,
                                                              MotorType.kBrushless);
  		private SparkAbsoluteEncoder m_elevatorMotorEncoder = m_elevatorMotor.getAbsoluteEncoder();
	  // Create our SmartMotorController from our Spark and config with the NEO.
	  private SmartMotorController sparkSmartMotorController = new SparkWrapper(m_elevatorMotor, DCMotor.getNEO(1), smcConfig);
	
	  private ElevatorConfig elevconfig = new ElevatorConfig(sparkSmartMotorController)
	      .withStartingHeight(Meters.of(0.5))
	      .withHardLimits(Meters.of(0), Meters.of(3))
	      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
	      .withMass(Pounds.of(16));
	
	  // Elevator Mechanism
	  private Elevator elevator = new Elevator(elevconfig);
	
	  /**
	   * Set the height of the elevator and does not end the command when reached.
	   * @param angle Distance to go to.
	   * @return a Command
	   */
	  public Command setHeightCommand(Distance height) { return elevator.run(height);}
	  
	  /**
	   * Set the height of the elevator and ends the command when reached, but not the closed loop controller.
	   * @param height Distance to go to.
	   * @param tolerance Distance tolerance for completion.
	   * @return A Command
	   */
	  public Command setHeightAndStop(Distance height, Distance tolerance) { return elevator.runTo(height, tolerance);}
	  
	  /**
	   * Set the elevators closed loop controller setpoint.
	   * @param angle Distance to go to.
	   */
	  public void setHeightSetpoint(Distance height) { elevator.setMeasurementPositionSetpoint(height);}
	
	  /**
	   * Move the elevator up and down.
	   * @param dutycycle [-1, 1] speed to set the elevator too.
	   */
	  public Command set(double dutycycle) { return elevator.set(dutycycle);}
	
	  /**
	   * Run sysId on the {@link Elevator}
	   */
	  public Command sysId() { return elevator.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}
	
	  /** Creates a new ExampleSubsystem. */
	  public ElevatorSubsystem() {}
	
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
	
	  /**
	   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
	   *
	   * @return value of some boolean subsystem state, such as a digital sensor.
	   */
	  public boolean exampleCondition() {
	    // Query some boolean state, such as a digital sensor.
	    return false;
	  }

	   public Distance getHeight()
  {
    return elevator.getHeight();
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
	
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OutakeCommand;
import frc.robot.commands.ShootKickIndexCommand;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.systems.field.FieldConstants;
import swervelib.SwerveInputStream;


public class RobotContainer
{

  private final SendableChooser<Command> autoChooser;

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  // private final TurretVisualizer turretVisualizer =
  //     new TurretVisualizer(() -> new Pose3d(drivebase.getPose()), drivebase::getFieldVelocity);

  private final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem();
  private final IntakeArmSubsystem    intakeArm    = new IntakeArmSubsystem();

  private final HoodSubsystem     hood           = new HoodSubsystem();
  private final FlywheelSubsystem turretFlywheel = new FlywheelSubsystem();

  private final IndexerSubsystem  indexer  = new IndexerSubsystem();
  private final AgitatorSubsystem agitator = new AgitatorSubsystem();
  private final KickerSubsystem   kicker   = new KickerSubsystem();

  public static Timer                 timerThing           = new Timer();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController   =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
                           drivebase.getSwerveDrive(),
                           () -> m_driverController.getLeftY() * -1,
                           () -> m_driverController.getLeftX() * -1)
                       .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
                       .deadband(OperatorConstants.DEADBAND)
                       .scaleTranslation(0.8)
                       .allianceRelativeControl(true);


  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity.copy()
                                                                                               // Fallback
                                                                                               .aim(FieldConstants.Hub.getHubPose())
                                                                                               .aimHeadingOffset(
                                                                                                   Rotation2d.k180deg)
                                                                                               .aimWhile(
                                                                                                   m_driverController.leftBumper()));

  Command autoSwerveInputStream = drivebase.driveFieldOriented(driveAngularVelocity.copy()
                                                                                   // Fallback
                                                                                   .aim(FieldConstants.Hub.getHubPose())
                                                                                   .aimHeadingOffset(Rotation2d.k180deg)
                                                                                   .aimWhile(() -> true));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
    defaultCommands();

    // Named commands do NOT run with path's. They are inbetween paths.
    NamedCommands.registerCommand("ShootCommand",
                                  new ShootKickIndexCommand(turretFlywheel,
                                                            kicker,
                                                            indexer,
                                                            agitator,
                                                            hood,
                                                            Setpoints.Shooter.hubRPM, 
                                                            Setpoints.Hood.hubDegree).withTimeout(Seconds.of(6)));

    new EventTrigger("IntakeStart").onTrue(new IntakeCommand(intakeArm, intakeRoller, agitator));
    new EventTrigger("IntakeStop").onTrue(intakeArm.setAngleCommand(Setpoints.Intake.intakeArmAngleUp)
                                                   .alongWith(intakeRoller.stopCommand()));
  }


  public void defaultCommands()
  {

    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    kicker.setDefaultCommand(kicker.setDutyCycleCommand(-0)); // Set -0.3 before on field
    agitator.setDefaultCommand(agitator.setDutyCycleCommand(0));
    indexer.setDefaultCommand(indexer.setDutyCycleCommand(-0)); // Set -0.3 before on field
    turretFlywheel.setDefaultCommand(turretFlywheel.setDutyCycle(0));
//    intakeArm.setDefaultCommand(intakeArm.setVoltageCommand(Volts.of(0)));
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  //controlssss
  private void configureBindings()
  {

    // m_driverController.button(1).whileTrue(new AutoAimCommand(drivebase, driveAngularVelocity).withTimeout(5));
    // m_driverController.button(2).whileTrue(drivebase.lockPos().withTimeout(5));
    // m_driverController.button(3).whileTrue(new ShootKickIndexCommand(turretFlywheel, kicker, indexer, agitator, Setpoints.Shooter.hubRPM).withTimeout(5));
    // m_driverController.button(4).whileTrue(new ShootKickIndexCommand(turretFlywheel, kicker, indexer, agitator, drivebase));
    // m_driverController.button(5).whileTrue(new IntakeCommand(intakeArm, intakeRoller, agitator));
    // m_driverController.button(6).whileTrue(new OutakeCommand(intakeArm, intakeRoller, agitator));
    //m_driverController.b().whileTrue(intakeArm.setAngleCommand(Degrees.of(45)));
    //m_driverController.y().whileTrue(intakeArm.setAngleCommand(Degrees.of(0)));

    if (DriverStation.isTest())
    {
      // Test mode controls.
      m_operatorController.x().whileTrue(intakeArm.setVoltageCommand(Volts.of(1)));
      m_operatorController.y().whileTrue(intakeArm.setVoltageCommand(Volts.of(-1)));
      m_operatorController.b().whileTrue(intakeArm.setVoltageCommand(Volts.of(1), Volts.of(0)));
      m_operatorController.a().whileTrue(intakeArm.setVoltageCommand(Volts.of(-1), Volts.of(0)));
      m_operatorController.leftBumper().whileTrue(intakeArm.setVoltageCommand(Volts.of(0), Volts.of(-1)));
      m_operatorController.rightBumper().whileTrue(intakeArm.setVoltageCommand(Volts.of(0), Volts.of(1)));

    } else
    {
      // Regular driver and operator controls.
//      m_driverController.a().whileTrue(new AutoAimCommand(drivebase, driveAngularVelocity));
//      m_driverController.x().whileTrue(drivebase.lockPos());
//      m_driverController.start().and(m_driverController.back()).onTrue(drivebase.zeroGyroWithAlliance());
//
//      m_operatorController.rightTrigger(0.2).whileTrue(new ShootKickIndexCommand(turretFlywheel,
//                                                                                 kicker,
//                                                                                 indexer,
//                                                                                 agitator,
//                                                                                 hood,
//                                                                                 drivebase));
//
//      m_operatorController.povDown().whileTrue(new OutakeCommand(intakeArm, intakeRoller, agitator));
//
//      m_operatorController.x().whileTrue(new ShootKickIndexCommand(turretFlywheel,
//                                                                   kicker,
//                                                                   indexer,
//                                                                   agitator,
//                                                                   hood,
//                                                                   Setpoints.Shooter.hubRPM,
//                                                                   Setpoints.Hood.hubDegree));
//
//      m_operatorController.a().whileTrue(intakeArm.setAngleCommand(Degrees.of(0)));
//      m_operatorController.b().whileTrue(intakeArm.setAngleCommand(Degrees.of(50)));
      m_driverController.button(1).whileTrue(new IntakeCommand(intakeArm, intakeRoller, agitator));

    }


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*  */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
  }

}

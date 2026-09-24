package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.struct.parser.ParseException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AlignmentConstants.DriveToPose;
import frc.robot.Constants;
import frc.robot.subsystems.unused.TurretVisualizer;
import frc.robot.systems.ShooterTargetingSystem;
import frc.robot.systems.field.AllianceFlipUtil;
import frc.robot.systems.field.FieldConstants;
import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightResults;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase
{

  /**
   * Creates a new ExampleSubsystem.
   */
  File directory = new File(Filesystem.getDeployDirectory(), "swerve");

  private TurretVisualizer       turretVisualizer;
  private ShooterTargetingSystem shooterAimer;

  public static SwerveDrive            swerveDrive;
  private       Limelight              limelight_swerve; //Instance Variables
  private       LimelightPoseEstimator limelightPoseEstimator_swerve;
  private       Limelight              limelight_turret;
  private       LimelightPoseEstimator limelightPoseEstimator_turret;

  Field2d m_field2d = new Field2d();

  // Look I am above the constuctor!!!
  public SwerveSubsystem()
  {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    Pose3d initialPose = new Pose3d();

    // error catching
    try
    {
      swerveDrive = new SwerveParser(directory)
          .createSwerveDrive(
              Constants.maxSpeed,
              new Pose2d(
                  new Translation2d(Meter.of(13), Meter.of(4)), Rotation2d.fromDegrees(180)));
      // Alternative method if you don't want to supply the conversion factor via JSON
      // files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
      // angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setModuleStateOptimization(true);

    turretVisualizer = new TurretVisualizer(
        () -> new Pose3d(swerveDrive.getPose()), () -> swerveDrive.getFieldVelocity());
    shooterAimer = new ShooterTargetingSystem(
        new Transform3d(
            0, // back from robot center
            0, // centered left/right
            0.451739, // up from the floor reference
            new Rotation3d()));

    setupPathPlanner();
    setupLimelight(); //I am inside the constuctor !!!!
  } //I am a bracket!

  public void setupLimelight() //I am a method!!
  {
    List<Integer> filteredListTurret = List.of(1, 3, 4, 6, 7,8, 9, 10, 11, 12,13,14,15,16, 17, 18, 19, 20, 21, 22, 23,24, 25, 26, 27, 28,31, 32);
    List<Integer> filteredListSwerve = List.of(1, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 22, 23, 24, 25, 26, 27, 28,29, 30, 31, 32);

    swerveDrive.stopOdometryThread();
    limelight_swerve = new Limelight("limelight"); // We didn't rename the limelight
    limelight_swerve
        .getSettings()
        .withPipelineIndex(0)
        .withCameraOffset(
            new Pose3d( // TODO: Give the right offset here
                        Units.inchesToMeters(-12), // CHANGE MEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
                        Units.inchesToMeters(-12), /// +Right, maybe?
                        Units.inchesToMeters(10),
                        new Rotation3d(0,
                                       Units.degreesToRadians(45),
                                       Units.degreesToRadians(180)))) ///  Roll, Pitch, Yaw
        .withAprilTagIdFilter(filteredListSwerve)
        .save();
    limelightPoseEstimator_swerve = limelight_swerve.createPoseEstimator(EstimationMode.MEGATAG1);

    limelight_turret = new Limelight("limelight-turret"); // We didn't rename the limelight
    limelight_turret
        .getSettings()
        .withPipelineIndex(0)
        .withCameraOffset(
            new Pose3d( // TODO: Give the right offset here
                        Units.inchesToMeters(0),
                        Units.inchesToMeters(0), /// +Right, maybe?
                        Units.inchesToMeters(20.5),
                        new Rotation3d(0, Units.degreesToRadians(45), 0))) ///  Roll, Pitch, Yaw
        .withAprilTagIdFilter(filteredListTurret)
        .save();

    limelightPoseEstimator_turret = limelight_turret.createPoseEstimator(EstimationMode.MEGATAG1);
  } //I am also a bracket!!

  /**
   * Returns a Command that tells the robot to drive forward until the command ends.
   *
   * @return a Command that tells the robot to drive forward until the command ends
   */
  public Command driveForward()
  {
    return run(() -> {
      swerveDrive.drive(new Translation2d(2, 0), 0, false, false);
    })
        .finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  public Command turnRight()
  {
    return run(() -> {
      swerveDrive.drive(new Translation2d(0, 0), getHeadingDegrees() + 90, false, false);
    })
        .finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  public Command turnLeft()
  {
    return run(() -> {
      swerveDrive.drive(new Translation2d(0, 0), getHeadingDegrees() - 90.0, false, false);
    })
        .finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand()
  {
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
  public boolean exampleCondition()
  {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  // I am a bunch a important varaibles
  private int     outofAreaReading       = 0;
  private boolean initialReading         = false;
  private double  lastLLTimestamp_swerve = 0;
  private double  lastLLTimestamp_turret = 0;

  //limelight.local change explosure

  private double updateLimelight(Limelight ll, LimelightPoseEstimator llPoseEst, double llTimestamp, Angle cameraYaw,
                                 String llname)
  {
    ll
        .getSettings()
        .withRobotOrientation(
            new Orientation3d(
                new Rotation3d(swerveDrive.getOdometryHeading().rotateBy(new Rotation2d(cameraYaw))),
                new AngularVelocity3d(DegreesPerSecond.of(0), DegreesPerSecond.of(0), DegreesPerSecond.of(0))))
        .save();

    Optional<PoseEstimate> poseEstimates =
        llPoseEst.getPoseEstimate();
    Optional<LimelightResults> results = ll.getLatestResults();
    if (results.isPresent() && poseEstimates.isPresent())
    {
      LimelightResults result       = results.get();
      PoseEstimate     poseEstimate = poseEstimates.get();
      if (result.valid)
      {
        Pose2d estimatorPose = poseEstimate.pose.toPose2d();
        Pose2d usefulPose    = result.getBotPose2d(Alliance.Blue);
        swerveDrive.field.getObject("Vision").setPose(estimatorPose);
        // TODO: Tune this to be better
        SmartDashboard.putNumber("LimelightTuning/" + llname + "/ambiguity", poseEstimate.getAvgTagAmbiguity());
        if (poseEstimate.getAvgTagAmbiguity() < 0.2
            && // TODO: Change me, i am bad, too low
            poseEstimate.tagCount > 1 && poseEstimate.avgTagDist < Feet.of(18).in(Meters))
        {
          if (llTimestamp != poseEstimate.timestampSeconds)
          {
            var stdDevScale = Math.pow(poseEstimate.avgTagDist, 2.0) / poseEstimate.tagCount;
            // stdDevScale = distance^2/tagsInView$
            // swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.09 * stdDevScale,
            // 0.023 * stdDevScale,
            //  0.07 * stdDevScale));
            swerveDrive.addVisionMeasurement(estimatorPose,
                                             poseEstimate.timestampSeconds);
            return poseEstimate.timestampSeconds;
          }
        }
      }
    }
    return llTimestamp;
  }

  // I am above periodic!!!
  @Override
  public void periodic() //I get called every 20 ms!!!
  {
    outofAreaReading = 0;
    swerveDrive.updateOdometry();

    lastLLTimestamp_swerve = updateLimelight(limelight_swerve,
                                             limelightPoseEstimator_swerve,
                                             lastLLTimestamp_swerve,
                                             Degrees.of(180),
                                             "swerve"); //Hiiiiii, I am a method being called
    lastLLTimestamp_turret = updateLimelight(limelight_turret,
                                             limelightPoseEstimator_turret,
                                             lastLLTimestamp_turret,
                                             Degrees.zero(),
                                             "turret"); // I am not needed if I do not have a turret limelight
    // TODO: Remove me
    distanceToHub();
  }


  public Rotation2d getOdometryHeading()
  {
    return swerveDrive.getOdometryHeading();
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public double getHeadingDegrees()
  {
    return getPose().getRotation().getDegrees();
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  public Supplier<Pose2d> getPoseSupplier()
  {
    return () -> swerveDrive.getPose();
  }

  public Rotation2d getRotation()
  {
    return swerveDrive.getYaw();
  }

  @Override
  public void simulationPeriodic()
  {
    // This method will be called once per scheduler run during simulation
  }

  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }

  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(
        () -> {
          swerveDrive.driveFieldOrientedAndRobotOriented(getRobotVelocity(), getFieldVelocity());
        });
  }

  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          swerveDrive::getPose,
          // Robot pose supplier
          swerveDrive::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          swerveDrive::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally
          // outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic
              // drive
              // trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return new PathPlannerAuto(pathName);
  }

  public Command driveToPose(Supplier<Pose2d> pose)
  {
    double tooCloseMeters = 0.5; // If the bot is too close by this much it needs to drive back a little bit.
    return defer(() -> driveToPose(pose.get()));
  }

  public Command pathndToPose(Supplier<Pose2d> pose)
  {
    return defer(
        () -> {
          PathConstraints constraints = new PathConstraints(
              DriveToPose.maximumVelocityMetersPerSecond,
              DriveToPose.maximumAccelerationMetersPerSecondSquared,
              Degrees.of(DriveToPose.maximumAngularVelocityDegreesPerSecond)
                     .per(Second)
                     .in(RadiansPerSecond),
              Units.degreesToRadians(
                  DriveToPose.maximumAngularAccelerationDegreesPerSecondSquared));
          // Since AutoBuilder is configured, we can use it to build pathfinding commands
          return AutoBuilder.pathfindToPose(
              pose.get(),
              constraints,
              edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
                                           );
        });
  }

  public void stopDriving()
  {
    swerveDrive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  public Command stopDrivingCommand()
  {
    return runOnce(() -> stopDriving());
  }

  public Command driveToPose(Pose2d pose)
  {
    stopDriving();
    boolean                 useSetpointGenerator  = false;
    boolean                 useProfiledController = true;
    boolean                 resetBeforehand       = true;
    Supplier<ChassisSpeeds> robotRelativeSpeeds;
    BooleanSupplier         atTargetPose;

    if (resetBeforehand)
    {
      DriveToPose.driveController.getXController().reset();
      DriveToPose.driveController.getYController().reset();
      DriveToPose.driveController.getThetaController().reset(getPose().getRotation().getRadians());

      if (useProfiledController)
      {
        DriveToPose.profiledDriveController.reset(getSwerveDrive().getPose());
      }
    }
    if (useProfiledController)
    {
      atTargetPose = DriveToPose.profiledDriveController::atReference;
      robotRelativeSpeeds = () -> DriveToPose.profiledDriveController.calculate(getPose(), pose, 0, pose.getRotation());
    } else
    {
      atTargetPose = DriveToPose.driveController::atReference;
      robotRelativeSpeeds = () -> DriveToPose.driveController.calculate(getPose(), pose, 0, pose.getRotation());
    }

    if (useSetpointGenerator)
    {
      try
      {

        return driveWithSetpointGenerator(robotRelativeSpeeds)
            .until(atTargetPose)
            .finallyDo(this::stopDriving);
      } catch (Exception ignored)
      {
        DriverStation.reportWarning("Could not use setpoint generator with drive to pose.", false);
      }
    }

    return run(() -> swerveDrive.drive(robotRelativeSpeeds.get()))
        .until(atTargetPose)
        .finallyDo(this::stopDriving);
    /*
     * // Create the constraints to use while pathfinding
     */
  }

  /**
   * Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.
   *
   * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to achieve.
   * @return {@link Command} to run.
   * @throws IOException                           If the PathPlanner GUI settings is invalid
   * @throws ParseException                        If PathPlanner GUI settings is nonexistent.
   * @throws org.json.simple.parser.ParseException
   */
  private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
  throws IOException, ParseException
  {
    try
    {
      SwerveSetpointGenerator setpointGenerator =

          new SwerveSetpointGenerator(
              RobotConfig.fromGUISettings(), swerveDrive.getMaximumChassisAngularVelocity());

      AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
          new SwerveSetpoint(
              swerveDrive.getRobotVelocity(),
              swerveDrive.getStates(),
              DriveFeedforwards.zeros(swerveDrive.getModules().length)));
      AtomicReference<Double> previousTime = new AtomicReference<>();

      return startRun(
          () -> previousTime.set(Timer.getFPGATimestamp()),
          () -> {
            double newTime = Timer.getFPGATimestamp();
            SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(
                prevSetpoint.get(),
                robotRelativeChassisSpeed.get(),
                newTime - previousTime.get());
            swerveDrive.drive(
                newSetpoint.robotRelativeSpeeds(),
                newSetpoint.moduleStates(),
                newSetpoint.feedforwards().linearForces());
            prevSetpoint.set(newSetpoint);
            previousTime.set(newTime);
          });
    } catch (Exception e)
    {
      return null;
    }
  }

  /**
   * Drive with 254's Setpoint generator; port written by PathPlanner.
   *
   * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
   * @return Command to drive the robot using the setpoint generator.
   */
  public Command driveWithSetpointGeneratorFieldRelative(
      Supplier<ChassisSpeeds> fieldRelativeSpeeds)
  {
    try
    {
      return driveWithSetpointGenerator(
          () -> {
            return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());
          });
    } catch (Exception e)
    {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();
  }

  public Command driveForwards()
  {
    return run(() -> {
      swerveDrive.drive(new Translation2d(1, 0), 0, false, false);
    })
        .finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  public Command driveBackwards()
  {
    return run(() -> {
      swerveDrive.drive(new Translation2d(-1, 0), 0, false, false);
    })

        .finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  public Command lockPos()
  {
    return run(swerveDrive::lockPose);
  }

  public Command drive(Supplier<ChassisSpeeds> driveAngularVelocity)
  {
    return run(
        () -> {
          swerveDrive.drive(driveAngularVelocity.get());
        });
  }

  /**
   * Gets the current 3d pose (position and rotation) of the robot, as reported by odometry. Transforms into a 3d pose
   * assuming on the XY plane.
   *
   * @return
   */
  public Pose3d getPose3d()
  {
    return new Pose3d(swerveDrive.getPose());
  }

  public Command rotateToHeading(Rotation2d rotation2d)
  {
    return run(
        () -> swerveDrive.drive(
            new Translation2d(0, 0),
            swerveDrive
                .getSwerveController()
                .headingCalculate(
                    getHeading().getRadians(),
                    getHeading().getRadians() - rotation2d.getRadians()),
            false,
            true));
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(),
        scaledInputs.getY(),
        headingX,
        headingY,
        getHeading().getRadians(),
        Constants.maxSpeed);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        Constants.maxSpeed);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current robot-relative velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Get the distance to the hub in meters
   *
   * @return Distance to hub in meters.
   */
  public Double distanceToHub()
  {
    // p = robot position, h = hub position, d = desired distance (midRange)
    Translation2d p = getPose().getTranslation();
    Translation2d h = FieldConstants.Hub.getHubTranslation2d();

    // vector from hub to robot: v = p - h
    Translation2d v = p.minus(h);

    // distance ||v||
    double dist = Math.hypot(v.getX(), v.getY());

    SmartDashboard.putNumber("AutoShootRPM/distance/meters", dist);
    return dist;
  }

  public void driveFieldOrientedSetpoint(ChassisSpeeds speeds)
  {
    swerveDrive.driveFieldOriented(speeds);
  }

  public Command resetOdometryCommand(Pose2d odom)
  {
    return runOnce(() -> swerveDrive.resetOdometry(AllianceFlipUtil.apply(odom)));
  }


  public Command zeroGyroWithAlliance()
  {
    return runOnce(() -> {
      zeroGyro();
      DriverStation.getAlliance().ifPresent(alliance -> {
        if (alliance == DriverStation.Alliance.Red)
        {
          swerveDrive.resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.k180deg));
        }
      });
    });
  }

  //public Command resetOdometryCommand(Pose2d odom){
  // return runOnce(()->)}

}

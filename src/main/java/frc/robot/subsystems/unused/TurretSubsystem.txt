package frc.robot.subsystems.unused;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretConstants.PivotConstants;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class TurretSubsystem extends SubsystemBase {

    private static final String RERUN_SEED = "Turret/CRT/RerunSeed";

    private SparkMax turretMotor = new SparkMax(CanIDConstants.turretID, MotorType.kBrushless);

    private final AbsoluteEncoder cancoderB = turretMotor.getAbsoluteEncoder(); // 20t B SparkFlex
    private final DutyCycleEncoder cancoderA = new DutyCycleEncoder(0); // 19 A rio
    // Create a timer to delay CRT run until encoders are ready

    private Timer startUpTimer = new Timer();
    private boolean startTimer = false;
    private boolean delayForCRTDone = false;
    private final Double absPositionASignal;
    private final Double absPositionBSignal;
    private final EasyCRTConfig easyCRTConfig;

    private boolean rotorSeededFromAbs = false;
    private double lastSeededTurretDeg = Double.NaN;
    private double lastSeedError = Double.NaN;
    private double lastAbsA = Double.NaN;
    private double lastAbsB = Double.NaN;
    private String lastSeedStatus = "NOT_ATTEMPTED";

    Limelight limelight;
    LimelightPoseEstimator limelightPoseEstimator;
    private int outofAreaReading = 0;
    private boolean initialReading = false;

    private final SmartMotorControllerConfig motorConfig =
            new SmartMotorControllerConfig(this)
                    .withSimClosedLoopController(
                            PivotConstants.kPSim,
                            PivotConstants.kISim,
                            PivotConstants.kDSim,
                            DegreesPerSecond.of(180),
                            DegreesPerSecondPerSecond.of(90))
                    .withClosedLoopController(
                            PivotConstants.kP,
                            PivotConstants.kI,
                            PivotConstants.kD,
                            DegreesPerSecond.of(180),
                            DegreesPerSecondPerSecond.of(90))
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))//to change
                    .withIdleMode(MotorMode.BRAKE)
                    .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withVoltageCompensation(Volts.of(12))
                    .withMotorInverted(true)
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25))
                    .withFeedforward(new SimpleMotorFeedforward(1, 1, 1, 10))
                    .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorController motor =
            new SparkWrapper(turretMotor, DCMotor.getNEO(1), motorConfig);

    private final MechanismPositionConfig robotToMechanism =
            new MechanismPositionConfig()
                    .withMaxRobotHeight(Meters.of(1.5))
                    .withMaxRobotLength(Meters.of(0.75))
                    .withRelativePosition(new Translation3d(Meters.of(-0.25), // back from robot center
                            Meters.of(0),     // centered left/right
                            Meters.of(0.5))); // up from the floor reference

    private final PivotConfig m_config =
            new PivotConfig()
                    .withHardLimit(Degrees.of(-155), Degrees.of(155))
                    .withSoftLimits(Degrees.of(-135), Degrees.of(135))
                    .withTelemetry("TurretPivot", TelemetryVerbosity.HIGH)
                    .withStartingPosition(Degrees.of(0))
                    .withMechanismPositionConfig(robotToMechanism)
                    .withMOI(Meter.of(0.001), Pounds.of(8));

    private final Pivot turret = new Pivot(m_config,motor);

    // Robot to turret transform, from center of robot to turret.
    private final Transform3d roboToTurret = new Transform3d(Feet.of(-1.5), Feet.of(0), Feet.of(0.5), Rotation3d.kZero);

    public TurretSubsystem() {
        absPositionASignal = (getAbsoluteEncoderWithOffset());
        absPositionBSignal = cancoderB.getPosition();

        easyCRTConfig = buildEasyCrtConfig();
        logCrtConfigTelemetry();
        SmartDashboard.putBoolean(RERUN_SEED, false);

        setupLimelight();
    }

    public Pose2d getPose(Pose2d robotPose) {
        return robotPose.plus(new Transform2d(
                roboToTurret.getTranslation().toTranslation2d(), roboToTurret.getRotation().toRotation2d()));
    }

    public ChassisSpeeds getVelocity(ChassisSpeeds robotVelocity, Angle robotAngle)//the robot heading angle is in regards to the x axis(+x is up, +y is left)
    {
        var robotAngleRads = robotAngle.in(Radians);
        // rotational linear velocity at turret (v = ω × r)
        // vTurretX = vRobotX + ω × r
        double turretVelocityX =
                robotVelocity.vxMetersPerSecond
                        + robotVelocity.omegaRadiansPerSecond
                        * (roboToTurret.getY() * Math.cos(robotAngleRads)
                        - roboToTurret.getX() * Math.sin(robotAngleRads));
        double turretVelocityY =
                robotVelocity.vyMetersPerSecond
                        + robotVelocity.omegaRadiansPerSecond
                        * (roboToTurret.getX() * Math.cos(robotAngleRads)
                        - roboToTurret.getY() * Math.sin(robotAngleRads));
        double turretOmega =
                robotVelocity.omegaRadiansPerSecond
                        + motor.getMechanismVelocity().in(RadiansPerSecond);

        return new ChassisSpeeds(turretVelocityX,
                turretVelocityY,
                turretOmega);
    }
    //robot frame, field frame
    //cnp
  /*double turretVelocityX =
        robotVelocity.vxMetersPerSecond
        + robotVelocity.omegaRadiansPerSecond
          * (roboToTurret.getX() * Math.cos(robotAngleRads)
             - roboToTurret.getY() * Math.sin(robotAngleRads));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
        + robotVelocity.omegaRadiansPerSecond
          * (roboToTurret.getX() * Math.sin(robotAngleRads)
             - roboToTurret.getY() * Math.cos(robotAngleRads));
    */

    public void setupLimelight() {

        // limelight = new Limelight("limelight");
        // limelight
        //     .getSettings()
        //     .withPipelineIndex(0)
        //     .withCameraOffset(
        //         new Pose3d(
        //             Units.inchesToMeters(0),
        //             Units.inchesToMeters(0),
        //             Units.inchesToMeters(0),
        //             new Rotation3d(0, 0, Units.degreesToRadians(0))))
        //     .withAprilTagIdFilter(List.of(17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11))
        //     .save();

        //   limelightPoseEstimator = limelight.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG2);
    }


    public void periodic() {
        turret.updateTelemetry();


        if (!startTimer) {
            startUpTimer.reset();
            startUpTimer.start();
            startTimer = true;
        }

        if (startUpTimer.hasElapsed(3)) {
            delayForCRTDone = true;
        }

        if (SmartDashboard.getBoolean(RERUN_SEED, false) && delayForCRTDone) {
            SmartDashboard.putBoolean(RERUN_SEED, false);
            rerunCrtSeed();
        }
        SmartDashboard.putNumber(
                "Turret/CRT/CurrentPositionDeg", motor.getMechanismPosition().in(Degrees));
        if (!rotorSeededFromAbs && delayForCRTDone) {
            attemptRotorSeedFromCANCoders();
        }
        turret.updateTelemetry();

        SmartDashboard.putNumber("Encoder A Raw", cancoderA.get());
        SmartDashboard.putNumber("Encoder A Adjusted", (getAbsoluteEncoderWithOffset()));
        SmartDashboard.putNumber("Encoder B", cancoderB.getPosition());
        SmartDashboard.putBoolean("Encoder A Raw", rotorSeededFromAbs);
        SmartDashboard.putNumber("Position", getRawAngle().in(Rotations));

        // limelight
        //     .getSettings()
        //     .withRobotOrientation(
        //         new Orientation3d(
        //             new Rotation3d(
        //                 SwerveSubsystem.swerveDrive.getOdometryHeading().rotateBy(Rotation2d.kZero)),
        //             new AngularVelocity3d(
        //                 DegreesPerSecond.of(0), DegreesPerSecond.of(0), DegreesPerSecond.of(0))))
        //     .withCameraOffset(
        //         TurretConstants.cameraOffsetFromRobotCenter.rotateAround(
        //             TurretConstants.turretPivotCenterFromCameraCenter,
        //             new Rotation3d(0, Degrees.of(0).in(Radians), turret.getAngle().in(Radians))))
        //     .save(); // camera pose is the camera pose from the center of robot
        // Optional<PoseEstimate> poseEstimates = limelightPoseEstimator.getPoseEstimate();
        // Optional<LimelightResults> results = limelight.getLatestResults();
        // if (results.isPresent() /* && poseEstimates.isPresent()*/) {
        //   LimelightResults result = results.get();
        //   PoseEstimate poseEstimate = poseEstimates.get();
        //   SmartDashboard.putNumber("Avg Tag Ambiguity", poseEstimate.getAvgTagAmbiguity());
        //   SmartDashboard.putNumber("Min Tag Ambiguity", poseEstimate.getMinTagAmbiguity());
        //   SmartDashboard.putNumber("Max Tag Ambiguity", poseEstimate.getMaxTagAmbiguity());
        //   SmartDashboard.putNumber("Avg Distance", poseEstimate.avgTagDist);
        //   SmartDashboard.putNumber("Avg Tag Area", poseEstimate.avgTagArea);
        //   SmartDashboard.putNumber("Limelight Pose/x", poseEstimate.pose.getX());
        //   SmartDashboard.putNumber("Limelight Pose/y", poseEstimate.pose.getY());
        //   SmartDashboard.putNumber(
        //       "Limelight Pose/degrees", poseEstimate.pose.toPose2d().getRotation().getDegrees());
        //   if (result.valid) {
        //     // Pose2d estimatorPose = poseEstimate.pose.toPose2d();
        //     Pose2d usefulPose = result.getBotPose2d(Alliance.Blue);
        //     double distanceToPose =
        //         usefulPose
        //             .getTranslation()
        //             .getDistance(SwerveSubsystem.swerveDrive.getPose().getTranslation());
        //     if (distanceToPose < 0.5
        //         || (outofAreaReading > 10)
        //         || (outofAreaReading > 10 && !initialReading)) {
        //       if (!initialReading) {
        //         initialReading = true;
        //       }
        //       outofAreaReading = 0;

        //       // System.out.println(usefulPose.toString());
        //       SwerveSubsystem.swerveDrive.setVisionMeasurementStdDevs(
        //           VecBuilder.fill(0.05, 0.05, 0.022));
        //       // System.out.println(result.timestamp_LIMELIGHT_publish);
        //       // System.out.println(result.timestamp_RIOFPGA_capture);
        //       SwerveSubsystem.swerveDrive.addVisionMeasurement(
        //           usefulPose, result.timestamp_RIOFPGA_capture);
        //     } else {
        //       outofAreaReading += 1;
        //     }
        //   }
        // }
    }

    public void simulationPeriodic() {
        turret.simIterate();
    }

    public Command sysId() {
        return turret.sysId(Volts.of(4.0), Volts.of(8.0).per(Second), Second.of(30));
    }

    public Command set(double dutyCycle) {
        return turret.set(dutyCycle);
    }

    public Command setAngle(Angle angle) {
        return turret
                .setAngle(angle)
                .until(turret.isNear(angle, TurretConstants.PivotConstants.tolerance));
    }

    public Command setAngleDynamic(Supplier<Angle> angle) {
        return turret.setAngle(angle);
    }

    public Angle getRawAngle() {
        return turret.getAngle();
    }

    public Angle getRobotAdjustedAngle() {
        return turret.getAngle().plus(Degrees.of(180));
    }

    public double getRobotRelativeYawRadians() {
        return getRawAngle().in(edu.wpi.first.units.Units.Radians);
    }

    /**
     * Forces a CRT reseed attempt
     */
    public void rerunCrtSeed() {
        rotorSeededFromAbs = false;
        SmartDashboard.putNumber("Turret/CRT/ManualRerunTimestampSec", Timer.getFPGATimestamp());
        attemptRotorSeedFromCANCoders();
    }

    /**
     * Tries to solve turret position via CRT and seed the relative encoder with the result. Reads
     * both CANCoder values, runs the solver, updates the SmartMotorController encoder, and publishes
     * CRT status to the dashboard.
     */
    private void attemptRotorSeedFromCANCoders() {
        AbsSensorRead absRead = readAbsSensors();
        if (!absRead.ok()) {
            if (!"NO_DEVICES".equals(absRead.status())) {
                SmartDashboard.putString("Turret/CRT/SeedStatus", absRead.status());
            }
            lastSeedStatus = absRead.status();
            return;
        }

        double absA = absRead.absA();
        double absB = absRead.absB();
        lastAbsA = absA;
        lastAbsB = absB;

        var solver = new EasyCRT(easyCRTConfig);
        var solvedAngle = solver.getAngleOptional();

        SmartDashboard.putNumber("Turret/CRT/AbsA", absA);
        SmartDashboard.putNumber("Turret/CRT/AbsB", absB);
        SmartDashboard.putString("Turret/CRT/SolverStatus", solver.getLastStatus() + "");
        SmartDashboard.putNumber("Turret/CRT/SolverErrorRot", solver.getLastErrorRotations());
        SmartDashboard.putNumber("Turret/CRT/SolverIterations", solver.getLastIterations());

        if (solvedAngle.isEmpty()) {
            SmartDashboard.putBoolean("Turret/CRT/SolutionFound", false);
            lastSeedStatus = solver.getLastStatus() + "";
            return;
        }

        Angle turretRotations = solvedAngle.get();
        motor.setEncoderPosition(turretRotations);
        rotorSeededFromAbs = true;
        lastSeededTurretDeg = turretRotations.in(Degrees);
        lastSeedError = solver.getLastErrorRotations();
        SmartDashboard.putBoolean("Turret/CRT/SolutionFound", true);
        SmartDashboard.putNumber("Turret/CRT/SeededTurretDeg", lastSeededTurretDeg);
        SmartDashboard.putNumber("Turret/CRT/MatchErrorRot", lastSeedError);

        lastSeedStatus = "OK";
        SmartDashboard.putString("Turret/CRT/SeedStatus", lastSeedStatus);
        SmartDashboard.putBoolean("Turret/CRT/Seeded", rotorSeededFromAbs);
    }

    /**
     * Reads both absolute encoders and returns their rotations plus a status.
     */
    private AbsSensorRead readAbsSensors() {
        // Double absPositionASignal = (cancoderA.get());
        // Double absPositionBSignal = cancoderB.getPosition();

        boolean haveDevices = cancoderA != null && cancoderB != null;

        if (haveDevices) {

            return new AbsSensorRead(true, absPositionASignal, absPositionBSignal, "ok");
        }
        return new AbsSensorRead(false, Double.NaN, Double.NaN, "NO_DEVICES");
    }

    /**
     * Build the CRT config
     */
    private EasyCRTConfig buildEasyCrtConfig() {
        // if (cancoderA.isConnected() && cancoderB.getPosition() > 0) {
        return new EasyCRTConfig(
                () -> Rotations.of(getAbsoluteEncoderWithOffset()),
                () -> Rotations.of(cancoderB.getPosition()))
                .withCommonDriveGear(1, 160, 19, 21)
                .withAbsoluteEncoderOffsets(Rotations.of(0), Rotations.of(-0.650758)) //We gotta find this
                .withAbsoluteEncoderInversions(false, false)
                .withMechanismRange(PivotConstants.softLimitMin, PivotConstants.softLimitMax)
                .withMatchTolerance(Rotations.of(0.05))
                .withCrtGearRecommendationConstraints(1.2, 15, 60, 40);
        // } else {
        //   return null;
        // }
    }

    /**
     * Publish CRT config-derived values for debugging coverage/ratios.
     */
    private void logCrtConfigTelemetry() {
        double mechanismRangeRot = easyCRTConfig.getMechanismRange().in(Rotations);
        double uniqueCoverageRot =
                easyCRTConfig.getUniqueCoverage().map(angle -> angle.in(Rotations)).orElse(Double.NaN);
        SmartDashboard.putNumber(
                "Turret/CRT/Config/RatioA", easyCRTConfig.getEncoder1RotationsPerMechanismRotation());
        SmartDashboard.putNumber(
                "Turret/CRT/Config/RatioB", easyCRTConfig.getEncoder2RotationsPerMechanismRotation());
        SmartDashboard.putNumber("Turret/CRT/Config/UniqueCoverageRot", uniqueCoverageRot);
        SmartDashboard.putBoolean(
                "Turret/CRT/Config/CoverageSatisfiesRange", easyCRTConfig.coverageSatisfiesRange());
        SmartDashboard.putNumber("Turret/CRT/Config/RequiredRangeRot", mechanismRangeRot);

        var configPair = easyCRTConfig.getRecommendedCrtGearPair();
        SmartDashboard.putBoolean("Turret/CRT/Config/RecommendedPairFound", configPair.isPresent());
        if (configPair.isPresent()) {
            var pair = configPair.get();
            SmartDashboard.putNumber("Turret/CRT/Config/Reccomender/RecommendedGearA", pair.gearA());
            SmartDashboard.putNumber("Turret/CRT/Config/Reccomender/RecommendedGearB", pair.gearB());
            SmartDashboard.putNumber(
                    "Turret/CRT/Config/Reccomender/RecommendedCoverageRot", pair.coverage().in(Rotations));
            SmartDashboard.putNumber("Turret/CRT/Config/Reccomender/RecommendedLcm", pair.lcm());
            SmartDashboard.putBoolean(
                    "Turret/CRT/Config/Reccomender/RecommendedCoprime",
                    EasyCRTConfig.isCoprime(pair.gearA(), pair.gearB()));
            SmartDashboard.putNumber(
                    "Turret/CRT/Config/Reccomender/RecommendedIterations", pair.theoreticalIterations());
        }
    }

    private Double getAbsoluteEncoderWithOffset() {

        return MathUtil.inputModulus(cancoderA.get() - TurretConstants.PivotConstants.EncoderAOffset, 0, 1);
    }

    private static record AbsSensorRead(boolean ok, double absA, double absB, String status) {
    }

    public void setAngleSetpoint(Angle measure) {
        turret.setMechanismPositionSetpoint(measure);
    }
}

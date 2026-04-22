package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.MutMomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

import static edu.wpi.first.units.Units.*;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double maxSpeed = 4.5; // meters per second
  public static boolean disableHAL = false;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double DEADBAND = 0.05;
  }

  public static class CanIDConstants {

    public static final int CanIDclimber = 11;

    public static final int intakeRollerID = 20;
    public static final int turretID = 30;
    public static final int turretFlywheelID = 36;
    public static final int turretFlywheelFollowerID = 37;
    
    public static final int hoodID = 31;
    public static final int agitatorID = 32;
    //public static final int indexerID = 33;
    public static final int indexerflywheelID = 34;
    public static final int kickerflywheelID = 35;
    

    public static final int intakeArmID = 40;
    public static final int intakeArmFollowerID = 41;
  }

  public static class AgitatorConstants {
    
    public static double AgitatorSpeed = 0.55;
    public static double AgitatorInverseSpeed = -0.4;

  }

  public static class IndexerConstants {

    public static final  MechanismGearing gearingIndexer = new MechanismGearing(GearBox.fromStages("3:1"));
    public static final  MechanismGearing gearingKicker = new MechanismGearing(GearBox.fromStages("3:1"));

    public static double indexerVoltage = 4;
    public static double indexerVoltageOut = -4;
    public static double kickerVoltage = 4;
    public static double kickerVoltageReverse = -4;

    public static double indexerRPM = 1817;
    public static double indexerInverse = -0.1;

  }

  public static class GroundConstants {
  // public static final double kP = 0; // radians to rotations
  // public static final double kI = 0;
  // public static final double kD = 0;

  // public static final double ksimP = 2;
  // public static final double ksimI = 0;
  // public static final double ksimD = 0;


  // public static final double kS = 0;
  // public static final double kG = 0 ;
  // public static final double kV = 0 ;

  // public static final double ksimS = 0;
  // public static final double ksimG = 0;
  // public static final double ksimV = 0;


  public static final MechanismGearing gearing = new MechanismGearing(GearBox.fromReductionStages(9,5));

  public static final Mass weight = Pounds.of(11);
  public static final Distance length = Inches.of(19.25);

  public static final Angle softLowerLimit = Degrees.of(-600);
  public static final Angle softUpperLimit = Degrees.of(768);
  public static final Angle hardLowerLimit = Degrees.of(-3);
  public static final Angle hardUpperLimit = Degrees.of(70);

  //public static final Angle kHorizontalZero = Degrees.of(0);// Parallel to the ground at 15deg - setting position of absolute
  //public static final Angle kArmAllowableError = Degrees.of(RobotBase.isSimulation() ? 0.01 : 4);
  public static final Angle  tolerationAngle = Degrees.of(5);

  //public static final Angle kStartingPose = Degrees.of(145);

}

  public static class IntakeRollerConstants {

    public static double IntakeRollerOuttakeSpeeds = 0.8;
    public static double IntakeRollerIntakeSpeeds = -0.8;
    //public static double IntakeRollerHoldSpeed = 0.3;

    public static double kWristMomentOfInertia = 0.5;
    public static double kWristGearRatio = 3.0;
  }

  public static class TurretConstants {

    public static final Translation3d turretPivotCenterFromCameraCenter = new Translation3d(Inches.of(0), Inches.of(0), Inches.of(0));
    public static double wheelDiameter = 0;
    public static Pose3d cameraOffsetFromRobotCenter =
        new Pose3d( new Translation3d(Inches.of(0), Inches.of(0), Inches.of(0)), new Rotation3d());
    public static Transform3d turretPivotCenterFromRobotCenter =
        new Transform3d(new Translation3d(Inches.of(0), Inches.of(0), Inches.of(0)), new Rotation3d());

    public static double flywheelTolerance = 3000.0;
    public static double FARShooterGolRPM = 3800.0;
    public static class PivotConstants {

      public static double kP = 5;
      public static double kD = 0;
      public static double kI = 0;

      public static double kPSim = 0.4;
      public static double kISim = 0;
      public static double kDSim = 0;

      public static final double kS = 0;
      public static final double kV = 0;
      public static final double kA = 0;
      public static final double kG = 0;

      public static final double kSSim = 0;
      public static final double kVSim = 0;
      public static final double kASim = 0;
      public static final double kGSim = 0;
      public static final Angle tolerance = Degrees.of(2);
      
      // public static Angle softLimitMin = Degrees.of(-105);
      // public static Angle softLimitMax = Degrees.of(105);//change it back!
      public static Angle softLimitMin = Degrees.of(-70);//for testing
      public static Angle softLimitMax = Degrees.of(70);
      public static double EncoderAOffset = (0);
    }
    
  }

  public static class KickerConstants{
      public static double kickerVoltage = 5;
      public static double kickerVoltageReverse = -5;

      public static double kickerRPM = 1792;
  }

  public static class ClimberConstants {
    public static final double kP = 8;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kPSim = 8;
    public static final double kISim = 0;
    public static final double kDSim = 0;

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kG = 0;

    public static final double kSSim = 0;
    public static final double kVSim = 0;
    public static final double kASim = 0;
    public static final double kGSim = 0;

    public static final Mass mass = Pounds.of(15);
    public static final Distance hardLimitMin = Meters.of(0);
    public static final Distance hardLimitMax = Meters.of(3);
  }

  public static class HoodConstants{

    public static final Angle hardLimitMin = Degrees.of(-1);
    public static final Angle hardLimitMax = Degrees.of(21000);
    public static final Distance length = Inches.of(7);
    public static final Angle softLimitMin = Degrees.of(-1);
    public static final Angle softLimitMax = Degrees.of(21000);
    public static final MomentOfInertia MOIInKilogram =
            KilogramSquareMeters.of(Pounds.of(306.068).in(Kilograms) * Inches.of(1).in(Meters) * Inches.of(1).in(Meters));
    //.per(Inches).per(Inches).in(KilogramSquareMeters);

  }

  
}

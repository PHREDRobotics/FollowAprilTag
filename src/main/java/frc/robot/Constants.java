package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;

import com.revrobotics.CANSparkBase.IdleMode;

/**
 * A class to keep all of the robot's constants
 */
public final class Constants {
  public static final double k2pi = Math.PI * 2;

  /**
   * General constants for the robot's build
   */
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kDriveWheelFreeRps = NeoMotorConstants.kFreeSpeedRpm / 60;

    // public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio;
    public static final int kTurnMotorEncoderTicksPerRotation = 42;
    public static final double kTurningMotorRotationPerSteerRotation = 150 / 7;
    public static final double kTurningEncoderRot2Rad = kTurningMotorRotationPerSteerRotation
        * kTurnMotorEncoderTicksPerRotation;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    public static final double kTurningMotorPositionFactor = k2pi; // Radians
    public static final double kTurningEncoderPositionPIDMinInput = 0; // Radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningMotorPositionFactor; // Radians

    public static final double kDrivingP = 0.4;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeRps;
    public static final double kDrivingMinInput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kPTurning = 1;
    public static final double kITurning = 0;
    public static final double kDTurning = 0;
    public static final double kFFTurning = 0;
    public static final double kTurningMinInput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps

  }

  /**
   * Constants for driving
   */
  public static final class DriveConstants {
    // Distance between right and left wheels in inches
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between front and back wheels in inches
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    public static final int kBackLeftDriveMotorPort = 21;
    public static final int kFrontLeftDriveMotorPort = 11;
    public static final int kFrontRightDriveMotorPort = 16;
    public static final int kBackRightDriveMotorPort = 26;

    public static final int kBackLeftTurningMotorPort = 22;
    public static final int kFrontLeftTurningMotorPort = 12;
    public static final int kFrontRightTurningMotorPort = 17;
    public static final int kBackRightTurningMotorPort = 27;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kBackRightDriveEncoderReversed = true;

    public static final boolean kFrontLeftDriveInverted = false;
    public static final boolean kBackLeftDriveInverted = true;
    public static final boolean kFrontRightDriveInverted = true;
    public static final boolean kBackRightDriveInverted = false;

    // public static final int kFrontLeftDriveAbsoluteEncoderPort = 12;
    // public static final int kBackLeftDriveAbsoluteEncoderPort = 22;
    // public static final int kFrontRightDriveAbsoluteEncoderPort = 17;
    // public static final int kBackRightDriveAbsoluteEncoderPort = 27;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kFrontLeftModuleChassisAngularOffset = 0;
    public static final double kBackLeftModuleChassisAngularOffset = 0;
    public static final double kFrontRightModuleChassisAngularOffset = 0;
    public static final double kBackRightModuleChassisAngularOffset = 0;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * k2pi;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
        kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double kTeleDriveThrottleMultiplier = 1;
    public static final double kThrottleMin = 0.75;
    public static final double kThrottleMax = 5;

  }

  /**
   * Constants for the arm
   */
  public static final class ArmConstants {
    public static final int kArmControllerPort = 35;
    public static final int kLimitSwitchControllerPort = 9;
    public static final double kArmMotorGearRatio = 40;

    // Change later after arm built -------------------------------------
    // need to be remeasured so that 0 is horizontal and in radians.
    public static final double kArmPickup = -0.40;
    public static final double kArmAmp = 1.55 + 0.3;
    public static final double kArmShooter = 3.411;
    // radii needed to make the limit switch horizontal.
    public static double kArmOffsetRads = 3.511;

    public static final double kVoltageMultiplier = 1.5;
    // feedforward constants
    public static double kSVolts = 0;
    public static double kGVolts = 0.1;
    public static double kAVoltSecondSquaredPerRad = 0.1;
    public static double kVVoltSecondPerRad = 1.5;

    public static double kMaxVelocityRadPerSecond = 3;
    public static double kMaxAccelerationRadPerSecSquared = 10;

    public static double kP = 1.5;
    public static double kI = 0.002;
    public static double kD = 0.002;
  }

  /**
   * Constants for the lift
   */
  public static final class LiftConstants {
    public static final int kLeftLiftControllerPort = 46;
    public static final int kRightLiftControllerPort = 47;

    public static final double kExtendSpeed = 0.8;
    public static final double kRetractSpeed = 0.65;

  }

  public static final class ShooterConstants {
    public static final int kLeftShooterControllerPort = 41;
    public static final int kRightShooterControllerPort = 42;
    public static final int kXBtn = Button.kX.value;
    public static final double kShooterSpeed = 1;
    public static final double kShooterTime = 1.5;

  }

  /**
   * Constants for autonomous
   */
  public static final class AutoConstants {

    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = //
        DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared);

    public static final double kAutoSpeedMetersPerSecond = kMaxSpeedMetersPerSecond - .25;

    public static final TrapezoidProfile.Constraints kXControllerConstraints = new TrapezoidProfile.Constraints(kAutoSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    public static final TrapezoidProfile.Constraints kYControllerConstraints = new TrapezoidProfile.Constraints(kAutoSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);

    // 1
    public static TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        AutoConstants.kAutoSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    /**
     * Must be greater than 1.
     */
    public static final double kAutoDriveSlowDownFactor = 2;
  }

  /**
   * Constants for the controller
   */
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;

    public static final int kDriverYAxis = Axis.kLeftY.value;
    public static final int kDriverXAxis = Axis.kLeftX.value;
    public static final int kDriverRotAxis = Axis.kRightX.value;
    public static final int kDriverFieldOrientedButtonIdx = Button.kB.value;
    public static final int kLeftTriggerAxis = Axis.kLeftTrigger.value;
    public static final int kRightTriggerAxis = Axis.kRightTrigger.value;

    public static final int kStartButton = Button.kStart.value;
    public static final int kLeftBumper = Button.kLeftBumper.value;
    public static final int kRightBumper = Button.kRightBumper.value;
    public static final int kXButton = Button.kX.value;
    public static final int kYButton = Button.kY.value;
    public static final int kAButton = Button.kA.value;
    public static final int kBButton = Button.kB.value;

    public static final double kDeadband = 0.15;
    public static final double kHighDeadband = 0.25;

  }

  // public static final class TestConstants {
  // // public static final int kTestMotorCanId = 52;
  // // public static final int kTestMotorCanIdTwo = 51;
  // }

  /**
   * Constants for the neo motors
   */
  public final static class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class GrabberConstants {
    public static final int kABtn = Button.kA.value;
    // public static final int kBBtn = Button.kB.value;
    public static final int kYBtn = Button.kY.value;
    public static final double kOuttakeTime = 1.0;
  }

  public static final class VisionConstants {
    public static final double kLimelightMountAngleDegrees = 33.0;
    public static final double kLimelightLensHeightInches = 32.0;
    public static final double kAmpOrSourceHeightInches = 48.5;
    public static final double kSpeakerHeightInches = 51.0 + 7.0 / 8.0;
    public static final double kStageHeightInches = 47.5;
    public static final double kAreaToCentimeters = 150;
  }

  public static final class IntakeConstants {
    public static final int kLeftIntakeControllerPort = 31;
    public static final int kRightIntakeControllerPort = 32;

    public static final double kIntakeSpeed = 0.25;
    public static final double kOuttakeSpeed = 0.6;
  }

}
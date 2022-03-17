// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double kDt = 0.02;

  public static final class ShooterConstants {
    public static final int kShooterMotorLeaderPort = 1;
    public static final int kShooterMotorFollowerPort = 2;
    public static final double kShooterFenderRPM = 1500;
    public static final double kShooterRPMThreshold = 75;
    public static final double kPower = 0.6;
    public static final double kS = 0.39592;
    public static final double kV = 0.15034;
    public static final double kA = 0.0099325;
    public static final double kP = 0.0001;

    public static final double kRampUpTimeoutSeconds = 2.5;
  }

  public static final class ControllerConstants {
    public static final int kNavigatorPort = 0;
    public static final int kOperatorPort = 1;
  }

  public static final class IntakeConstants {
    public static final int kIntakeSolenoidPorts[] = {1, 0};
    public static final int kIntakeMotorPort = 10;
    public static final int kIntakeVoltage = 6;
  }

  public static final class IndexerConstants {
    public static final int kProximitySensorPort = 0;
    public static final int kBallElevatorPort = 20;
    public static final int kIndexerPort = 21;

    public static final double kPassivePower = 3;
    public static final double kFeedingPower = 5.5;
    public static final double kIndexerPower = 7;
    public static final int kIndexerShootingTimeout = 4;
  }

  public static final class DrivetrainConstants {
    public static final int kLeftSRXDrivePort = 1;
    public static final int kLeftSPXDrivePort = 2;
    public static final int kRightSRXDrivePort = 3;
    public static final int kRightSPXDrivePort = 4;

    public static final double kDriveForwardMultiplier = 1;
    public static final double kDriveTurnMultiplier = 1;

    public static final double kLinearKS = 1.5819;
    public static final double kLinearKV = 2.9238;
    public static final double kLinearKA = 1.6274;
    public static final double kAngularKS = 1.9351;
    public static final double kAngularKV = 3.4356;
    public static final double kAngularKA = 1.2996;

    public static final double kP = 1.5551;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kWheelDiameterInches = 6;
    public static final double kWheelCircumferenceMeters = Units.inchesToMeters(6 * Math.PI);
    public static final double kTrackwidth = 0.57591;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidth);
    public static final double kCimToWheelGearing = 10.71;

    public static final double kAngularKP = .1;
    public static final double kAngularKD = .0;

    public static final double kMaxTurnRate = 100;
    public static final double kMaxTurnAccel = 10;
    public static final double kDegreeTolerance = 0.75;
    public static final double kTurnRateTolerance = 1.5;

    public static final double kAutoMaxSpeed = 1;
    public static final double kAutoMaxAccel = 2;

    public static final double kAutoTime = 2.5;
  }

  public static final class PneumaticClimbConstants {
    public static final int kClimbSolenoidPorts[] = {3, 2};
  }

  public static final class ClimbConstants {
    public static final int kLeftClimberPort = 40;
    public static final int kRightClimberPort = 41;

    public static final double kClimbPower = 0.4;
  }

  public static final class HoodConstants {
    public static final int kHoodSRXPort = 51;

    public static final double kMaxAngleDegrees = 30.0;
    public static final double kMinAngleDegrees = 0.0;

    public static final double kHoodGearing = 2 * (470 / 22);
    public static final double kHoodCMLengthInches = 2.5;
    public static final double kHoodMassPounds = 3.5;

    public static final double kS = 0.50434;
    public static final double kV = 11.671;
    public static final double kA = 1.4378;
    public static final double kCos = 0.0;
    public static final double kP = 75;
    public static final double kD = 10;

    public static final double kMaxDegreeError = 0.5;

    public static final double kOpenLoopPercentPower = 0.15;

    // 30 degrees of reasonable motion ideally covered in half a second
    public static final double kMaxVelocity = Units.degreesToRadians(30.0 / 0.5);
    // Acceleration should get to max velocity in half the time it takes to move all the way across
    // (mainly a guess number)
    public static final double kMaxAccel = kMaxVelocity / 0.25;
  }

  public static final class LimelightConstants {
    public static final double kLensHeightMeters = Units.inchesToMeters(27.299450);
    public static final double kMountAngleDegrees = 50.000000;
    public static final double kTargetHeightMeters = Units.inchesToMeters(104.0 - 1.0);

    public static final double kLimelightP = 0.45;
    public static final double kLimelightD = 0.0;

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kShooterRPMMap =
        new InterpolatingTreeMap<>();

    static {
      kShooterRPMMap.put(new InterpolatingDouble(0.97), new InterpolatingDouble(1450.0));
      kShooterRPMMap.put(new InterpolatingDouble(1.72), new InterpolatingDouble(1600.0));
      kShooterRPMMap.put(new InterpolatingDouble(2.33), new InterpolatingDouble(1700.0));
      kShooterRPMMap.put(new InterpolatingDouble(3.05), new InterpolatingDouble(1875.0));
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodMap =
        new InterpolatingTreeMap<>();

    static {
      kHoodMap.put(new InterpolatingDouble(0.97), new InterpolatingDouble(8.0));
      kHoodMap.put(new InterpolatingDouble(1.72), new InterpolatingDouble(20.0));
      kHoodMap.put(new InterpolatingDouble(2.33), new InterpolatingDouble(23.5));
      kHoodMap.put(new InterpolatingDouble(3.05), new InterpolatingDouble(28.5));
    }
  }
}

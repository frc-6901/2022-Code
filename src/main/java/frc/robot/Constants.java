// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class ShooterConstants {
    public static final int kShooterMotorLeaderPort = 1;
    public static final int kShooterMotorFollowerPort = 2;
    public static final double kShooterFenderRPM = 1700;
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

    public static final double kDriveForwardMultiplier = 0.8;
    public static final double kDriveTurnMultiplier = 0.8;

    public static final double kLinearKS = 0.6;
    public static final double kLinearKV = 3.2105;
    public static final double kLinearKA = 0.33214;
    public static final double kAngularKS = 1.3526;
    public static final double kAngularKV = 3.4422;
    public static final double kAngularKA = 0.34065;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kWheelDiameterInches = 6;
    public static final double kWheelCircumferenceMeters = Units.inchesToMeters(6 * Math.PI);
    public static final double kTrackwidth = 0.56589;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidth);
    public static final double kCimToWheelGearing = 10.71;

    public static final double kAutoMaxSpeed = 1.5;
    public static final double kAutoMaxAccel = 0.5;
  }

  public static final class PneumaticClimbConstants {
    public static final int kClimbSolenoidPorts[] = {3, 2};
  }

  public static final class ClimbConstants {
    public static final int kLeftClimberPort = 40;
    public static final int kRightClimberPort = 41;

    public static final double kClimbPower = 0.4;
  }
}

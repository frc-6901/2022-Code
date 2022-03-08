// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int kLeftShooterMotorPort = 1;
    public static final int kRightShooterMotorPort = 2;
    public static final int kRPMThreshold = 50;
    public static final double kPower = 0.6;
    public static final double kS = 0.23638;
    public static final double kV = 0.1292;
    public static final double kA = 0.0094019;
    public static final double kP = 0.0001;
  }

  public static final class ControllerConstants {
    public static final int kNavigatorPort = 0;
    public static final int kOperatorPort = 1;
  }
}

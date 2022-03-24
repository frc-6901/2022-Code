// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.util.InterpolatingDouble;

public class LimelightManager extends SubsystemBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry lightState = table.getEntry("ledMode");
  NetworkTableEntry cameraState = table.getEntry("camMode");

  double m_targetVerticalAngle;
  double m_targetHorizontalError;

  /** Creates a new LimelightManager. */
  public LimelightManager() {}

  public void turnOnLED() {
    lightState.setNumber(3);
  }

  public void turnOffLED() {
    lightState.setNumber(1);
  }

  /**
   * Get's distance estimate based off of target y position.
   *
   * <p>(From LL Website: https://docs.limelightvision.io/en/latest/cs_estimating_distance.html) d =
   * (h_target - h_camera) / tan(mount_angle + targetAngle)
   *
   * @return Distance from target
   */
  public double getDistance() {
    double heightDifference =
        LimelightConstants.kTargetHeightMeters - LimelightConstants.kLensHeightMeters;
    double totalAngleRadians =
        Units.degreesToRadians(LimelightConstants.kMountAngleDegrees + m_targetVerticalAngle);
    return heightDifference / Math.tan(totalAngleRadians);
  }

  public double getHorizontalErrorDegrees() {
    return m_targetHorizontalError;
  }

  public double getShooterTargetRPM() {
    return LimelightConstants.kShooterRPMMap.getInterpolated(new InterpolatingDouble(getDistance()))
        .value;
  }

  public double getHoodTargetPos() {
    return LimelightConstants.kHoodMap.getInterpolated(new InterpolatingDouble(getDistance()))
        .value;
  }

  public void setVerticalAngleDegrees(double degrees) {
    // No-op if this isn't sim (testing purposes only)
    if (RobotBase.isSimulation()) {
      m_targetVerticalAngle = degrees;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_targetVerticalAngle = ty.getDouble(0.0);
    m_targetHorizontalError = tx.getDouble(0.0);
    SmartDashboard.putNumber("LL DIst", getDistance());
  }
}

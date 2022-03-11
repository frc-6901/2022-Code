// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util;

/** Utility class for conversions between magnetic encoder counts and real world units. */
public class MagEncoderUtil {
  /**
   * A constant denoting the number of 100ms sensor periods per second: 1 period / 100 ms * 1000 ms
   * / 1 s = 10 periods / s
   */
  public static final double k100msPerSecond = 10;

  /** The number of sensor counts per revolution */
  public static final int kCountsPerRev = 4096;

  /**
   * Converts a mag encoder native unit measurement to a real world distance value (e.g. rotations,
   * meters, radians)
   *
   * <p>Uses the following conversion: measurement * (rotation / countsPerRev) * unitsPerRotation /
   * gearing
   *
   * @param sensorCounts the measured sensor counts
   * @param unitsPerRotation the units per rotation of the shaft (a wheel with radius 3in would have
   *     a units per rotation of 2 * pi * 3 inches)
   * @param gearing the gearing between the shaft and encoder
   * @return the distance travelled by the encoder in the units specified by the units per rotation
   */
  public static double nativeUnitsToDistance(
      double sensorCounts, double unitsPerRotation, double gearing) {
    double motorRotations = (double) sensorCounts / kCountsPerRev;
    double position = motorRotations * unitsPerRotation / gearing;
    return position;
  }

  /**
   * Converts a mag encoder native unit measurement to a real world distance value (e.g. rotations,
   * meters, radians) and assumes a gearing of one.
   *
   * @param sensorCounts
   * @param unitsPerRotation
   * @return the distance travelled by the encoder in the units specified by the units per rotation
   */
  public static double nativeUnitsToDistance(double sensorCounts, double unitsPerRotation) {
    return nativeUnitsToDistance(sensorCounts, unitsPerRotation, 1);
  }

  /**
   * Converts a distance travelled into native sensor units.
   *
   * <p>Uses the following conversion: position * (rotation / (unitsPerRotation/ gearing)) *
   * (countsPerRotation / rotation)
   *
   * @param position the measured position
   * @param unitsPerRotation the units per rotation of the shaft (a wheel with radius 3in would have
   *     a units per rotation of 2 * pi * 3 inches)
   * @param gearing the gearing between the shaft and encoder
   * @return the encoder counts that resulted from the measured distance
   */
  public static int distanceToNativeUnits(
      double position, double unitsPerRotation, double gearing) {
    double shaftRotations = position / unitsPerRotation * gearing;
    int sensorCounts = (int) (shaftRotations * kCountsPerRev);
    return sensorCounts;
  }

  /**
   * Converts a distance travelled into native sensor units assuming gearing is 1.
   *
   * @param position the measured position
   * @param unitsPerRotation the units per rotation of the shaft (a wheel with radius 3in would have
   *     a units per rotation of 2 * pi * 3 inches)
   * @return the encoder counts that resulted from the measured distance
   */
  public static int distanceToNativeUnits(double position, double unitsPerRotation) {
    return distanceToNativeUnits(position, unitsPerRotation, 1);
  }

  /**
   * Converts a recorded velocity into native units
   *
   * <p>Uses the following conversion: velocity * (rotation / (unitsPerRotation/ gearing)) *
   * (countsPerRotation / rotation) * (seconds / 100msPerSecond)
   *
   * @param velocityPerSecond the recorded velocity
   * @param unitsPerRotation the units per rotation of the shaft (a wheel with radius 3in would have
   *     a units per rotation of 2 * pi * 3 inches)
   * @param gearing the gearing between the shaft and encoder
   * @return the corresponding sensor velocity that the encoder should record
   */
  public static int velocityToNativeUnits(
      double velocityPerSecond, double unitsPerRotation, double gearing) {
    int encoderCountsConversion =
        distanceToNativeUnits(velocityPerSecond, unitsPerRotation, gearing);
    int sensorCountsPer100ms = encoderCountsConversion / ((int) k100msPerSecond);
    return sensorCountsPer100ms;
  }

  /**
   * Converts a recorded velocity into native units assuming gearing is 1.
   *
   * @param velocityPerSecond the recorded velocity
   * @param unitsPerRotation the units per rotation of the shaft (a wheel with radius 3in would have
   *     a units per rotation of 2 * pi * 3 inches)
   * @return the corresponding sensor velocity that the encoder should record
   */
  public static int velocityToNativeUnits(double velocityPerSecond, double unitsPerRotation) {

    return velocityToNativeUnits(velocityPerSecond, unitsPerRotation, 1);
  }

  /**
   * Converts a recorded native sensor units rate to a velocity (meters per second, radians per
   * second).
   *
   * <p>Uses the following conversion: sensorRate * (rotation / countsPerRev) * (unitsPerRotation /
   * gearing) * (100msPerSecond / second)
   *
   * @param sensorVelocity the recorded sensor rate
   * @param unitsPerRotation the units per rotation of the shaft (a wheel with radius 3in would have
   *     a units per rotation of 2 * pi * 3 inches)
   * @param gearing the gearing between the shaft and encoder
   * @return the velocity per second in the distance units specified in the unitsPerRotation (e.g.
   *     specifying units per rotation in meters leads to a velocity measurement of meters/second)
   */
  public static double nativeUnitsToVelocity(
      double sensorVelocity, double unitsPerRotation, double gearing) {
    double distanceConversion = nativeUnitsToDistance(sensorVelocity, unitsPerRotation, gearing);
    double velocity = distanceConversion * k100msPerSecond;
    return velocity;
  }

  /**
   * Converts a recorded native sensor units rate to a velocity (meters per second, radians per
   * second).
   *
   * @param sensorVelocity the recorded sensor rate
   * @param unitsPerRotation the units per rotation of the shaft (a wheel with radius 3in would have
   *     a units per rotation of 2 * pi * 3 inches)
   * @return the velocity per second in the distance units specified in the unitsPerRotation
   */
  public static double nativeUnitsToVelocity(double sensorVelocity, double unitsPerRotation) {
    return nativeUnitsToVelocity(sensorVelocity, unitsPerRotation, 1);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util;

import org.junit.Assert;
import org.junit.Test;

/** Class that tests the system test */
public class MagEncoderUtilTest {
  private static final double k6InchWheelMetersPerRotation = 0.47877871986;

  @Test
  public void nativeToDistanceTest() {
    // assert statements
    Assert.assertEquals(1, MagEncoderUtil.nativeUnitsToDistance(4096, 1), .001);
    Assert.assertEquals(.1, MagEncoderUtil.nativeUnitsToDistance(4096, 1, 10), .001);
    Assert.assertEquals(
        0.478793556,
        MagEncoderUtil.nativeUnitsToDistance(4096, k6InchWheelMetersPerRotation),
        .001);
  }

  @Test
  public void distanceToNative() {
    Assert.assertEquals(4096, MagEncoderUtil.distanceToNativeUnits(1, 1));
    Assert.assertEquals(40960, MagEncoderUtil.distanceToNativeUnits(1, 1, 10), .001);
    Assert.assertEquals(
        4096,
        MagEncoderUtil.distanceToNativeUnits(0.478793556, k6InchWheelMetersPerRotation),
        .001);
  }

  @Test
  public void nativeUnitsToVelocity() {
    Assert.assertEquals(10, MagEncoderUtil.nativeUnitsToVelocity(4096, 1), .001);
    Assert.assertEquals(1, MagEncoderUtil.nativeUnitsToVelocity(4096, 1, 10), .001);
    Assert.assertEquals(
        4.78793556, MagEncoderUtil.nativeUnitsToVelocity(4096, k6InchWheelMetersPerRotation), .001);
  }

  @Test
  public void velocityToNativeUnits() {
    Assert.assertEquals(4096, MagEncoderUtil.velocityToNativeUnits(10, 1), .001);
    Assert.assertEquals(4096, MagEncoderUtil.velocityToNativeUnits(1, 1, 10), .001);
    Assert.assertEquals(
        4096, MagEncoderUtil.velocityToNativeUnits(4.78793556, k6InchWheelMetersPerRotation), .001);
  }
}

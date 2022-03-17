package frc.robot;

import static org.junit.Assert.*;

import frc.robot.subsystems.LimelightManager;
import org.junit.*;

/** Class that tests the limelight distance calculations */
public class LimelightDistanceCalcTest {
  @Test
  public void test15Degrees() {
    LimelightManager testManager = new LimelightManager();
    // tests 15 degree offset
    testManager.setVerticalAngleDegrees(15);
    Assert.assertEquals(0.896622948, testManager.getDistance(), 0.003);
  }
}

package frc.robot.driver.controlTask;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import frc.lib.helpers.Helpers;
import frc.lib.helpers.ImmutablePair;
import frc.robot.driver.controltasks.VisionShooterTurnAndAimRelativeTask;

public class VisionShooterTurnAndAimRelativeTaskTests
{
    // @Test
    public void TestRedOnTopOfOffsetTag()
    {
        ImmutablePair<Double, Double> pair = VisionShooterTurnAndAimRelativeTask.calculateOffsetAngle(true, 0.0, 0.0, 0.0);
        Assertions.assertEquals(24.0, pair.first, 0.001);
        Assertions.assertEquals(90.0, pair.second, 0.001);
    }

    // @Test
    public void TestRedAlignedWithOffsetTag()
    {
        ImmutablePair<Double, Double> pair = VisionShooterTurnAndAimRelativeTask.calculateOffsetAngle(true, 24.0, 0.0, 0.0);
        Assertions.assertEquals(Math.sqrt(24.0 * 24.0 * 2.0), pair.first, 0.001);
        Assertions.assertEquals(45.0, pair.second, 0.001);
    }

    // @Test
    public void TestRedAlignedWithOffsetTagAtAngleNoYawSquare()
    {
        ImmutablePair<Double, Double> pair = VisionShooterTurnAndAimRelativeTask.calculateOffsetAngle(true, 24.0, 24.0, 0.0);
        Assertions.assertEquals(Math.sqrt(24.0 * 24.0 + 48.0 * 48.0), pair.first, 0.001);
        Assertions.assertEquals(Helpers.atan2d(48.0, 24.0), pair.second, 0.001);
    }

    // @Test
    public void TestRedAlignedWithOffsetTagAtAngleNoYawRect()
    {
        ImmutablePair<Double, Double> pair = VisionShooterTurnAndAimRelativeTask.calculateOffsetAngle(true, 24.0, 12.0, 0.0);
        Assertions.assertEquals(Math.sqrt(24.0 * 24.0 + 36.0 * 36.0), pair.first, 0.001);
        Assertions.assertEquals(Helpers.atan2d(36.0, 24.0), pair.second, 0.001);
    }

    // @Test
    public void TestRedAlignedWithOffsetTagAtAngleWithPosYaw()
    {
        ImmutablePair<Double, Double> pair = VisionShooterTurnAndAimRelativeTask.calculateOffsetAngle(true, 24.0 * Helpers.cosd(15.0) - 12.0 * Helpers.sind(15.0), 24.0 * Helpers.sind(15.0) + 12.0 * Helpers.cosd(15.0), 15.0);
        Assertions.assertEquals(Math.sqrt(24.0 * 24.0 + 36.0 * 36.0), pair.first, 0.001);
        Assertions.assertEquals(Helpers.atan2d(36.0, 24.0) + 15.0, pair.second, 0.001);
    }

    // @Test
    public void TestRedAlignedWithOffsetTagAtAngleWithNegYaw()
    {
        ImmutablePair<Double, Double> pair = VisionShooterTurnAndAimRelativeTask.calculateOffsetAngle(true, 24.0 * Helpers.cosd(-15.0) - 12.0 * Helpers.sind(-15.0), 24.0 * Helpers.sind(-15.0) + 12.0 * Helpers.cosd(-15.0), -15.0);
        Assertions.assertEquals(Math.sqrt(24.0 * 24.0 + 36.0 * 36.0), pair.first, 0.001);
        Assertions.assertEquals(Helpers.atan2d(36.0, 24.0) - 15.0, pair.second, 0.001);
    }
    
    // @Test
    public void TestBlueOnTopOfOffsetTag()
    {
        ImmutablePair<Double, Double> pair = VisionShooterTurnAndAimRelativeTask.calculateOffsetAngle(false, 0.0, 0.0, 0.0);
        Assertions.assertEquals(24.0, pair.first, 0.001);
        Assertions.assertEquals(-90.0, pair.second, 0.001);
    }

    // @Test
    public void TestBlueAlignedWithOffsetTag()
    {
        ImmutablePair<Double, Double> pair = VisionShooterTurnAndAimRelativeTask.calculateOffsetAngle(false, 24.0, 0.0, 0.0);
        Assertions.assertEquals(Math.sqrt(24.0 * 24.0 * 2.0), pair.first, 0.001);
        Assertions.assertEquals(-45.0, pair.second, 0.001);
    }

    // @Test
    public void TestBlueAlignedWithOffsetTagAtAngleNoYawSquare()
    {
        ImmutablePair<Double, Double> pair = VisionShooterTurnAndAimRelativeTask.calculateOffsetAngle(false, 24.0, 24.0, 0.0);
        Assertions.assertEquals(24.0, pair.first, 0.001);
        Assertions.assertEquals(0.0, pair.second, 0.001);
    }

    // @Test
    public void TestBlueAlignedWithOffsetTagAtAngleNoYawRect()
    {
        ImmutablePair<Double, Double> pair = VisionShooterTurnAndAimRelativeTask.calculateOffsetAngle(false, 24.0, 12.0, 0.0);
        Assertions.assertEquals(Math.sqrt(24.0 * 24.0 + 12.0 * 12.0), pair.first, 0.001);
        Assertions.assertEquals(Helpers.atan2d(-12.0, 24.0), pair.second, 0.001);
    }

    // @Test
    public void TestBlueAlignedWithOffsetTagAtAngleWithPosYaw()
    {
        ImmutablePair<Double, Double> pair = VisionShooterTurnAndAimRelativeTask.calculateOffsetAngle(false, 24.0 * Helpers.cosd(15.0) - 12.0 * Helpers.sind(15.0), 24.0 * Helpers.sind(15.0) + 12.0 * Helpers.cosd(15.0), 15.0);
        Assertions.assertEquals(Math.sqrt(24.0 * 24.0 + 12.0 * 12.0), pair.first, 0.001);
        Assertions.assertEquals(Helpers.atan2d(-12.0, 24.0) + 15.0, pair.second, 0.001);
    }

    // @Test
    public void TestBlueAlignedWithOffsetTagAtAngleWithNegYaw()
    {
        ImmutablePair<Double, Double> pair = VisionShooterTurnAndAimRelativeTask.calculateOffsetAngle(false, 24.0 * Helpers.cosd(-15.0) - 12.0 * Helpers.sind(-15.0), 24.0 * Helpers.sind(-15.0) + 12.0 * Helpers.cosd(-15.0), -15.0);
        Assertions.assertEquals(Math.sqrt(24.0 * 24.0 + 12.0 * 12.0), pair.first, 0.001);
        Assertions.assertEquals(Helpers.atan2d(-12.0, 24.0) - 15.0, pair.second, 0.001);
    }
}

package frc.robot.mechanisms;

import frc.lib.helpers.Pair;
import frc.robot.TuningConstants;
import frc.robot.mechanisms.ArmKinematicsCalculator.ArmGraphNode;

import java.io.IOException;
import java.nio.charset.StandardCharsets;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import de.siegmar.fastcsv.writer.*;

public class ArmKinematicsCalculatorTests
{
    public static void main(String[] args)
    {
        // Generate CSV file containing valid positions for the arm given the IK/FK (and that they agree)
        try (CsvWriter validCsvWriter = CsvWriter.builder().build(java.nio.file.Path.of("possiblePositions.csv"), StandardCharsets.UTF_8))
        {
            validCsvWriter.writeRow("shoulderAngle", "wristAngle", "invalid", "issue", "x1", "z1", "x2", "z2", "x3", "z3", "x4", "z4");

            Pair<Double, Double> result = new Pair<Double, Double>(0.0, 0.0);
            ArmKinematicsCalculator calculator = new ArmKinematicsCalculator(TuningConstants.ARM_SHOULDER_MIN_POSITION, TuningConstants.ARM_WRIST_MIN_POSITION);
            for (double shoulderPosition = 27.8; shoulderPosition <= 28.3; shoulderPosition += 0.1)
            {
                for (double wristPosition = TuningConstants.ARM_WRIST_MIN_POSITION; wristPosition <= TuningConstants.ARM_WRIST_MAX_POSITION; wristPosition += 0.1)
                {
                    boolean invalid = calculator.calculateArmLimits(shoulderPosition, wristPosition, result);
                    validCsvWriter.writeRow(String.valueOf(shoulderPosition), String.valueOf(wristPosition), String.valueOf(invalid), calculator.getExtensionType().toString(), String.valueOf(calculator.getShooterBottomAbsPos().x), String.valueOf(calculator.getShooterBottomAbsPos().y), String.valueOf(calculator.getShooterTopAbsPos().x), String.valueOf(calculator.getShooterTopAbsPos().y), String.valueOf(calculator.getIntakeBottomAbsPos().x), String.valueOf(calculator.getIntakeBottomAbsPos().y), String.valueOf(calculator.getIntakeTopAbsPos().x), String.valueOf(calculator.getIntakeTopAbsPos().y));
                }
            }
        }
        catch (IOException e)
        {
            e.printStackTrace(System.err);
        }
    }

    @Test
    public void testThings()
    {
        ArmGraphNode closestNode = ArmKinematicsCalculator.getClosestArmNode(-8.92, 90.06);
        Assertions.assertEquals(closestNode.getShoulderAngle(), TuningConstants.ARM_SHOULDER_POSITION_TUCKED_TRANSIT);
        Assertions.assertEquals(closestNode.getWristAngle(), TuningConstants.ARM_WRIST_POSITION_TUCKED_TRANSIT);
    }

    @Test
    public void testUniversals()
    {
        // ensure that we find the universals when we are looking for the closest node that is not at/very near to another node
        ArmGraphNode closestNode = ArmKinematicsCalculator.getClosestArmNode(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL + 1.7, -45.0);
        Assertions.assertTrue(closestNode.isUniversal());
        Assertions.assertEquals(closestNode.getShoulderAngle(), TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL);

        closestNode = ArmKinematicsCalculator.getClosestArmNode(TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL - 1.7, 90.06);
        Assertions.assertTrue(closestNode.isUniversal());
        Assertions.assertEquals(closestNode.getShoulderAngle(), TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL);

        // ensure that we find the closest node around a universal when we are starting at a node
        closestNode = ArmKinematicsCalculator.getClosestArmNode(TuningConstants.ARM_SHOULDER_POSITION_INTAKE_FLIPPED - TuningConstants.ARM_SHOULDER_GOAL_THRESHOLD / 2.0, TuningConstants.ARM_WRIST_POSITION_INTAKE_FLIPPED + TuningConstants.ARM_WRIST_GOAL_THRESHOLD / 2.0);
        Assertions.assertFalse(closestNode.isUniversal());
        Assertions.assertEquals(closestNode.getShoulderAngle(), TuningConstants.ARM_SHOULDER_POSITION_INTAKE_FLIPPED);
        Assertions.assertEquals(closestNode.getWristAngle(), TuningConstants.ARM_WRIST_POSITION_INTAKE_FLIPPED);
    }
}

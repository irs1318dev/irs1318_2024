package frc.robot.mechanisms;

import frc.lib.helpers.Pair;
import frc.robot.TuningConstants;

import java.io.IOException;
import java.nio.charset.StandardCharsets;

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
            for (double shoulderPosition = TuningConstants.ARM_SHOULDER_MIN_POSITION; shoulderPosition <= TuningConstants.ARM_SHOULDER_MAX_POSITION; shoulderPosition += 1.0)
            {
                for (double wristPosition = TuningConstants.ARM_WRIST_MIN_POSITION; wristPosition <= TuningConstants.ARM_WRIST_MAX_POSITION; wristPosition += 1.0)
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
}

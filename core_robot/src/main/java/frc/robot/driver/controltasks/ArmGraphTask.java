package frc.robot.driver.controltasks;

import java.util.List;

import frc.lib.helpers.ExceptionHelpers;
import frc.lib.helpers.Helpers;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.mechanisms.ArmKinematicsCalculator;
import frc.robot.mechanisms.ArmKinematicsCalculator.ArmGraphNode;
import frc.robot.mechanisms.ArmMechanism;

public class ArmGraphTask extends ControlTaskBase
{
    private final double shoulderGoalPos;
    private final double wristGoalPos;

    private ArmMechanism arm;
    private List<ArmGraphNode> path;
    private int currPos;
    private boolean hasCompleted;

    public ArmGraphTask(double shoulderGoalPos, double wristGoalPos)
    {
        this.shoulderGoalPos = shoulderGoalPos;
        this.wristGoalPos = wristGoalPos;
    }

    @Override
    public void begin()
    {
        this.arm = this.getInjector().getInstance(ArmMechanism.class);

        ArmGraphNode goalArmGraphNode = ArmKinematicsCalculator.getClosestArmNode(this.shoulderGoalPos, this.wristGoalPos);
        ArmGraphNode startArmGraphNode = ArmKinematicsCalculator.getClosestArmNode(this.arm.getShoulderPosition(), this.arm.getWristPosition());

        System.out.println("Starting Node");
        System.out.println(startArmGraphNode.shoulderAngle);
        System.out.println(startArmGraphNode.wristAngle);

        System.out.println("Goal Node");
        System.out.println(goalArmGraphNode.shoulderAngle);
        System.out.println(goalArmGraphNode.wristAngle);

        this.path = ArmKinematicsCalculator.getOptimalPath(startArmGraphNode, goalArmGraphNode);
        this.currPos = 0;

        ExceptionHelpers.Assert(this.path != null, "The provided start node is not reachable from the provided end node.");
        if (this.path != null)
        {
            this.hasCompleted = false;

            ArmGraphNode currNode = this.path.get(this.currPos);
            ExceptionHelpers.Assert(currNode != null, "The current node is null?!");
            System.out.println("Navigate to " + currNode.shoulderAngle + ", " + currNode.wristAngle);
            System.out.println("Path Size: " + this.path.size());
            this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, currNode.shoulderAngle);
            this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, currNode.wristAngle);
        }
        else
        {
            this.hasCompleted = true;

            System.out.println("Navigate to " + this.shoulderGoalPos + ", " + this.wristGoalPos);
            this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, this.shoulderGoalPos);
            this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, this.wristGoalPos);
        }
    }

    @Override
    public void update()
    {
        ArmGraphNode currNode = this.path.get(this.currPos);

        ExceptionHelpers.Assert(currNode != null, "The current node is null?!");
        if (Helpers.RoughEquals(currNode.shoulderAngle, this.arm.getShoulderPosition(), TuningConstants.ARM_SHOULDER_GOAL_THRESHOLD) &&
            Helpers.RoughEquals(currNode.wristAngle, this.arm.getWristPosition(), TuningConstants.ARM_WRIST_GOAL_THRESHOLD))
        {
            this.currPos++;
            if (this.currPos < this.path.size())
            {
                currNode = this.path.get(this.currPos);
                System.out.println("Navigate to " + currNode.shoulderAngle + ", " + currNode.wristAngle);
                ExceptionHelpers.Assert(currNode != null, "The current node is null?!");
            }
            else
            {
                this.hasCompleted = true;
            }
        }

        this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, currNode.shoulderAngle);
        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, currNode.wristAngle);
    }

    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, this.shoulderGoalPos);
        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, this.wristGoalPos);
    }

    @Override
    public boolean hasCompleted()
    {
        return this.hasCompleted;
    }
}

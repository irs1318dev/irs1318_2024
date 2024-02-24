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
    private enum ArmGraphState
    {
        MovingToNode,
        MovingToGoal,
        Completed
    }

    private final double shoulderGoalPos;
    private final double wristGoalPos;

    private ArmMechanism arm;
    private List<ArmGraphNode> path;
    private int currPos;

    private ArmGraphState state;

    public ArmGraphTask(double shoulderGoalPos, double wristGoalPos)
    {
        this.shoulderGoalPos = shoulderGoalPos;
        this.wristGoalPos = wristGoalPos;
    }

    @Override
    public void begin()
    {
        this.arm = this.getInjector().getInstance(ArmMechanism.class);

        double armShoulderPosition = this.arm.getShoulderPosition();
        double armWristPosition = this.arm.getWristPosition();
        ArmGraphNode startArmGraphNode = ArmKinematicsCalculator.getClosestArmNode(armShoulderPosition, armWristPosition);
        ArmGraphNode goalArmGraphNode = ArmKinematicsCalculator.getClosestArmNode(this.shoulderGoalPos, this.wristGoalPos);

        System.out.println(String.format("Current position (%.2f, %.2f)", armShoulderPosition, armWristPosition));
        System.out.println(String.format("Starting Node (%.2f, %.2f)", startArmGraphNode.shoulderAngle, startArmGraphNode.wristAngle));
        System.out.println(String.format("Goal Node (%.2f, %.2f)", goalArmGraphNode.shoulderAngle, goalArmGraphNode.wristAngle));

        this.path = ArmKinematicsCalculator.getOptimalPath(startArmGraphNode, goalArmGraphNode);
        this.currPos = 0;

        ExceptionHelpers.Assert(this.path != null, "The provided start node is not reachable from the provided end node.");
        if (this.path != null)
        {
            this.state = ArmGraphState.MovingToNode;

            ArmGraphNode currNode = this.path.get(this.currPos);
            ExceptionHelpers.Assert(currNode != null, "The current node is null?!");

            System.out.println(String.format("Navigating to node (%.2f, %.2f)", armShoulderPosition, armWristPosition));
            this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, currNode.shoulderAngle);
            this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, currNode.wristAngle);
        }
        else
        {
            this.state = ArmGraphState.MovingToGoal;

            System.out.println(String.format("Navigating to goal (%.2f, %.2f)", this.shoulderGoalPos, this.wristGoalPos));
            this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, this.shoulderGoalPos);
            this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, this.wristGoalPos);
        }
    }

    @Override
    public void update()
    {
        double shoulderAngle;
        double wristAngle;
        if (this.state == ArmGraphState.MovingToNode)
        {
            ArmGraphNode currNode = this.path.get(this.currPos);

            ExceptionHelpers.Assert(currNode != null, "The current node is null?!");

            double armShoulderPosition = this.arm.getShoulderPosition();
            double armWristPosition = this.arm.getWristPosition();
            if (Helpers.RoughEquals(currNode.shoulderAngle, armShoulderPosition, TuningConstants.ARM_SHOULDER_GOAL_THRESHOLD) &&
                Helpers.RoughEquals(currNode.wristAngle, armWristPosition, TuningConstants.ARM_WRIST_GOAL_THRESHOLD))
            {
                System.out.println(String.format("Reached (%.2f, %.2f)", armShoulderPosition, armWristPosition));
                this.currPos++;
                if (this.currPos < this.path.size())
                {
                    currNode = this.path.get(this.currPos);
                    ExceptionHelpers.Assert(currNode != null, "The current node is null?!");

                    System.out.println(String.format("Navigating to node (%.2f, %.2f)", currNode.shoulderAngle, currNode.wristAngle));
                    shoulderAngle = currNode.shoulderAngle;
                    wristAngle = currNode.wristAngle;
                }
                else
                {
                    System.out.println(String.format("Navigating to goal (%.2f, %.2f)", this.shoulderGoalPos, this.wristGoalPos));
                    this.state = ArmGraphState.MovingToGoal;
                    shoulderAngle = this.shoulderGoalPos;
                    wristAngle = this.wristGoalPos;
                }
            }
            else
            {
                shoulderAngle = currNode.shoulderAngle;
                wristAngle = currNode.wristAngle;
            }
        }
        else // if (this.state == ArmGraphState.MovingToGoal || this.state == ArmGraphState.Completed)
        {
            if (Helpers.RoughEquals(this.shoulderGoalPos, this.arm.getShoulderPosition(), TuningConstants.ARM_SHOULDER_GOAL_THRESHOLD) &&
                Helpers.RoughEquals(this.wristGoalPos, this.arm.getWristPosition(), TuningConstants.ARM_WRIST_GOAL_THRESHOLD))
            {
                this.state = ArmGraphState.Completed;
            }

            shoulderAngle = this.shoulderGoalPos;
            wristAngle = this.wristGoalPos;
        }

       this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, shoulderAngle);
       this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, wristAngle);
    }

    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
    }

    @Override
    public boolean hasCompleted()
    {
        return this.state == ArmGraphState.Completed;
    }
}

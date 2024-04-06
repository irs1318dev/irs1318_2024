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

    private static final boolean DEBUG_PRINTS = true;
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

        if (ArmGraphTask.DEBUG_PRINTS)
        {
            System.out.println(String.format("Current position (%.2f, %.2f)", armShoulderPosition, armWristPosition));
            System.out.println(String.format("Starting Node %s", startArmGraphNode));
            System.out.println(String.format("Goal Node %s", goalArmGraphNode));
        }

        this.path = ArmKinematicsCalculator.getOptimalPath(startArmGraphNode, goalArmGraphNode);
        this.currPos = 0;

        ExceptionHelpers.Assert(this.path != null, "The provided start node is not reachable from the provided end node.");
        if (this.path != null)
        {
            this.state = ArmGraphState.MovingToNode;

            ArmGraphNode currNode = this.path.get(this.currPos);
            ExceptionHelpers.Assert(currNode != null, "The current node is null?!");

            if (ArmGraphTask.DEBUG_PRINTS)
            {
                System.out.println(String.format("Navigating to node %s", currNode));
            }

            this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, currNode.getShoulderAngle());
            if (currNode.isUniversal())
            {
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
            }
            else
            {
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, currNode.getWristAngle());
            }
        }
        else
        {
            this.state = ArmGraphState.MovingToGoal;

            if (ArmGraphTask.DEBUG_PRINTS)
            {
                System.out.println(String.format("Navigating to goal (%.2f, %.2f)", this.shoulderGoalPos, this.wristGoalPos));
            }

            this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, this.shoulderGoalPos);
            this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, this.wristGoalPos);
        }
    }

    @Override
    public void update()
    {
        double shoulderAngle;
        Double wristAngle;
        if (this.state == ArmGraphState.MovingToNode)
        {
            ArmGraphNode currNode = this.path.get(this.currPos);

            ExceptionHelpers.Assert(currNode != null, "The current node is null?!");

            double armShoulderPosition = this.arm.getShoulderPosition();
            double armWristPosition = this.arm.getWristPosition();
            if ((Helpers.RoughEquals(currNode.getShoulderAngle(), armShoulderPosition, TuningConstants.ARM_SHOULDER_GOAL_THRESHOLD) &&
                    (currNode.isUniversal() || Helpers.RoughEquals(currNode.getWristAngle(), armWristPosition, TuningConstants.ARM_WRIST_GOAL_THRESHOLD))) ||
                (currNode.equals(ArmKinematicsCalculator.startingConfiguration) && 
                    (this.arm.getWristLimitSwitchStatus() ||
                        (Helpers.RoughEquals(currNode.getShoulderAngle(), armShoulderPosition, TuningConstants.ARM_SHOULDER_GOAL_THRESHOLD) &&
                        Helpers.RoughEquals(currNode.getWristAngle(), armWristPosition, TuningConstants.ARM_WRIST_GOAL_THRESHOLD_STOW)))))
            {
                if (ArmGraphTask.DEBUG_PRINTS)
                {
                    System.out.println(String.format("Reached (%.2f, %.2f)", armShoulderPosition, armWristPosition));
                }

                this.currPos++;
                if (this.currPos < this.path.size())
                {
                    currNode = this.path.get(this.currPos);
                    ExceptionHelpers.Assert(currNode != null, "The current node is null?!");

                    if (ArmGraphTask.DEBUG_PRINTS)
                    {
                        System.out.println(String.format("Navigating to node %s", currNode));
                    }

                    shoulderAngle = currNode.getShoulderAngle();
                    if (currNode.isUniversal())
                    {
                        wristAngle = TuningConstants.MAGIC_NULL_VALUE;
                    }
                    else
                    {
                        wristAngle = currNode.getWristAngle();
                    }
                }
                else
                {
                    if (ArmGraphTask.DEBUG_PRINTS)
                    {
                        System.out.println(String.format("Navigating to goal (%.2f, %.2f)", this.shoulderGoalPos, this.wristGoalPos));
                    }

                    this.state = ArmGraphState.MovingToGoal;
                    shoulderAngle = this.shoulderGoalPos;
                    wristAngle = this.wristGoalPos;
                }
            }
            else
            {
                shoulderAngle = currNode.getShoulderAngle();
                if (currNode.isUniversal())
                {
                    wristAngle = TuningConstants.MAGIC_NULL_VALUE;
                }
                else
                {
                    wristAngle = currNode.getWristAngle();
                }
            }
        }
        else // if (this.state == ArmGraphState.MovingToGoal || this.state == ArmGraphState.Completed)
        {
            double armShoulderPosition = this.arm.getShoulderPosition();
            double armWristPosition = this.arm.getWristPosition();
            if (Helpers.RoughEquals(this.shoulderGoalPos, armShoulderPosition, TuningConstants.ARM_SHOULDER_GOAL_THRESHOLD) &&
                Helpers.RoughEquals(this.wristGoalPos, armWristPosition, TuningConstants.ARM_WRIST_GOAL_THRESHOLD))
            {
                if (ArmGraphTask.DEBUG_PRINTS)
                {
                    System.out.println(String.format("Reached (%.2f, %.2f)", armShoulderPosition, armWristPosition));
                }

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

package frc.robot.driver.controltasks;

import java.util.List;

import frc.lib.helpers.Graph;
import frc.lib.helpers.GraphNode;
import frc.lib.helpers.Helpers;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.mechanisms.ArmMechanism;

public class ArmGraphTask extends ControlTaskBase
{
    private static ArmGraph graph;
    private static ArmGraphNode startUp;
    private static ArmGraphNode groundPickup;
    private static ArmGraphNode tucked;
    private static ArmGraphNode sourcePickup;
    private static ArmGraphNode upperUnivShot;
    private static ArmGraphNode ampScore;
    private static ArmGraphNode upperIntakeFlipped;
    private static ArmGraphNode trapIntermediate;
    private static ArmGraphNode upperObtuseWrist;
    private static ArmGraphNode tuckedGroundTransition;

    static
    {
        // initialize graph
        ArmGraphTask.graph = new ArmGraph();

        // intialize all of the graph nodes
        ArmGraphTask.startUp = ArmGraphTask.graph.createNode(
            TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION,
            TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION);

        ArmGraphTask.groundPickup = ArmGraphTask.graph.createNode(
            TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL,
            TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP);

        ArmGraphTask.tucked = ArmGraphTask.graph.createNode(
            TuningConstants.ARM_SHOULDER_POSITION_TUCKED,
            TuningConstants.ARM_WRIST_POSITION_TUCKED_SHOT);

        ArmGraphTask.sourcePickup = ArmGraphTask.graph.createNode(
            TuningConstants.ARM_SHOULDER_POSITION_SOURCE_PICKUP,
            TuningConstants.ARM_WRIST_POSITION_SOURCE_PICKUP);

        ArmGraphTask.upperUnivShot = ArmGraphTask.graph.createNode(
            TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL,
            TuningConstants.ARM_WRIST_POSITION_UPPER_UNIVERSAL_SHOT);

        ArmGraphTask.ampScore = ArmGraphTask.graph.createNode(
            TuningConstants.ARM_SHOULDER_POSITION_AMP_SCORE,
            TuningConstants.ARM_WRIST_POSITION_AMP_SCORE);

        ArmGraphTask.upperIntakeFlipped = ArmGraphTask.graph.createNode(
            TuningConstants.ARM_SHOULDER_POSITION_INTAKE_FLIPPED,
            TuningConstants.ARM_WRIST_POSITION_INTAKE_FLIPPED);
            
        ArmGraphTask.trapIntermediate = ArmGraphTask.graph.createNode(
            TuningConstants.ARM_SHOULDER_POSITION_TRAP_INTERMEDIATE,
            TuningConstants.ARM_WRIST_POSITION_TRAP_INTERMEDIATE);

        ArmGraphTask.upperObtuseWrist = ArmGraphTask.graph.createNode(
            TuningConstants.ARM_SHOULDER_POSITION_INTAKE_OBTUSE,
            TuningConstants.ARM_WRIST_POSITION_INTAKE_OBTUSE);
            
        ArmGraphTask.tuckedGroundTransition = ArmGraphTask.graph.createNode(
            TuningConstants.ARM_SHOULDER_POSITION_TUCKED,
            TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP);

        // create all of the links between the nodes
        ArmGraphTask.graph.connectBidirectional(
            ArmGraphTask.startUp, 
            ArmGraphTask.groundPickup, 
            TuningConstants.STARTUP_AND_GROUND_PICKUP_WEIGHT);
        ArmGraphTask.graph.connectBidirectional(
            ArmGraphTask.startUp, 
            ArmGraphTask.sourcePickup, 
            TuningConstants.STARTUP_AND_SOURCE_PICKUP_WEIGHT);
        ArmGraphTask.graph.connectBidirectional(
            ArmGraphTask.startUp, 
            ArmGraphTask.upperIntakeFlipped, 
            TuningConstants.STARTUP_AND_UPPER_INTAKE_FLIPPED_WEIGHT);
        ArmGraphTask.graph.connectBidirectional(
            ArmGraphTask.sourcePickup, 
            ArmGraphTask.upperIntakeFlipped, 
            TuningConstants.SOURCE_PICKUP_AND_UPPER_INTAKE_FLIPPED_WEIGHT);
        ArmGraphTask.graph.connectBidirectional(
            ArmGraphTask.upperIntakeFlipped, 
            ArmGraphTask.trapIntermediate, 
            TuningConstants.UPPER_INTALE_FLIPPED_AND_TRAP_INTER_WEIGHT);
        ArmGraphTask.graph.connectBidirectional(
            ArmGraphTask.upperIntakeFlipped, 
            ArmGraphTask.upperUnivShot, 
            TuningConstants.UPPER_INTALE_FLIPPED_AND_UPPER_UNIV_WEIGHT);
    
        ArmGraphTask.graph.connectBidirectional(
            ArmGraphTask.upperUnivShot, 
            ArmGraphTask.groundPickup, 
            TuningConstants.UPPER_UNIV_AND_GROUND_PICKUP_WEIGHT);
        ArmGraphTask.graph.connectBidirectional(
            ArmGraphTask.upperUnivShot, 
            ArmGraphTask.upperObtuseWrist, 
            TuningConstants.UPPER_UNIV_AND_OBTUSE_WRIST_WEIGHT);
        ArmGraphTask.graph.connectBidirectional(
            ArmGraphTask.ampScore, 
            ArmGraphTask.upperObtuseWrist, 
            TuningConstants.AMP_SCORE_AND_OBTUSE_WRIST_WEIGHT);
        ArmGraphTask.graph.connectBidirectional(
            ArmGraphTask.upperObtuseWrist, 
            ArmGraphTask.groundPickup, 
            TuningConstants.OBTUSE_WRIST_AND_GROUND_PICKUP_WEIGHT);
        ArmGraphTask.graph.connectBidirectional(
            ArmGraphTask.upperObtuseWrist, 
            ArmGraphTask.tucked, 
            TuningConstants.OBTUSE_WRIST_AND_TUCKED_WEIGHT);

        ArmGraphTask.graph.connect(
            ArmGraphTask.groundPickup, 
            ArmGraphTask.tucked, 
            TuningConstants.GROUND_PICKUP_TO_TUCKED_WEIGHT);
        ArmGraphTask.graph.connect(
            ArmGraphTask.tucked, 
            ArmGraphTask.tuckedGroundTransition, 
            TuningConstants.TUCKED_TO_TUCKED_GROUND_TRANS_WEIGHT);
        ArmGraphTask.graph.connect(
            ArmGraphTask.tuckedGroundTransition, 
            ArmGraphTask.groundPickup, 
            TuningConstants.TUCCKED_GROUND_TRANS_TO_GROUND_PICKUP_WEIGHT);
    }

    private double shoulderGoalPos;
    private double wristGoalPos;

    private ArmMechanism arm;

    private List<ArmGraphNode> path;

    private boolean hasCompleted = false;

    public ArmGraphTask(double shoulderGoalPos, double wristGoalPos)
    {
        this.shoulderGoalPos = shoulderGoalPos;
        this.wristGoalPos = wristGoalPos;
    }

    @Override
    public void begin()
    {
        ArmGraphNode goalArmGraphNode = getClosestArmNode(this.shoulderGoalPos, this.wristGoalPos);
        ArmGraphNode startArmGraphNode = getClosestArmNode(this.arm.getTheta1(), this.arm.getTheta2());

        this.path = ArmGraphTask.graph.getOptimalPath(startArmGraphNode, goalArmGraphNode);
    }

    @Override
    public void update()
    {
        int curPos = 0;

        if ( Helpers.RoughEquals(this.path.get(curPos).shoulderAngle, this.arm.getTheta1(), TuningConstants.ARM_SHOULDER_GOAL_THRESHOLD) 
            && Helpers.RoughEquals(this.path.get(curPos).wristAngle, this.arm.getTheta2(), TuningConstants.ARM_WRIST_GOAL_THRESHOLD) )
        {
            if (curPos != this.path.size())
            {
                curPos += 1;
            }
            else
            {
                this.hasCompleted = true;
            }
        }

        this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, this.path.get(curPos).shoulderAngle);
        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, this.path.get(curPos).wristAngle);
        
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

    public ArmGraphNode getClosestArmNode(double shoulderPos, double wristPos)
    {
        double diff = 100;
        ArmGraphNode nearest = new ArmGraphNode(1000, 1000);

        for (ArmGraphNode cur : ArmGraphTask.graph.getNodes())
        {
            if ((Math.abs(cur.shoulderAngle - shoulderPos) * TuningConstants.ARM_SHOULDER_WEIGHT_MULTIPLIER
                + Math.abs(cur.wristAngle - wristPos) * TuningConstants.ARM_WRIST_WEIGHT_MULTIPLIER)
                < diff)
            {
                nearest = cur;
                diff = (Math.abs(cur.shoulderAngle - shoulderPos) * TuningConstants.ARM_SHOULDER_WEIGHT_MULTIPLIER
                    + Math.abs(cur.wristAngle - wristPos) * TuningConstants.ARM_WRIST_WEIGHT_MULTIPLIER);
            }
        }

        return nearest;
    }

    private static class ArmGraph extends Graph<ArmGraphNode>
    {
        public ArmGraph()
        {
            super();
        }

        public ArmGraphNode createNode(double shoulderAngle, double wristAngle)
        {
            ArmGraphNode node = new ArmGraphNode(shoulderAngle, wristAngle);
            this.addNode(node);
            return node;
        }
    }

    private static class ArmGraphNode extends GraphNode
    {
        public final double wristAngle;
        public final double shoulderAngle;

        public ArmGraphNode(double shoulderAngle, double wristAngle)
        {
            super();

            this.shoulderAngle = shoulderAngle;
            this.wristAngle = wristAngle;
        }
    }
}

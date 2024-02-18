package frc.robot.driver.controltasks;

import frc.lib.helpers.Graph;
import frc.lib.helpers.GraphNode;
import frc.robot.TuningConstants;

public class ArmGraphTask {
    ArmGraph graph = new ArmGraph();

    ArmGraphNode startUp = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION,
        TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION);
        
    ArmGraphNode groundPickup = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL,
        TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP);

    ArmGraphNode tucked = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_TUCKED,
        TuningConstants.ARM_WRIST_POSITION_TUCKED_SHOT);

    ArmGraphNode sourcePickup = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_SOURCE_PICKUP,
        TuningConstants.ARM_WRIST_POSITION_SOURCE_PICKUP);

    ArmGraphNode upperUnivShot = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL,
        TuningConstants.ARM_WRIST_POSITION_UPPER_UNIVERSAL_SHOT);
        
    ArmGraphNode ampScore = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_AMP_SCORE,
        TuningConstants.ARM_WRIST_POSITION_AMP_SCORE);

    ArmGraphNode upperIntakeFlipped = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_INTAKE_FLIPPED,
        TuningConstants.ARM_WRIST_POSITION_INTAKE_FLIPPED);
        
    ArmGraphNode trapIntermediate = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_TRAP_INTERMEDIATE,
        TuningConstants.ARM_WRIST_POSITION_TRAP_INTERMEDIATE);

    ArmGraphNode upperObtuseWrist = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_INTAKE_OBTUSE,
        TuningConstants.ARM_WRIST_POSITION_INTAKE_OBTUSE);
        
    ArmGraphNode tuckedGroundTransition = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_TUCKED,
        TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP);

    // graph.connectBidirectional(startUp, groundPickup, );
}


final class ArmGraph extends Graph<ArmGraphNode>
{
    public ArmGraph()
    {
        super();
    }
    public ArmGraphNode createNode(double shoulderAngle, double wristAngle)
    {
        ArmGraphNode node = new ArmGraphNode(shoulderAngle, wristAngle);
        // ArmGraph.addNode(node);
        return node;
    }
}

final class ArmGraphNode extends GraphNode
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
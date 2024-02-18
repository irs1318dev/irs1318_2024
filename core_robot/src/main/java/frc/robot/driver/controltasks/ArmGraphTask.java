// package frc.robot.driver.controltasks;

// import java.util.List;

// import frc.lib.helpers.Graph;
// import frc.lib.helpers.GraphNode;
// import frc.lib.helpers.Helpers;
// import frc.robot.TuningConstants;
// import frc.robot.driver.AnalogOperation;

// public class ArmGraphTask extends ControlTaskBase {
//     ArmGraph graph = new ArmGraph();

//     ArmGraphNode startUp = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION,
//         TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION);
        
//     ArmGraphNode groundPickup = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL,
//         TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP);

//     ArmGraphNode tucked = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_TUCKED,
//         TuningConstants.ARM_WRIST_POSITION_TUCKED_SHOT);

//     ArmGraphNode sourcePickup = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_SOURCE_PICKUP,
//         TuningConstants.ARM_WRIST_POSITION_SOURCE_PICKUP);

//     ArmGraphNode upperUnivShot = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL,
//         TuningConstants.ARM_WRIST_POSITION_UPPER_UNIVERSAL_SHOT);
        
//     ArmGraphNode ampScore = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_AMP_SCORE,
//         TuningConstants.ARM_WRIST_POSITION_AMP_SCORE);

//     ArmGraphNode upperIntakeFlipped = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_INTAKE_FLIPPED,
//         TuningConstants.ARM_WRIST_POSITION_INTAKE_FLIPPED);
        
//     ArmGraphNode trapIntermediate = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_TRAP_INTERMEDIATE,
//         TuningConstants.ARM_WRIST_POSITION_TRAP_INTERMEDIATE);

//     ArmGraphNode upperObtuseWrist = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_INTAKE_OBTUSE,
//         TuningConstants.ARM_WRIST_POSITION_INTAKE_OBTUSE);
        
//     ArmGraphNode tuckedGroundTransition = graph.createNode(TuningConstants.ARM_SHOULDER_POSITION_TUCKED,
//         TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP);

//     List<ArmGraphNode> allObjectList = new List<ArmGraphNode>() {
//         add(startUp);
//         add(groundPickup);
//         add(tucked);
//         add(sourcePickup);
//         add(upperUnivShot);
//         add(ampScore);
//         add(upperIntakeFlipped);
//         add(trapIntermediate);
//         add(upperObtuseWrist);
//         add(tuckedGroundTransition);
//     };

//     private double shoulderGoalPos;
//     private double wristGoalPos;

//     private ArmMechanism arm;

//     private List<ArmGraphNode> path = new List<ArmGraphNode>();

//     private boolean hasCompleted = false;
    
    

//     graph.connectBidirectional(startUp, groundPickup, TuningConstants.STARTUP_AND_GROUND_PICKUP_WEIGHT);
//     graph.connectBidirectional(startUp, sourcePickup, TuningConstants.STARTUP_AND_SOURCE_PICKUP_WEIGHT);
//     graph.connectBidirectional(startUp, upperIntakeFlipped, TuningConstants.STARTUP_AND_UPPER_INTAKE_FLIPPED_WEIGHT);
//     graph.connectBidirectional(sourcePickup, upperIntakeFlipped, TuningConstants.SOURCE_PICKUP_AND_UPPER_INTAKE_FLIPPED_WEIGHT);
//     graph.connectBidirectional(upperIntakeFlipped, trapIntermediate, TuningConstants.UPPER_INTALE_FLIPPED_AND_TRAP_INTER_WEIGHT);
//     graph.connectBidirectional(upperIntakeFlipped, upperUnivShot, TuningConstants.UPPER_INTALE_FLIPPED_AND_UPPER_UNIV_WEIGHT);

//     graph.connectBidirectional(upperUnivShot, groundPickup, TuningConstants.UPPER_UNIV_AND_GROUND_PICKUP_WEIGHT);
//     graph.connectBidirectional(upperUnivShot, upperObtuseWrist, TuningConstants.UPPER_UNIV_AND_OBTUSE_WRIST_WEIGHT);
//     graph.connectBidirectional(ampScore, upperObtuseWrist, TuningConstants.AMP_SCORE_AND_OBTUSE_WRIST_WEIGHT);
//     graph.connectBidirectional(upperObtuseWrist, groundPickup, TuningConstants.OBTUSE_WRIST_AND_GROUND_PICKUP_WEIGHT);
//     graph.connectBidirectional(upperObtuseWrist, tucked, TuningConstants.OBTUSE_WRIST_AND_tucked_WEIGHT);

//     graph.connect(groundPickup, tucked, TuningConstants.GROUND_PICKUP_TO_TUCKED_WEIGHT);
//     graph.connect(tucked, tuckedGroundTransition, TuningConstants.TUCKED_TO_TUCKED_GROUND_TRANS_WEIGHT);
//     graph.connect(tuckedGroundTransition, groundPickup, TuningConstants.TUCCKED_GROUND_TRANS_TO_GROUND_PICKUP_WEIGHT);

//     public ArmGraphTask(double shoulderGoalPos, double wristGoalPos)
//     {
//         this.shoulderGoalPos = shoulderGoalPos;
//         this.wristGoalPos = wristGoalPos;
//     }

//     @Override
//     public void begin()
//     {
//         ArmGraphNode goalArmGraphNode = getClosestArmNode(this.shoulderGoalPos, this.wristGoalPos);
//         ArmGraphNode startArmGraphNode = getClosestArmNode(this.arm.getTheta1(), this.arm.getTheta2());

//         this.path = graph.getOptimalPath(startArmGraphNode, goalArmGraphNode);
//     }

//     @Override
//     public void update()
//     {
//         int curPos = 0;

//         if( Helpers.RoughEquals(this.path.get(curPos).shoulderAngle, this.arm.getTheta1, TuningConstants.ARM_SHOULDER_GOAL_THRESHOLD) 
//             && Helpers.RoughEquals(this.path.get(curPos).wristAngle, this.arm.getTheta2, TuningConstants.ARM_WRIST_GOAL_THRESHOLD) )
//         {
//             if(curPos != this.path.size())
//             {
//                 curPos += 1;
//             }
//             else {
//                 this.hasCompleted = true;
//             }
//         }

//         this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, this.path.get(curPos).shoulderAngle);
//         this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, this.path.get(curPos).wristAngle);
        
//     }

//     @Override
//     public void end()
//     {
//         this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, this.shoulderGoalPos);
//         this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, this.wristGoalPos);
//     }

//     @Override
//     public boolean hasCompleted()
//     {
//         return this.hasCompleted;
//     }

//     public ArmGraphNode getClosestArmNode(double shoulderPos, double wristPos)
//     {
//         double diff = 100;
//         ArmGraphNode nearest = new ArmGraphNode(1000, 1000);

//         for (ArmGraphNode cur : allObjectList())
//         {
//             if( (Math.abs(cur.shoulderAngle - shoulderPos) * TuningConstants.ARM_SHOULDER_WEIGHT_MULTIPLIER
//                 + Math.abs(cur.wristAngle - wristPos) * TuningConstants.ARM_WRIST_WEIGHT_MULTIPLIER)
//                 < diff)
//             {
//                 nearest = cur;
//                 diff = (Math.abs(cur.shoulderAngle - shoulderPos) * TuningConstants.ARM_SHOULDER_WEIGHT_MULTIPLIER
//                     + Math.abs(cur.wristAngle - wristPos) * TuningConstants.ARM_WRIST_WEIGHT_MULTIPLIER);
//             }
//         }

//         return nearest;
//     }
// }


// final class ArmGraph extends Graph<ArmGraphNode>
// {
//     public ArmGraph()
//     {
//         super();
//     }
//     public ArmGraphNode createNode(double shoulderAngle, double wristAngle)
//     {
//         ArmGraphNode node = new ArmGraphNode(shoulderAngle, wristAngle);
//         // ArmGraph.addNode(node);
//         return node;
//     }
// }

// final class ArmGraphNode extends GraphNode
// {
//     public final double wristAngle;
//     public final double shoulderAngle;

//     public ArmGraphNode(double shoulderAngle, double wristAngle)
//     {
//         super();
//         this.shoulderAngle = shoulderAngle;
//         this.wristAngle = wristAngle;
//     }    
// }
/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.struct.bio;

import com.jme3.math.FastMath;
import swim.core.StateChangeReason;
import swim.core.motion.CompositeTurnSequence;
import swim.sim.uwswarm.UWVehicleControl;
import swim.sim.uwswarm.intelli.brain.MiniBrain;
import swim.sim.uwswarm.intelli.brain.constr.Action;
import swim.sim.uwswarm.intelli.brain.constr.ActionType;
import swim.sim.uwswarm.intelli.brain.funct.DynamicAssocFunct;
import swim.sim.uwswarm.intelli.brain.struct.artif.Duration;
import swim.sim.uwswarm.intelli.brain.struct.artif.MotionPace;
import swim.sim.uwswarm.intelli.brain.struct.artif.MotionPeriodicity;
import swim.sim.uwswarm.intelli.brain.struct.artif.MotionType;
import swim.sim.uwswarm.intelli.brain.struct.artif.MotorFuncParams;
import swim.sim.uwswarm.intelli.brain.struct.artif.MotorFuncType;
import swim.sim.uwswarm.intelli.brain.struct.artif.MotorFuncsRepository;
import swim.sim.uwswarm.intelli.brain.struct.artif.SequenceRepository;
import swim.sim.uwswarm.intelli.brain.struct.artif.TriggeringEvent;

/**
 *
 * @author Sherif
 */
public class MiniMotorFuncArea extends MotorFuncArea {

    // =========================================================================
    // Constants
    // =========================================================================
    private static final int NUM_SUB_AREAS = 1;
    private final float PACE_LAST_NEIGH_ENC_TIME_PROP_CONST = 0.3f;
    private final int SPEED_CHANGE_PERIOD = 100; // TICKS
    // =========================================================================
    // Variables
    // =========================================================================
    private MiniBrain brain;
    private Action activeAction;
    private UWVehicleControl agentControl;
    
    private DynamicAssocFunct actionsToMotorFunctionsMap;
    
    private SequenceRepository sequenceRepo;
    
    private MotionPeriodicity prevActionMotionPeriodicity;
    
    private boolean neverPerformedAction = true;
    private ActionType previousAction;
    // =========================================================================
    
    public MiniMotorFuncArea(MiniBrain brain) {
        super(NUM_SUB_AREAS);
        
        this.brain = brain;
        this.agentControl = brain.getAgentController();
        
        // Dynamic association maps
        buildActionsToMotorFunctionsMap();
        
        sequenceRepo = new SequenceRepository(agentControl);
    }

    @Override
    protected void function() {
        if( activeAction == null ) {
            System.out.println("Higher Mental Function area must be excited first with an action signal");
            return;
        }
        performAction();
    }

    @Override
    protected void fire() {
        System.out.println("Action successfully queued.");
    }

    @Override
    public void excite(ExcitationSignal actionSignal) {
        activeAction = ((ActionSignal)actionSignal).getAction();
        this.function();  
    }
    
    private void performAction() {
        
        if( neverPerformedAction ) {
            neverPerformedAction = false;
            // In this special case, the action set by the Higher Mental Function
            // (predefinded mission stage action) takes place 
            previousAction = activeAction.getType();
            return;
        }
        
        if( previousAction == activeAction.getType() ) {
            // For now, I chose to update active mission action. In reality, a
            // short term behavior could be executing instead of a mission action
            // Change this as soon as you can!
            agentControl.updateActiveMissionAction(0.0f); // hack
            return;
        }
        
        // Get the motor function underlying the action
        MotorFunction motorFunction = 
                (MotorFunction)actionsToMotorFunctionsMap.getAssociation(activeAction.getType().hashCode());
        
        // Parse its parameters
        MotorFuncParams params = motorFunction.getParameters();
        
        // =====================================================================
        // Motion Pace Determination
        // =====================================================================
        
        // Hold new AUV speed (pace)
        float newAgentSpeed =  0;
        
        // Handle the special case where the pace has to copy that of the most
        // rewarded action
        MotionPace[] motionPaces = MotionPace.values();
        if( params.pace.equals(MotionPace.OF_BEST_REWARDED_ACTION) ){
            
            Action bestRewardedAction = brain.getHigherMentalFunc().getHighestRewActForCurrSituation();
            
            // Get the motor function underlying the action
            MotorFunction motorFunc = 
                    (MotorFunction)actionsToMotorFunctionsMap.getAssociation(bestRewardedAction.getType().hashCode());

            // Parse its parameters
            MotorFuncParams mfParams = motorFunc.getParameters();
            
            // Use its pace
            params.pace = mfParams.pace;
           
        }
        
        // Handle the special case where a random pace has to be selected
        while( params.pace.equals(MotionPace.RANDOM_AMONG_PACES) ) {
            params.pace = motionPaces[FastMath.rand.nextInt(motionPaces.length)];
        }
        
        
        // Check motion pace
        switch( params.pace ){
            case PROP_TO_LAST_NEIGH_ENCOUNTER_TIME:
                
                // Get last neighbor encounter time
                int lastNeighEncTime = 
                        brain.getSensoryArea().getDurationSince( TriggeringEvent.LAST_NEIGHBOR_ENCOUNTER );
                
                // change the speed proportionally to that time
                newAgentSpeed = PACE_LAST_NEIGH_ENC_TIME_PROP_CONST * lastNeighEncTime;

                break;
            case AVERAGE_OF_NEIGHBORS:
                // Get average neighbors speed from agent controller
                // Note that getAvgNeighDirection() returns the estimated,
                // non-normlized average velocity vector
                newAgentSpeed = agentControl.getAvgNeighDirection().length();
                break;
            case FAST:
            case FAST_THEN_SLOW:
                newAgentSpeed = agentControl.getMaxVehicleSpeed();
                break;
            case SLOW:
            case SLOW_THEN_FAST:
                newAgentSpeed = agentControl.getMinVehicleSpeed();               
                break;
            case MEDIUM:
                newAgentSpeed = ( agentControl.getMinVehicleSpeed() + agentControl.getMaxVehicleSpeed() )/2;
                break;
            case SAME:
                newAgentSpeed = agentControl.getActiveVelocity().length();      
                break;
            case RANDOM:
                newAgentSpeed = FastMath.rand.nextFloat() * agentControl.getMaxVehicleSpeed();                
        }
        
        // Save current state of the controller
        agentControl.saveState();

        // Set a timer to increase/decrease speed for a predefined time
        // before the original speed is restored
        if( params.pace.equals(MotionPace.FAST_THEN_SLOW) || 
                params.pace.equals(MotionPace.SLOW_THEN_FAST) ) {
            agentControl.enableSpeedTimer();
            agentControl.setSpeedTimer(SPEED_CHANGE_PERIOD);
        }
        
        // Update speed
        agentControl.updateVehicleSpeed(newAgentSpeed);
        
        // =====================================================================
        // Determination of: 
        // Motor Function Type, 
        // Motion Periodicity (used only by sequences), and 
        // Motion Duration    (used only by algorithms) 
        // =====================================================================
        MotorFuncType mfType = params.type;
        MotionPeriodicity periodicity = params.periodicity;
        
        // Motion Algorithms
        if( mfType.getMotionType().equals(MotionType.ALGORITHM) ) {           
            
            // Get motion duration
            Duration motionDuration = params.duration;
        
            // ===============================================================================================================
            agentControl.requestFSMStateChange(StateChangeReason.RUN_MOTION_ALG, mfType.getMotionAlgorithm(), motionDuration);
            // ===============================================================================================================
            
        // Motion Sequences
        } else if( mfType.getMotionType().equals(MotionType.SEQUENCE) ) {

            // Handle the spcial case of "OF_BEST_REWARDED_ACTION" periodicity
            if( periodicity.type().equals(MotionPeriodicity.Type.OF_BEST_REWARDED_ACTION) ) {
                
                Action bestRewardedAction = brain.getHigherMentalFunc().getHighestRewActForCurrSituation();

                // Get the motor function underlying the action
                MotorFunction motorFunc = 
                        (MotorFunction)actionsToMotorFunctionsMap.getAssociation(bestRewardedAction.getType().hashCode());

                // Parse its parameters
                MotorFuncParams mfParams = motorFunc.getParameters();
                
                // Use its periodicity
                periodicity = mfParams.periodicity;
            }
            
            // Handle the spcial case of "SAME_AS_BEFORE" periodicity
            if( periodicity.type().equals(MotionPeriodicity.Type.SAME_AS_BEFORE) ) {
                if( prevActionMotionPeriodicity != null ) {
                    periodicity = prevActionMotionPeriodicity;
                } else {
                    periodicity = new MotionPeriodicity(MotionPeriodicity.Type.RANDOM_AMONG_TYPES);
                }
            }
            
            // Handle the special case of "RANDOM_AMONG_TYPES" periodicity
            MotionPeriodicity.Type[] periodicityTypes = MotionPeriodicity.Type.values();
            while( periodicity.type().equals(MotionPeriodicity.Type.RANDOM_AMONG_TYPES) ) {
                periodicity.setType(periodicityTypes[FastMath.rand.nextInt(periodicityTypes.length)]);
            }
            
            // Periodic sequences
            if( periodicity.type().equals(MotionPeriodicity.Type.PERIODIC) ) {
                
                int repititions = periodicity.repititions();
                
                CompositeTurnSequence seq = sequenceRepo.getSequence(mfType.getSequenceName());
                CompositeTurnSequence repeatedSeq = new CompositeTurnSequence();
                
                while( repititions > 0 ) {
                    repeatedSeq.addSequenceSet(seq);
                }
                // ===============================================================================================================
                agentControl.requestFSMStateChange(StateChangeReason.RUN_COMP_TURN_SEQUENCE, repeatedSeq);
                // ===============================================================================================================
            }

        }

        
        // =====================================================================
        // Remember state
        prevActionMotionPeriodicity = periodicity;
        
        fire();
    }
    
    private void buildActionsToMotorFunctionsMap() {
        
        actionsToMotorFunctionsMap = new DynamicAssocFunct();
        
        MotorFuncsRepository motorFuncsRepo = new MotorFuncsRepository();

        actionsToMotorFunctionsMap.addAssociation(
                ActionType.FIND_NEIGHBORS_LOCALLY.hashCode(),
                motorFuncsRepo.get("Reconnaissance")
        );
        
        actionsToMotorFunctionsMap.addAssociation(
                ActionType.FIND_NEIGHBORS_GLOBALLY.hashCode(),
                motorFuncsRepo.get("WideAreaNeighborsSearch")
        );
        
        actionsToMotorFunctionsMap.addAssociation(
                ActionType.FLOCK.hashCode(),
                motorFuncsRepo.get("NeighborsFlocking")
        );
        
        actionsToMotorFunctionsMap.addAssociation(
                ActionType.ACT_CREATIVELY.hashCode(),
                motorFuncsRepo.get("CreativeMoves")
        );
        
        actionsToMotorFunctionsMap.addAssociation(
                ActionType.ESCAPE_CROWD.hashCode(),
                motorFuncsRepo.get("NeighborsEscape")
        );
        
        actionsToMotorFunctionsMap.addAssociation(
                ActionType.CHASE_TARGETS.hashCode(),
                motorFuncsRepo.get("TargetChasing")
        );
        
        actionsToMotorFunctionsMap.addAssociation(
                ActionType.CATCH_UP_WITH_NEIGHBORS.hashCode(),
                motorFuncsRepo.get("NeighborsChasing")
        );
        
        actionsToMotorFunctionsMap.addAssociation(
                ActionType.SCURRY_RANDOMLY.hashCode(),
                motorFuncsRepo.get("RandomScurrying")
        );
        
        actionsToMotorFunctionsMap.addAssociation(
                ActionType.SEARCH_TARGETS_LOCALLY.hashCode(),
                motorFuncsRepo.get("ShortRandomExploration")
        );
        
        actionsToMotorFunctionsMap.addAssociation(
                ActionType.KEEP_CURRENT_ACTION.hashCode(),
                motorFuncsRepo.get("KeepingItUp")
        );
        
        actionsToMotorFunctionsMap.addAssociation(
                ActionType.KEEP_CURRENT_PLACE.hashCode(),
                motorFuncsRepo.get("Circling")
        );

        actionsToMotorFunctionsMap.addAssociation(
                ActionType.SEARCH_TARGETS_GLOBALLY.hashCode(),
                motorFuncsRepo.get("StableExploration")
        );
        
        actionsToMotorFunctionsMap.addAssociation(
                ActionType.THE_MOST_REWARDED.hashCode(),
                motorFuncsRepo.get("MotionOfMostRewardedAction")
        );
        
        actionsToMotorFunctionsMap.addAssociation(
                ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN.hashCode(),
                motorFuncsRepo.get("MissionStageAction")
        );
        
        actionsToMotorFunctionsMap.addAssociation(
                ActionType.SURFACE.hashCode(),
                motorFuncsRepo.get("Surface")
        );
        
    }
    
    public CompositeTurnSequence getSequenceFromRepo( String sequenceName ) {
        return sequenceRepo.getSequence(sequenceName);
    }

    public Action getActiveAction() {
        return activeAction;
    }

}

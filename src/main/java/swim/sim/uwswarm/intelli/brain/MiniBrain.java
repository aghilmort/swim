package swim.sim.uwswarm.intelli.brain;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.logging.Level;
import java.util.logging.Logger;
import swim.exception.StaticMapModificationException;
import swim.sim.uwswarm.UWVehicleControl;
import swim.sim.uwswarm.intelli.brain.struct.bio.ExcitationSignal;
import swim.sim.uwswarm.intelli.Nutrient;
import swim.sim.uwswarm.intelli.brain.constr.ActionType;
import swim.sim.uwswarm.intelli.brain.constr.Reward;
import swim.sim.uwswarm.intelli.brain.constr.RewardSize;
import swim.sim.uwswarm.intelli.brain.constr.RewardSpan;
import swim.sim.uwswarm.intelli.brain.constr.ShortTermRewardType;
import swim.sim.uwswarm.intelli.brain.funct.StaticAssocFunct;
import swim.sim.uwswarm.intelli.brain.state.ActionReinforcements;
import swim.sim.uwswarm.intelli.brain.constr.Emotion;
import swim.sim.uwswarm.intelli.brain.constr.LongTermRewardType;
import swim.sim.uwswarm.intelli.brain.constr.Situation;
import swim.sim.uwswarm.intelli.brain.struct.artif.Trigger;
import swim.sim.uwswarm.intelli.brain.struct.bio.Feelings;
import swim.sim.uwswarm.intelli.brain.struct.bio.MiniEmotionalArea;
import swim.sim.uwswarm.intelli.brain.struct.bio.MiniHigherMentalFunc;
import swim.sim.uwswarm.intelli.brain.struct.bio.MiniMotorFuncArea;
import swim.sim.uwswarm.intelli.brain.struct.bio.MiniSensoryArea;
import swim.sim.uwswarm.intelli.resource.BatteryLevel;
import swim.sim.uwswarm.intelli.resource.TimeDiff;
import swim.sim.uwswarm.intelli.sense.Senses;

/**
 *
 * @author Sherif
 */
public class MiniBrain implements Brain {
    
    // =========================================================================
    // Constants
    // =========================================================================
    public final float DEFAULT_EXCITATION_SIGNAL_STRENGTH = 0.33f;
    // =========================================================================
    // Brain variables
    // =========================================================================
    private UWVehicleControl auvController;
    
    private StaticAssocFunct triggersToEmotionsMap,
                             awarenessToEmotionsMap,
                             actionsToExpectedRewardsMap;
    
    // Brain areas
    MiniEmotionalArea emotionalArea;
    MiniMotorFuncArea motorFuncArea;
    MiniHigherMentalFunc higherMentalFunc;
    MiniSensoryArea sensoryArea;
    
    private ArrayList<Trigger> stimuli;
    
    private Situation situation, prevSituation;
    
    // =========================================================================
    
    public MiniBrain(UWVehicleControl auvController) {
        
        // Reference to the vehicle controller
        this.auvController = auvController;
        
        // 1) Build static and dynamic associations
        
        // Static association maps
        buildTriggersToEmotionsMap();
        buildAwarenessToEmotionsMap();
        buildActionsToRewardsMap();
        
        // 2) Build different brain areas
        emotionalArea = new MiniEmotionalArea(this);
        motorFuncArea = new MiniMotorFuncArea(this);
        sensoryArea = new MiniSensoryArea(this, auvController);
        higherMentalFunc = new MiniHigherMentalFunc(this);
                
    }
    
    @Override
    public void excite(ExcitationSignal signal) {
    
    }

    @Override
    public void makeDecision() {
        
        // 1) Excite sensory area then extract its stimuli
        //    - Sensory area excites Emotional and HMF areas
        //    - HMF area, in turn, excites Motor function area
        sensoryArea.excite();
        stimuli = sensoryArea.getStimuli();

        // 2) After exciting the feelings contained in the trigger, get emotional state and associate
        // with the set of triggers to define a situation (from the agent's point of view). This situation is 
        // used to determine an action to be taken, taking into consideration the mission plan as well.
        
        // Actual resulting (dominan) emotion after excitation
        Emotion resultingEmotion = emotionalArea.getDominantEmotion();
        
        if( !stimuli.isEmpty() ){
            situation = new Situation(stimuli, resultingEmotion);
        } else {
            situation = prevSituation;
        }

        prevSituation = situation;
    }

    @Override
    public void learn() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public void nurture(Nutrient nutrient) {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }
    
    /**
     * Maps triggers (the result of senses, elapsed time, rewards, and action 
     * reinforcements) to feelings that need to be excited when these triggers
     * are active.
     */
    private void buildTriggersToEmotionsMap() {
        
        triggersToEmotionsMap = new StaticAssocFunct();
        
        try {
            
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.NO_NEIGHBORS, TimeDiff.BIG}),
                new Emotion(new Feelings[]{Feelings.LONELY_SAD})
            );

            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.NO_TARGETS, TimeDiff.BIG}),
                new Emotion(new Feelings[]{Feelings.BORED_SAD})
            );
            
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.NO_TARGETS, TimeDiff.VERY_BIG}),
                new Emotion(new Feelings[]{Feelings.DEPRESSED_SAD}) 
            );

            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.NEIGHBOR_COLLISION, TimeDiff.SMALL}),
                new Emotion(new Feelings[]{Feelings.ANGREY_MAD}) 
            );
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.NEIGHBOR_COLLISION, TimeDiff.VERY_SMALL}),
                new Emotion(new Feelings[]{Feelings.ANGREY_MAD})   
            );

            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.A_TARGET_NEARBY, TimeDiff.BIG}),
                new Emotion(new Feelings[]{Feelings.SELFISH_MAD})
            );
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.FEW_TARGETS, Senses.MANY_NEIGHBORS}),
                new Emotion(new Feelings[]{Feelings.SELFISH_MAD})   
            );

            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.HEADING_AWAY_FROM_NEIGHBORS}),
                new Emotion(new Feelings[]{Feelings.SUBMISSIVE_SCARED})    
            );

            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.NO_NEIGHBORS, TimeDiff.BIG}),
                new Emotion(new Feelings[]{Feelings.INSECURE_SCARED})     
            );
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.NO_NEIGHBORS, TimeDiff.VERY_BIG}),
                new Emotion(new Feelings[]{Feelings.INSECURE_SCARED})   
            );

            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.MANY_TARGETS, TimeDiff.BIG}),
                new Emotion(new Feelings[]{Feelings.EXCITED_JOYFUL})    
            );
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.MANY_TARGETS, TimeDiff.VERY_BIG}),
                new Emotion(new Feelings[]{Feelings.EXCITED_JOYFUL})     
            );

            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.MANY_NEIGHBORS, TimeDiff.BIG}),
                new Emotion(new Feelings[]{Feelings.EXCITED_JOYFUL})
            );
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.MANY_NEIGHBORS, TimeDiff.VERY_BIG}),
                new Emotion(new Feelings[]{Feelings.EXCITED_JOYFUL})  
            );

            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.MANY_NEIGHBORS, TimeDiff.MEDIUM}),
                new Emotion(new Feelings[]{Feelings.ENERGETIC_JOYFUL})
            );
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.MANY_TARGETS, TimeDiff.MEDIUM}),
                new Emotion(new Feelings[]{Feelings.ENERGETIC_JOYFUL})      
            );
            
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{RewardSize.BIG, TimeDiff.BIG}),
                new Emotion(new Feelings[]{Feelings.ENERGETIC_JOYFUL})   
            );
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{RewardSize.VERY_BIG, TimeDiff.VERY_BIG}),
                new Emotion(new Feelings[]{Feelings.ENERGETIC_JOYFUL})   
            );
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{RewardSize.BIG, TimeDiff.VERY_BIG}),
                new Emotion(new Feelings[]{Feelings.ENERGETIC_JOYFUL})    
            );
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{RewardSize.VERY_BIG, TimeDiff.BIG}),
                new Emotion(new Feelings[]{Feelings.ENERGETIC_JOYFUL})    
            );
            
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{RewardSize.SAME, TimeDiff.BIG}),
                new Emotion(new Feelings[]{Feelings.CREATIVE_JOYFUL})     
            );
            
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{RewardSize.VERY_BIG, TimeDiff.BIG}),
                new Emotion(new Feelings[]{Feelings.FAITHFUL_POWERFUL})    
            );
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.MANY_NEIGHBORS, TimeDiff.BIG}),
                new Emotion(new Feelings[]{Feelings.FAITHFUL_POWERFUL})   
            );
            
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.A_TARGET_NEARBY, TimeDiff.SMALL, Senses.FEW_NEIGHBORS}),
                new Emotion(new Feelings[]{Feelings.IMPORTANT_POWERFUL})   
            );
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.A_TARGET_NEARBY, TimeDiff.SMALL, Senses.NO_NEIGHBORS}),
                new Emotion(new Feelings[]{Feelings.IMPORTANT_POWERFUL})   
            );
            
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{TimeDiff.WITHIN_CONSTRAINT, Senses.MANY_TARGETS}),
                new Emotion(new Feelings[]{Feelings.CONTENT_PEACEFUL})     
            );
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{TimeDiff.WITHIN_CONSTRAINT, Senses.A_TARGET_NEARBY}),
                new Emotion(new Feelings[]{Feelings.CONTENT_PEACEFUL})      
            );

            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{ActionReinforcements.POS_A_LOT, ActionReinforcements.NEG_VERY_FEW}),
                new Emotion(new Feelings[]{Feelings.TRUSTING_PEACEFUL})   
            );
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{ActionReinforcements.POS_MANY, ActionReinforcements.NEG_FEW}),
                new Emotion(new Feelings[]{Feelings.TRUSTING_PEACEFUL})
            );
            
            // Used at the absolute beginning
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.NO_TARGETS, TimeDiff.VERY_SMALL}),
                new Emotion(new Feelings[]{Feelings.ENERGETIC_JOYFUL})
            );
            
            triggersToEmotionsMap.addAssociation(
                Arrays.hashCode(new Enum[]{Senses.NO_TARGETS, TimeDiff.MEDIUM}),
                new Emotion(new Feelings[]{Feelings.INSECURE_SCARED})
            );

            triggersToEmotionsMap.build();
            
        } catch (StaticMapModificationException ex) {
            Logger.getLogger(MiniBrain.class.getName()).log(Level.SEVERE, ex.getMessage(), ex);
        }
   
    }
    
    /**
     * Maps triggers generated from awareness of personal resource levels to 
     * feelings that need to be excited when these triggers are active.
     */
    private void buildAwarenessToEmotionsMap() {
        
        awarenessToEmotionsMap = new StaticAssocFunct();
        
        try {
            awarenessToEmotionsMap.addAssociation(Arrays.hashCode(new Enum[]{BatteryLevel.BELOW_20_PERCENT}),
                    new Emotion(new Feelings[]{Feelings.INSECURE_SCARED}));
            
            awarenessToEmotionsMap.addAssociation(Arrays.hashCode(new Enum[]{BatteryLevel.ABOVE_80_PERCENT}),
                    new Emotion(new Feelings[]{Feelings.FAITHFUL_POWERFUL}));
            
            awarenessToEmotionsMap.addAssociation(Arrays.hashCode(new Enum[]{TimeDiff.SMALL}),
                    new Emotion(new Feelings[]{Feelings.ENERGETIC_JOYFUL, Feelings.EXCITED_JOYFUL}));
            
            awarenessToEmotionsMap.addAssociation(Arrays.hashCode(new Enum[]{TimeDiff.BIG}),
                    new Emotion(new Feelings[]{Feelings.INSECURE_SCARED, Feelings.DEPRESSED_SAD}));
            
            awarenessToEmotionsMap.build();
            
        } catch (StaticMapModificationException ex) {
            Logger.getLogger(MiniBrain.class.getName()).log(Level.SEVERE, ex.getMessage(), ex);
        }
        
    }
    
    /**
     * Maps different actions to expected rewards from their execution.
     */
    private void buildActionsToRewardsMap() {
        
        actionsToExpectedRewardsMap = new StaticAssocFunct();
        
        try {
            
            actionsToExpectedRewardsMap.addAssociation(
                    ActionType.FIND_NEIGHBORS_LOCALLY.hashCode(), 
                    new Reward(RewardSpan.SHORT_TERM, ShortTermRewardType.FOUND_NEIGHBORS)
            );
            
            actionsToExpectedRewardsMap.addAssociation(
                    ActionType.FLOCK.hashCode(), 
                    new Reward(RewardSpan.SHORT_TERM, ShortTermRewardType.FLOCKED)
            );
            
            actionsToExpectedRewardsMap.addAssociation(
                    ActionType.SEARCH_TARGETS_LOCALLY.hashCode(), 
                    new Reward(RewardSpan.SHORT_TERM, ShortTermRewardType.FOUND_NEXT_TARGET)
            );
            
            actionsToExpectedRewardsMap.addAssociation(
                    ActionType.SEARCH_TARGETS_GLOBALLY.hashCode(), 
                    new Reward(RewardSpan.LONG_TERM, LongTermRewardType.FOUND_TARGET_LOCATION)
            );
            
            actionsToExpectedRewardsMap.addAssociation(
                    ActionType.NOTIFY_OTHERS_ABOUT_TARGETS.hashCode(), 
                    new Reward(RewardSpan.SHORT_TERM, ShortTermRewardType.ATTRACTED_OTHER_VEHS_TO_TARG)
            );
            
            actionsToExpectedRewardsMap.addAssociation(
                    ActionType.IDENTIFY_AND_PERFORM_TASK.hashCode(), 
                    new Reward(RewardSpan.SHORT_TERM, ShortTermRewardType.IDENTIFIED_AND_PERFORMED_TASK)
            );
            
            actionsToExpectedRewardsMap.addAssociation(
                    ActionType.ACT_CREATIVELY.hashCode(), 
                    new Reward(RewardSpan.SHORT_TERM, ShortTermRewardType.ACTED_CREATIVELY)
            );
            
            actionsToExpectedRewardsMap.addAssociation(
                    ActionType.SURFACE.hashCode(), 
                    new Reward(RewardSpan.LONG_TERM, LongTermRewardType.FLOATED_TO_SURFACE)
            );
            
            actionsToExpectedRewardsMap.addAssociation(
                    ActionType.IDLE.hashCode(), 
                    new Reward(RewardSpan.SHORT_TERM, ShortTermRewardType.SAVED_ENERGY)
            );
            
            actionsToExpectedRewardsMap.addAssociation(
                    ActionType.ESCAPE_CROWD.hashCode(), 
                    new Reward(RewardSpan.SHORT_TERM, ShortTermRewardType.ESCAPED_CROWD)
            );
            
            actionsToExpectedRewardsMap.addAssociation(
                    ActionType.DECLARE_MAINTAINED_LONG_TERM_COHESION.hashCode(), 
                    new Reward(RewardSpan.LONG_TERM, LongTermRewardType.MAINTAINED_LONG_TERM_COHESION)
            );
            
            actionsToExpectedRewardsMap.addAssociation(
                    ActionType.DECLARE_MISSION_COMPLETED_WITHIN_CONSTRAINT.hashCode(), 
                    new Reward(RewardSpan.LONG_TERM, LongTermRewardType.SATISFIED_TIME_CONSTRAINT)
            );
            
            actionsToExpectedRewardsMap.addAssociation(
                    ActionType.DECLARE_PROCESSED_N_TARGETS.hashCode(), 
                    new Reward(RewardSpan.LONG_TERM, LongTermRewardType.PROCESSED_N_TARGETS)
            );
            
            actionsToExpectedRewardsMap.addAssociation(
                    ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN.hashCode(), 
                    new Reward(RewardSpan.LONG_TERM, LongTermRewardType.FOUND_TARGET_LOCATION)
            );

            // Make sure all action types have been added!

            actionsToExpectedRewardsMap.build();
            
        } catch (StaticMapModificationException ex) {
            Logger.getLogger(MiniBrain.class.getName()).log(Level.SEVERE, ex.getMessage(), ex);
        }

    }
    
//    @Override
//    public MoodSignal getMood() {
//        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
//    }

    public MiniEmotionalArea getEmotionalArea() {
        return emotionalArea;
    }

    public MiniSensoryArea getSensoryArea() {
        return sensoryArea;
    }
    
    public MiniHigherMentalFunc getHigherMentalFunc() {
        return higherMentalFunc;
    }
    
    public MiniMotorFuncArea getMotorFuncArea() {
        return motorFuncArea;
    }

    public Situation getSituation() {
        return situation;
    }
    
    public UWVehicleControl getAgentController() {
        return auvController;
    }

    public StaticAssocFunct getTriggersToEmotionsMap() {
        return triggersToEmotionsMap;
    }

    public StaticAssocFunct getawarenessToEmotionsMap() {
        return awarenessToEmotionsMap;
    }

    public StaticAssocFunct getActionsToExpectedRewardsMap() {
        return actionsToExpectedRewardsMap;
    }
    
    
}

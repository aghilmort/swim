package swim.sim.uwswarm.intelli.brain.struct.artif;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;
import swim.exception.CallOrderException;
import swim.sim.uwswarm.UWVehicleControl;
import swim.sim.uwswarm.intelli.brain.MiniBrain;
import swim.sim.uwswarm.intelli.brain.constr.Action;
import swim.sim.uwswarm.intelli.brain.constr.Behavior;
import swim.sim.uwswarm.intelli.brain.constr.LongTermRewardType;
import swim.sim.uwswarm.intelli.brain.constr.Reward;
import swim.sim.uwswarm.intelli.brain.constr.RewardSpan;
import swim.sim.uwswarm.intelli.brain.constr.ShortTermRewardType;
import swim.sim.uwswarm.intelli.brain.constr.Situation;
import swim.sim.uwswarm.intelli.brain.constr.TriggerOccurrence;
import swim.sim.uwswarm.intelli.brain.struct.bio.MiniHigherMentalFunc;

/**
 *
 * @author Sherif
 */
public class MiniBehaviorAssessor extends BehaviorAssessor {

    // lt: long-term
    // st: short-term
    
    private TriggerRegistry triggerRegistry;

    private HashMap<Behavior, Float> shortTermedBehaviorScores,
                                     longTermedBehaviorScores;
    
    // This was added to enable getting the behavior with the highest score (from 
    // the above maps) among the behaviors formed by a specific sitauion and any 
    // action. The action of that highest score behavior can then be retrieved and 
    // is the most rewarded action for that sitauion.
    private HashMap<Situation, ArrayList<Behavior>> situationToBehaviorsMap;
    
    private HashMap<Behavior, Situation> ltBehaviorSituations;
    
    private MiniBrain brain;
    private UWVehicleControl agentControl;
    
    public MiniBehaviorAssessor(MiniBrain brain) {
        
        this.brain = brain;
        this.triggerRegistry = new TriggerRegistry();
        this.agentControl = brain.getAgentController();
        
        shortTermedBehaviorScores = new HashMap<Behavior, Float>();
        longTermedBehaviorScores = new HashMap<Behavior, Float>();
        
        situationToBehaviorsMap = new HashMap<Situation, ArrayList<Behavior>>();
        ltBehaviorSituations = new HashMap<Behavior, Situation>();
    }

    public boolean tryAssessBehavior(Situation situation, Behavior behavior, RewardSpan rewardSpan) {
        
        // Check if there are any triggers being tracked for this behavior
        // (check is done based on time at which triggers were generated)
        if( !triggerRegistry.preBehaviorTriggersTrackedFor(situation.getTimeOfTriggers()) ) {
            // Not tracked ... Track them
            triggerRegistry.setPreBehaviorTriggers(situation.getTimeOfTriggers(), situation.getTriggers());
        } 
            
        // Is action of the behavior the same as the active motion?
        // - Should match for short term behaviors
        // - Allowed (actually expected) to differ for long term behaviors
        if( behavior.getSelectedAction().getType().equals(brain.getMotorFuncArea().getActiveAction().getType()) ) {

            // Check active motion type and if behavior has completed:
            // ALGORITHM: 
            //   Completed vs. active motion check is done by comparing algorithm 
            //   keys
            // SEQUENCE:
            //   Composite turn sequence execution is checked for completion
            if( (agentControl.getActiveMotionType().equals(MotionType.ALGORITHM) && 
                    agentControl.getActiveMotionAlgKey() == agentControl.getCompletedMotionAlgKey()) 
                    ||
                (agentControl.getActiveMotionType().equals(MotionType.SEQUENCE) && 
                    agentControl.compTurnSeqMotionEnded())    
              ) {
                
                // Motion ended ... evaluate reward
                assessBehavAgainstCurrentTriggs(situation, behavior, rewardSpan);

                // Add the behavior to the list of behaviors that can be used
                // in that situation

                // =========================================================
                // IMPORTANT question here:
                // =========================================================
                // Should I use a situation consisting only of the highest 
                // priority trigger (as below) or all original triggers?
                // =========================================================
                // ArrayList<Trigger> triggerArr = new ArrayList<Trigger>(1);
                // triggerArr.add(behavior.getHighestPriTrigger());
                // Situation situationOfHighestPriTrig 
                // = new Situation(triggerArr, behavior.getDominantEmotion());
                // Leave it for experiments!
                // =========================================================

                ArrayList<Behavior> behaviorsList = situationToBehaviorsMap.get(situation);
                if( behaviorsList == null ) {
                    behaviorsList = new ArrayList<Behavior>(); 
                }
                behaviorsList.add(behavior);
                situationToBehaviorsMap.put(situation, behaviorsList);

                return true;

            } else {
                System.out.println("Action has not completed yet");  
                return false;
            }

        } else {
            
            if( rewardSpan.equals(RewardSpan.SHORT_TERM) ) {
                System.out.println("Action of assessed behavior is different from that of active action ... check how?!");
                return false;
            } else {    
                // Long term behaviors can be different from active ones (because 
                // of temporary short term behaviors that intervene)
                
                // Assess/reassess the behavior
                assessBehavAgainstCurrentTriggs(situation, behavior, rewardSpan);
                
                // Add the behavior to the list of behaviors that can be used
                // in that situation
                ArrayList<Behavior> behaviorsList = situationToBehaviorsMap.get(situation);
                if( behaviorsList == null ) {
                    behaviorsList = new ArrayList<Behavior>(); 
                }
                behaviorsList.add(behavior);
                situationToBehaviorsMap.put(situation, behaviorsList);

                return true;
            }
        }

    }
    
    private void assessBehavAgainstCurrentTriggs(Situation situation, Behavior behavior, RewardSpan rewardSpan) {
        
        // Get information about expected reward from action
        Action actionOfBehav = behavior.getSelectedAction();
        Reward expectedReward = (Reward)brain.getActionsToExpectedRewardsMap().getAssociation(actionOfBehav.getType().hashCode());

        // Get current triggers to search for the highest priority 
        // trigger (the one we are trying to fulfill using this 
        // behavior) in them
        ArrayList<Trigger> currentTriggers 
                = brain.getHigherMentalFunc().getCurrentSituation().getTriggers();

        float score = 0;

        // Check trigger fulfillment
        if( currentTriggers.contains(behavior.getHighestPriTrigger()) ) {
            // Trigger not fulfilled: it has been re-generated, meaning it has not been fulfilled yet
            // -ve reward
            if( rewardSpan.equals(RewardSpan.SHORT_TERM) ) {
                score = -((ShortTermRewardType)expectedReward.getType()).value();
                // Add the behavior and its score to the registry
                shortTermedBehaviorScores.put(behavior, score);
            } else {
                score = -((LongTermRewardType)expectedReward.getType()).value();
                // Add the behavior and its score to the registry
                longTermedBehaviorScores.put(behavior, score); 
            }
        } else {
             // Trigger fulfilled: it has not been generated again, meaning it is no longer there
            // +ve reward
            if( rewardSpan.equals(RewardSpan.SHORT_TERM) ) {
                score = ((ShortTermRewardType)expectedReward.getType()).value();
                // Add the behavior and its score to the registry
                shortTermedBehaviorScores.put(behavior, score);
            } else {
                score = ((LongTermRewardType)expectedReward.getType()).value();
                // Add the behavior and its score to the registry
                longTermedBehaviorScores.put(behavior, score); 
            }  
        }
        
        if( rewardSpan.equals(RewardSpan.LONG_TERM) && 
                !ltBehaviorSituations.containsKey(behavior) ) {
            ltBehaviorSituations.put(behavior, situation);
        }
        
    }
    
    public void reassessAllLongTermBehaviors() {
        
        Iterator itr = ltBehaviorSituations.keySet().iterator();
        Behavior currBehav;
        Situation situation;
        RewardSpan rewardSpan;
        Action action;
        
        while( itr.hasNext() ) {
            currBehav = (Behavior)itr.next();
            situation = ltBehaviorSituations.get(currBehav);
            action = currBehav.getSelectedAction();
            rewardSpan = ((Reward)brain.getActionsToExpectedRewardsMap().getAssociation(action.getType().hashCode())).getSpan();
            
            assessBehavAgainstCurrentTriggs(situation, currBehav, rewardSpan);
        }
    }
    
    public ArrayList<Behavior> getBehaviorsList( Situation situation ) {
        return situationToBehaviorsMap.get(situation);
    }
    
    @Override
    protected void rewardPenalizeBehavior() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }
    
    @Override
    public boolean behaviorAssessed(Behavior behavior) {
        if( isLongTermed(behavior) ) {
           return longTermedBehaviorScores.containsKey(behavior); 
        } else {
           return shortTermedBehaviorScores.containsKey(behavior);  
        }
    }

    @Override
    public float getScore(Behavior behavior) {
        if( isLongTermed(behavior) ) {
           return longTermedBehaviorScores.get(behavior); 
        } else {
           return shortTermedBehaviorScores.get(behavior);  
        }
    }
    
    public boolean isLongTermed(Behavior behavior) {
        return ((Reward)brain.getActionsToExpectedRewardsMap()
                .getAssociation(behavior.getSelectedAction().getType().hashCode()))
                .getSpan().equals(RewardSpan.LONG_TERM);
    }
    
    public boolean isShortTermed(Behavior behavior) {
        return !isLongTermed(behavior);
    }
    
    public Trigger getHighestPriorityTrigger( ArrayList<Trigger> triggers ) {
        Trigger highestPriTrigg = null;
        Integer highestPriority = 0;
        for( Trigger trigg : triggers ) {
            highestPriority = Math.max(highestPriority, trigg.getPriority());
            if( highestPriority == trigg.getPriority() ) highestPriTrigg = trigg;
        }
        
        return highestPriTrigg;
    }
    
    @Override
    public void fetchPreviousBehavior() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public void fetchTargetShortTermReward() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public void fetchTargetLongTermReward() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public void fetchActualReward() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }
  
}

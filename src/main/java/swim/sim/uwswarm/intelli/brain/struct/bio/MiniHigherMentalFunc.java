/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.struct.bio;

import com.jme3.math.FastMath;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import swim.algorithm.selforg.InitSelfOrgAlgorithms;
import swim.algorithm.surfacing.SurfacingAlgorithms;
import swim.algorithm.taskalloc.TaskAllocAlgorithms;
import swim.core.MissionStage;
import swim.sim.uwswarm.UWVehicleControl;
import swim.sim.uwswarm.intelli.brain.MiniBrain;
import swim.sim.uwswarm.intelli.brain.constr.Action;
import swim.sim.uwswarm.intelli.brain.constr.ActionType;
import swim.sim.uwswarm.intelli.brain.constr.Behavior;
import swim.sim.uwswarm.intelli.brain.constr.Emotion;
import swim.sim.uwswarm.intelli.brain.constr.Reward;
import swim.sim.uwswarm.intelli.brain.constr.RewardSpan;
import swim.sim.uwswarm.intelli.brain.constr.Situation;
import swim.sim.uwswarm.intelli.brain.funct.DynamicAssocFunct;
import swim.sim.uwswarm.intelli.brain.struct.artif.Duration;
import swim.sim.uwswarm.intelli.brain.struct.artif.MiniBehaviorAssessor;
import swim.sim.uwswarm.intelli.brain.struct.artif.MiniResourceMonitor;
import swim.sim.uwswarm.intelli.brain.struct.artif.ResidualEnergyLevel;
import swim.sim.uwswarm.intelli.brain.struct.artif.Trigger;
import swim.sim.uwswarm.intelli.resource.ElapsedTime;
import swim.sim.uwswarm.intelli.resource.TimeDiff;
import swim.sim.uwswarm.intelli.sense.Senses;
import swim.util.Pair;

/**
 *
 * @author Sherif
 */
public class MiniHigherMentalFunc extends HigherMentalFunc {
    
    // =========================================================================
    // Constants
    // =========================================================================
    private final int START_MISSION_STAGE = 3;
    
    private static final int NUM_SUB_AREAS = 1;
    private final int MIN_ACCEP_BEHAVIOR_SCORE = 3;
    private final float ACTION_ACTIVATION_EMOTIONAL_STRENGTH = 4;
    private final int LONG_TERM_BEHAV_REASSESS_PERIOD = 500;
    private final int NUM_PREDEFINED_TERIGGERS = 16;
    // =========================================================================
    // Variables
    // =========================================================================
    private MissionStage activeMissionStage;  
    private Enum activeMissionLvlAction;
    private HashMap<Integer, Pair<MissionStage,Enum>> missionStageActionPairs;
    
    // Artificial Higher Mental Function (HMF) units
    private MiniResourceMonitor resourceMonitor;
    private MiniBehaviorAssessor behaviorAssessor;
    
    private MiniBrain brain;
    private MiniSensoryArea sensoryArea;
    private MiniEmotionalArea emotionalArea;
    
    private Situation situation, situationOfActiveBehav, prevSituation;
    
    private DynamicAssocFunct situationsToActionsMap;
    
    private Action selectedAction;
    
    private Trigger highestPriorityTrigger;
    private Emotion dominantEmotion;
    
    private ArrayList<Trigger> triggers;
    
    private UWVehicleControl agentControl;
    
    private Behavior selectedBehavior, activeBehavior;
    
    private boolean assessingShortTermBehavior = false,
                    noBehaviorEverSelected = true,
                    activateInitialBehavior = true;
    
    private int timeLTBehavLastAssessed = 0;    // LT: Long-term
    
    private int activeMissionIndex = START_MISSION_STAGE;
    
    
    // Temporary
    private int checkCounter = 1;
    // =========================================================================
    
    public MiniHigherMentalFunc(MiniBrain brain) {
        super(NUM_SUB_AREAS);
        
        this.brain = brain;
        this.sensoryArea = brain.getSensoryArea();
        this.emotionalArea = brain.getEmotionalArea();
        this.agentControl = brain.getAgentController();
                
        missionStageActionPairs = new HashMap();
        
        // Instantiate artificial HMF units
        resourceMonitor = new MiniResourceMonitor(agentControl);
        behaviorAssessor = new MiniBehaviorAssessor(brain);
        
        specifyMissionStageActionPairs();
    }
    
    private void specifyMissionStageActionPairs() {
        missionStageActionPairs.put(1, new Pair(MissionStage.INITIAL_SELF_ORGANIZATION, InitSelfOrgAlgorithms.MAG_NORTH_BASED));
        missionStageActionPairs.put(2, new Pair(MissionStage.TARGET_SEARCH, UWVehicleControl.getActiveSearchAlgorithm()));
        missionStageActionPairs.put(3, new Pair(MissionStage.TASK_ALLOCATION, TaskAllocAlgorithms.EXP_RESP_THRESHOLD));
        missionStageActionPairs.put(4, new Pair(MissionStage.SOURCE_SEARCH, SurfacingAlgorithms.SSRF));
    }

    @Override
    protected void function() {
        
        boolean activeMissionCompleted = 
                (activeMissionStage != null && agentControl.isMissionStageCompleted(activeMissionStage));
        
        // The very beginning
        if( noBehaviorEverSelected || activeMissionCompleted ) {
            
            System.out.println("Entered the check in HMF: (" + checkCounter + ")");
            ++checkCounter;
            
            if( activeMissionCompleted ) {
                ++activeMissionIndex;
                if(activeMissionIndex > missionStageActionPairs.size()) return; 
            }
            
            activeMissionStage = missionStageActionPairs.get(activeMissionIndex).key;
            activeMissionLvlAction = missionStageActionPairs.get(activeMissionIndex).val;
            
            agentControl.setActiveMissionStage(activeMissionStage);
            agentControl.setActiveMissionLvlAction(activeMissionLvlAction);
            
            agentControl.initActiveMissionAction();
            agentControl.startActiveMissionAction();
            
            activeBehavior = new Behavior(
                    behaviorAssessor.getHighestPriorityTrigger(triggers), 
                    emotionalArea.getDominantEmotion(), 
                    new Action(ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN)     // Not fully addressed yet!
            );
            
            selectedAction = activeBehavior.getSelectedAction();
            situationOfActiveBehav = situation;
            
            noBehaviorEverSelected = false;
        }
        
        // =====================================================================
        // 1) Learning based on behavior assessment 
        // (if an active "short-term" behavior just completed)
        // =====================================================================
        
        // Idea:
        // Two ways to check rewards:
        // 1) Short-termed rewards: wait until action is completed then assess
        // 2) Long-termed rewards: assess at (temporary) action switching points,
        //    penalize if not much progress has been made. If, at any point, they
        //    have been severly penalized, relinquish them.
        // There should be an efficient mechanism for switching from and back to  
        // long-term rewarded behaviors
        
        if( assessingShortTermBehavior ) {
            
            // Asks behavior assessor to try assessing the short term behavior
            // try = if behavior has completed: assess it, 
            // otherwise: wait .. don't generate and perform another behavior
            boolean successfullyAssessed = 
                    behaviorAssessor.tryAssessBehavior(situationOfActiveBehav, activeBehavior, RewardSpan.SHORT_TERM);
            
            // We return because as long as there is a behavior taking place,
            // excitations to the brian should only affect emotional area and 
            // no other behaviors should be performed unless the current behavior 
            // terminated and assessed.
            if( !successfullyAssessed ) {
                return;
            }
        }
        
        // =====================================================================
        // 2) Making a decision based on resources, logic, emotions, and rewards
        // =====================================================================
        // Make a decision on which behavior to perform 
        // (stores result in: selectedAction and selectedBehavior)
        decideBehavior();
        // =====================================================================
        // 3) Learning based on behavior assessment 
        // (if the active "long-term" behavior is to be paused)
        // =====================================================================
        Trigger highPriTriggOfActBehav = behaviorAssessor.getHighestPriorityTrigger(situationOfActiveBehav.getTriggers());
        
        int timeToReachTriggDeadline = Math.abs(highPriTriggOfActBehav.getDeadline() - agentControl.getCurrentTick());
        
        if(
            (
                !selectedAction.getType().equals(activeBehavior.getSelectedAction().getType()) || 
                timeToReachTriggDeadline < 10 ||
                agentControl.getElapsedMissionIntTime() - timeLTBehavLastAssessed >= LONG_TERM_BEHAV_REASSESS_PERIOD
            )
            && behaviorAssessor.isLongTermed(activeBehavior) ) {
            
            System.out.println("Assessing long term behavior");
            behaviorAssessor.tryAssessBehavior(situationOfActiveBehav, activeBehavior, RewardSpan.LONG_TERM);
            
            // Reassess all other long-term behaviors
            behaviorAssessor.reassessAllLongTermBehaviors();
            
            // Store reassessment time
            timeLTBehavLastAssessed = agentControl.getCurrentTick();
            
        }
        
        
//        System.out.println("Action:" + selectedBehavior.getSelectedAction().getType());
//        for( Enum signal : selectedBehavior.getHighestPriTrigger().getSignals() ) {
//            System.out.println("Sense:" + signal.name());
//        }
//        if( selectedBehavior.getDominantEmotion().getFeelingsList() != null ) {
//            for( Enum feeling : selectedBehavior.getDominantEmotion().getFeelingsList() ) {
//                System.out.println("Feeling:" + feeling.name());
//            }
//        }
        
        
        // ============================================================
        // 4) Performing the bahavior
        // ============================================================
        // Set the active behavior as the selected one
        
        if( selectedBehavior != null ) {
            System.out.println("===============================================");
            System.out.println("Action ("+agentControl.getVehicleName() + "): "+ selectedBehavior.getSelectedAction().getType());
            for(Enum signal : selectedBehavior.getHighestPriTrigger().getSignals()) {
                System.out.println("Trigger ("+agentControl.getVehicleName() + "): "+ signal);
            }
            for(Feelings feeling : selectedBehavior.getDominantEmotion().getFeelingsList()) {
                System.out.println("Feeling (Selected behavior): " + feeling);
            }
            System.out.println("Strength of Dominant Emotion (Selected behavior) ("+agentControl.getVehicleName() + "): "+ selectedBehavior.getDominantEmotion().getStrength());
            System.out.println("===============================================");
        }

        boolean behaviorActivationCond = ( !selectedAction.getType().equals(activeBehavior.getSelectedAction().getType()) && 
               selectedBehavior.getDominantEmotion().getStrength() >= ACTION_ACTIVATION_EMOTIONAL_STRENGTH );
        
        if( behaviorActivationCond ) {
            activeBehavior = selectedBehavior;
        }
        situationOfActiveBehav = situation;
        assessingShortTermBehavior = behaviorAssessor.isShortTermed(activeBehavior);
        
        // Excite motor function area to actually perform the behavior/action
        //if( behaviorActivationCond || activateInitialBehavior ) {
            fire();
        //    activateInitialBehavior = false;
        //}
            
        prevSituation = situation;
    }

    @Override
    public void excite(ExcitationSignal signal) {
        
        triggers = ((HMFExcitationSignal)signal).extractStimuli();
        dominantEmotion = emotionalArea.getDominantEmotion();
        
        if( triggers.isEmpty() && prevSituation != null ) {
            situation = prevSituation;
        } else {
            situation = new Situation(triggers, dominantEmotion);
        }

        this.function();
    }
    
    public Action extractAction() {
        return selectedAction;
    }
    
    public Situation getCurrentSituation() {
        return situation;
    }

    @Override
    protected void fire() {
        // Build action signal based on selected action
        ActionSignal actionSignal = new ActionSignal(selectedAction);
        brain.getMotorFuncArea().excite(actionSignal);
    }
    
    /**
     * Higher Mental Function (HMF) works as follows ...
     * <br /><br />
     *
     * HMF LOGIC:
     * <br /><br />
     * Before anything else, if resources (battery level, elapsed mission time)
     * are low/high, respectively, do one of the three following cases (we can 
     * call this process as the "emergency plan"):
     * <br />
     * <ol>
     * <li>If battery level is LOW or VERY_LOW:
     * <ul>
     *      <li>Surface immediately, regardless of elapsed mission time</li>
     * </ul>
     * </li>
     * <li>If elapsed mission time EXCEEDED_CONSTRAINT:
     * <ul>
     *       <li>Surface immediately</li>
     * </ul>
     * </li>
     * <li>If elapsed mission time is HIGH or VERY_HIGH:
     * <br />
     *    (we don't know which is better here: following predefined global
     *     mission-stage plan, predefined behaviors based on active triggers,
     *     or being creative). Therefore, the better option is:
     * <ul>
     *       <li>If neighbors are around, KEEP_CURRENT_ACTION</li>
     *       <li>If no neighbors are around, act based on emotions and rewards  
     *          for active triggers</li>
     * </ul>
     * </li>
     * </ol>
     * If all the above doesn't apply, check emotional effect:
     * <br /><br />
     * EMOTIONAL EFFECT:
     * <ol>
     * <li>If emotions are "intense", i.e. there is a dominant brain emotion 
     *    that exceeded a threshold:
     * <ul>
     *    <li>Use the highest rewarded action from the candidate actions 
     *        available for that emotion</li>
     * </ul>
     * </li>
     * <li>If there is no dominant emotion, execute the current mission 
     *    plan-based action (FLOLLOW_PREDEF_MISSION_STAGE_PLAN)</li>
     * </ol>
     */
    private void decideBehavior() {

        // Check battery level
        ResidualEnergyLevel batteryLevel = resourceMonitor.getResidualEnergy();
        
        if( batteryLevel.equals(ResidualEnergyLevel.LOW) || 
                batteryLevel.equals(ResidualEnergyLevel.VERY_LOW) ) {
            selectedAction = new Action(ActionType.SURFACE);
            
            selectedBehavior = new Behavior(
                highestPriorityTrigger, 
                emotionalArea.getDominantEmotion(), 
                selectedAction
            );
            
            return;   
        }
        
        // Check elapsed mission time
        ElapsedTime elapsedMissionTime = brain.getAgentController().getElapsedMissionTime();
        
        if( elapsedMissionTime.equals(ElapsedTime.EXCEEDED_CONSTRAINT) ) {
            selectedAction = new Action(ActionType.SURFACE);
            
            selectedBehavior = new Behavior(
                highestPriorityTrigger, 
                emotionalArea.getDominantEmotion(), 
                selectedAction
            );
            
            return;   
        }
        
        if( elapsedMissionTime.equals(ElapsedTime.LONG) || 
                elapsedMissionTime.equals(ElapsedTime.VERY_LONG)) {
            
            boolean neighborsAround = brain.getAgentController().neighborsAround();
            
            if( neighborsAround ) {
                selectedAction = new Action(ActionType.KEEP_CURRENT_ACTION);
            } else {
                consultTriggersEmotionsRewards(); 
            }
            
            if( selectedAction != null ) {
                
                selectedBehavior = new Behavior(
                    highestPriorityTrigger, 
                    emotionalArea.getDominantEmotion(), 
                    selectedAction
                );
                
                return;
            }
        }
        
        // None of the above worked ...
        consultTriggersEmotionsRewards();
        
        // No dominant emotion, execute the current mission plan-based action
        if( selectedAction == null || selectedBehavior == null ) {
            selectedAction = new Action(ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN);
            
            selectedBehavior = new Behavior(
                highestPriorityTrigger, 
                emotionalArea.getDominantEmotion(), 
                selectedAction
            );
        }
        
    }
    
    private void consultTriggersEmotionsRewards() {
        
        situationsToActionsMap = new DynamicAssocFunct();   
        
        int highestPriority = NUM_PREDEFINED_TERIGGERS;   // because we have 15 predefined triggers with priorities [0, 15]
            
        //  ===============================================================
        // | VERY IMPORTANT!!!                                             |
        // |===============================================================|
        // | Base the decision not only on emotions, but on other aspects  |
        // | like: logic (predefined goals and behaviors), resources,      |
        // | learning (based on rewards).                                  |
        //  ===============================================================
        
        ActionType[] candidateActions;
        
        // Consider the case where there are no active triggers!
        
        
        // IMPORTANT: I think there is an error here: should min be max? or priority is defined differently?
        
        for( Trigger trigger : triggers ) {
            highestPriority = Math.min(highestPriority, trigger.getPriority());
            if( highestPriority == trigger.getPriority() ) {
                highestPriorityTrigger = trigger;
            }
        }

        // Keep in mind that triggers are static (they are not dynamically 
        // generated; the ones defined at compile time are the ones used 
        // all the time). Triggered emotions, on the other hand, can be 
        // different from time to time as current (overall) emotional state 
        // of mind can be different each time the dominant emotion is checked.

        if( Arrays.hashCode(highestPriorityTrigger.getSignals()) == Arrays.hashCode(new Enum[]{Senses.MANY_NEIGHBORS, TimeDiff.BIG}) ) {

            // None negotiable action ... purely logical ... 
            // agent has to flock to avoid being lost again
            selectedAction = new Action(ActionType.FLOCK);
            
            selectedBehavior = new Behavior(
                highestPriorityTrigger, 
                emotionalArea.getDominantEmotion(), 
                selectedAction
            );

        } else if( Arrays.hashCode(highestPriorityTrigger.getSignals()) == Arrays.hashCode((new Enum[]{Senses.FEW_NEIGHBORS, TimeDiff.BIG})) ) {

            // Actions that have potential to achieve a gain
            candidateActions = new ActionType[]{ActionType.FLOCK, ActionType.ACT_CREATIVELY, ActionType.FIND_NEIGHBORS_GLOBALLY, 
                ActionType.KEEP_CURRENT_ACTION, ActionType.THE_MOST_REWARDED, ActionType.FLOCK_THEN_LEAD};

            // =================================================================
            // IMPORTANT!!!
            // =================================================================
            // For now, we will consider single feelings only as dominant 
            // emotions. In the future, pairs may be added.
            //
            // Also, I have combined some triggers, which defeats the purpose of
            // their distiction in the first place. This was done for time  
            // limits and can be done later.
            // =================================================================
            // =================================================================
            
            // Emotions affect selected acion ...
            if( dominantEmotion.contains(Feelings.LONELY_SAD) ) {

                useActionForFeelingsOrTryAnother(
                        ActionType.FLOCK, 
                        new Feelings[]{Feelings.LONELY_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.BORED_SAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.ACT_CREATIVELY, 
                        new Feelings[]{Feelings.BORED_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.DEPRESSED_SAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.ACT_CREATIVELY, 
                        new Feelings[]{Feelings.DEPRESSED_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.ANGREY_MAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FIND_NEIGHBORS_GLOBALLY, 
                        new Feelings[]{Feelings.ANGREY_MAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.SELFISH_MAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.KEEP_CURRENT_ACTION, 
                        new Feelings[]{Feelings.SELFISH_MAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.SUBMISSIVE_SCARED) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FLOCK, 
                        new Feelings[]{Feelings.SUBMISSIVE_SCARED}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.INSECURE_SCARED) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.THE_MOST_REWARDED, 
                        new Feelings[]{Feelings.INSECURE_SCARED}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.EXCITED_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.ACT_CREATIVELY, 
                        new Feelings[]{Feelings.EXCITED_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.ENERGETIC_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FIND_NEIGHBORS_GLOBALLY, 
                        new Feelings[]{Feelings.ENERGETIC_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.CREATIVE_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.ACT_CREATIVELY, 
                        new Feelings[]{Feelings.CREATIVE_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.FAITHFUL_POWERFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.KEEP_CURRENT_ACTION, 
                        new Feelings[]{Feelings.FAITHFUL_POWERFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.IMPORTANT_POWERFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FLOCK_THEN_LEAD, 
                        new Feelings[]{Feelings.IMPORTANT_POWERFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.CONTENT_PEACEFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FLOCK, 
                        new Feelings[]{Feelings.CONTENT_PEACEFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.TRUSTING_PEACEFUL) ) {
                useActionForFeelingsOrTryAnother(
                        ActionType.FLOCK, 
                        new Feelings[]{Feelings.TRUSTING_PEACEFUL}, 
                        candidateActions, highestPriorityTrigger);
            }                     

        } else if( Arrays.hashCode(highestPriorityTrigger.getSignals()) == Arrays.hashCode((new Enum[]{Senses.ONE_NEIGHBOR, TimeDiff.BIG})) ) {

            // Actions that have potential to achieve a gain
            candidateActions = new ActionType[]{ActionType.FLOCK, ActionType.KEEP_CURRENT_ACTION, ActionType.THE_MOST_REWARDED, 
                ActionType.FLOCK_THEN_LEAD, ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN, ActionType.ACT_CREATIVELY};
            
            // Emotions affect selected acion ...
            if( dominantEmotion.contains(Feelings.LONELY_SAD) ) {

                useActionForFeelingsOrTryAnother(
                        ActionType.FLOCK, 
                        new Feelings[]{Feelings.LONELY_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.BORED_SAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FLOCK, 
                        new Feelings[]{Feelings.BORED_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.DEPRESSED_SAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.THE_MOST_REWARDED, 
                        new Feelings[]{Feelings.DEPRESSED_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.ANGREY_MAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.KEEP_CURRENT_ACTION, 
                        new Feelings[]{Feelings.ANGREY_MAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.SELFISH_MAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.KEEP_CURRENT_ACTION, 
                        new Feelings[]{Feelings.SELFISH_MAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.SUBMISSIVE_SCARED) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FLOCK, 
                        new Feelings[]{Feelings.SUBMISSIVE_SCARED}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.INSECURE_SCARED) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.THE_MOST_REWARDED, 
                        new Feelings[]{Feelings.INSECURE_SCARED}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.EXCITED_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FLOCK_THEN_LEAD, 
                        new Feelings[]{Feelings.EXCITED_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.ENERGETIC_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FLOCK_THEN_LEAD, 
                        new Feelings[]{Feelings.ENERGETIC_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.CREATIVE_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.ACT_CREATIVELY, 
                        new Feelings[]{Feelings.CREATIVE_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.FAITHFUL_POWERFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FLOCK_THEN_LEAD, 
                        new Feelings[]{Feelings.FAITHFUL_POWERFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.IMPORTANT_POWERFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FLOCK_THEN_LEAD, 
                        new Feelings[]{Feelings.IMPORTANT_POWERFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.CONTENT_PEACEFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN, 
                        new Feelings[]{Feelings.CONTENT_PEACEFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.TRUSTING_PEACEFUL) ) {
                useActionForFeelingsOrTryAnother(
                        ActionType.FLOCK, 
                        new Feelings[]{Feelings.TRUSTING_PEACEFUL}, 
                        candidateActions, highestPriorityTrigger);
            }  
            
        } else if( Arrays.hashCode(highestPriorityTrigger.getSignals()) == Arrays.hashCode((new Enum[]{Senses.NO_NEIGHBORS, TimeDiff.BIG})) ) {
            
            // Actions that have potential to achieve a gain
            candidateActions = new ActionType[]{ActionType.ACT_CREATIVELY, ActionType.FIND_NEIGHBORS_GLOBALLY, 
                ActionType.FIND_NEIGHBORS_LOCALLY, ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN, ActionType.KEEP_CURRENT_ACTION,
                ActionType.KEEP_CURRENT_PLACE, ActionType.SCURRY_RANDOMLY, ActionType.SEARCH_TARGETS_GLOBALLY, 
                ActionType.SEARCH_TARGETS_LOCALLY, ActionType.THE_MOST_REWARDED};
            
            // Emotions affect selected acion ...
            if( dominantEmotion.contains(Feelings.LONELY_SAD) ) {

                useActionForFeelingsOrTryAnother(
                        ActionType.ACT_CREATIVELY, 
                        new Feelings[]{Feelings.LONELY_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.BORED_SAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.KEEP_CURRENT_PLACE, 
                        new Feelings[]{Feelings.BORED_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.DEPRESSED_SAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.SCURRY_RANDOMLY, 
                        new Feelings[]{Feelings.DEPRESSED_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.ANGREY_MAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FIND_NEIGHBORS_GLOBALLY, 
                        new Feelings[]{Feelings.ANGREY_MAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.SELFISH_MAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.KEEP_CURRENT_ACTION, 
                        new Feelings[]{Feelings.SELFISH_MAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.SUBMISSIVE_SCARED) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FIND_NEIGHBORS_LOCALLY, 
                        new Feelings[]{Feelings.SUBMISSIVE_SCARED}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.INSECURE_SCARED) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.SCURRY_RANDOMLY, 
                        new Feelings[]{Feelings.INSECURE_SCARED}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.EXCITED_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.SEARCH_TARGETS_GLOBALLY, 
                        new Feelings[]{Feelings.EXCITED_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.ENERGETIC_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.THE_MOST_REWARDED, 
                        new Feelings[]{Feelings.ENERGETIC_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.CREATIVE_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.ACT_CREATIVELY, 
                        new Feelings[]{Feelings.CREATIVE_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.FAITHFUL_POWERFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN, 
                        new Feelings[]{Feelings.FAITHFUL_POWERFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.IMPORTANT_POWERFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.SEARCH_TARGETS_LOCALLY, 
                        new Feelings[]{Feelings.IMPORTANT_POWERFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.CONTENT_PEACEFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN, 
                        new Feelings[]{Feelings.CONTENT_PEACEFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.TRUSTING_PEACEFUL) ) {
                useActionForFeelingsOrTryAnother(
                        ActionType.KEEP_CURRENT_ACTION, 
                        new Feelings[]{Feelings.TRUSTING_PEACEFUL}, 
                        candidateActions, highestPriorityTrigger);
            }  

        } else if( Arrays.hashCode(highestPriorityTrigger.getSignals()) == Arrays.hashCode((new Enum[]{Senses.MANY_TARGETS, TimeDiff.VERY_BIG})) ||
                   Arrays.hashCode(highestPriorityTrigger.getSignals()) == Arrays.hashCode((new Enum[]{Senses.MANY_TARGETS, TimeDiff.BIG})) ) {
            
            // Actions that have potential to achieve a gain
            candidateActions = new ActionType[]{ActionType.CHASE_TARGETS, ActionType.IDENTIFY_AND_PERFORM_TASK, 
                ActionType.NOTIFY_OTHERS_ABOUT_TARGETS, ActionType.SCURRY_RANDOMLY, ActionType.THE_MOST_REWARDED,
                ActionType.FIND_NEIGHBORS_LOCALLY};
            
            // Emotions affect selected acion ...
            if( dominantEmotion.contains(Feelings.LONELY_SAD) ) {

                useActionForFeelingsOrTryAnother(
                        ActionType.NOTIFY_OTHERS_ABOUT_TARGETS, 
                        new Feelings[]{Feelings.LONELY_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.BORED_SAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.IDENTIFY_AND_PERFORM_TASK, 
                        new Feelings[]{Feelings.BORED_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.DEPRESSED_SAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.THE_MOST_REWARDED, 
                        new Feelings[]{Feelings.DEPRESSED_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.ANGREY_MAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.SCURRY_RANDOMLY, 
                        new Feelings[]{Feelings.ANGREY_MAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.SELFISH_MAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.CHASE_TARGETS, 
                        new Feelings[]{Feelings.SELFISH_MAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.SUBMISSIVE_SCARED) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FIND_NEIGHBORS_LOCALLY, 
                        new Feelings[]{Feelings.SUBMISSIVE_SCARED}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.INSECURE_SCARED) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.SCURRY_RANDOMLY, 
                        new Feelings[]{Feelings.INSECURE_SCARED}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.EXCITED_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.IDENTIFY_AND_PERFORM_TASK, 
                        new Feelings[]{Feelings.EXCITED_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.ENERGETIC_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.THE_MOST_REWARDED, 
                        new Feelings[]{Feelings.ENERGETIC_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.CREATIVE_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.NOTIFY_OTHERS_ABOUT_TARGETS, 
                        new Feelings[]{Feelings.CREATIVE_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.FAITHFUL_POWERFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.IDENTIFY_AND_PERFORM_TASK, 
                        new Feelings[]{Feelings.FAITHFUL_POWERFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.IMPORTANT_POWERFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.NOTIFY_OTHERS_ABOUT_TARGETS, 
                        new Feelings[]{Feelings.IMPORTANT_POWERFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.CONTENT_PEACEFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.IDENTIFY_AND_PERFORM_TASK, 
                        new Feelings[]{Feelings.CONTENT_PEACEFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.TRUSTING_PEACEFUL) ) {
                useActionForFeelingsOrTryAnother(
                        ActionType.IDENTIFY_AND_PERFORM_TASK, 
                        new Feelings[]{Feelings.TRUSTING_PEACEFUL}, 
                        candidateActions, highestPriorityTrigger);
            }  

        } else if( Arrays.hashCode(highestPriorityTrigger.getSignals()) == Arrays.hashCode((new Enum[]{Senses.A_TARGET_NEARBY, TimeDiff.VERY_BIG})) ||
                   Arrays.hashCode(highestPriorityTrigger.getSignals()) == Arrays.hashCode((new Enum[]{Senses.A_TARGET_NEARBY, TimeDiff.BIG})) ) {
            
            // Actions that have potential to achieve a gain
            candidateActions = new ActionType[]{ActionType.FIND_NEIGHBORS_LOCALLY, ActionType.IDENTIFY_AND_PERFORM_TASK,
                        ActionType.KEEP_CURRENT_PLACE, ActionType.SCURRY_RANDOMLY, ActionType.SEARCH_TARGETS_LOCALLY, 
                        ActionType.ACT_CREATIVELY};
            
            // Emotions affect selected acion ...
            if( dominantEmotion.contains(Feelings.LONELY_SAD) ) {

                useActionForFeelingsOrTryAnother(
                        ActionType.FIND_NEIGHBORS_LOCALLY, 
                        new Feelings[]{Feelings.LONELY_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.BORED_SAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.IDENTIFY_AND_PERFORM_TASK, 
                        new Feelings[]{Feelings.BORED_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.DEPRESSED_SAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.KEEP_CURRENT_PLACE, 
                        new Feelings[]{Feelings.DEPRESSED_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.ANGREY_MAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.SCURRY_RANDOMLY, 
                        new Feelings[]{Feelings.ANGREY_MAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.SELFISH_MAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.IDENTIFY_AND_PERFORM_TASK, 
                        new Feelings[]{Feelings.SELFISH_MAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.SUBMISSIVE_SCARED) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.KEEP_CURRENT_PLACE, 
                        new Feelings[]{Feelings.SUBMISSIVE_SCARED}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.INSECURE_SCARED) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.SCURRY_RANDOMLY, 
                        new Feelings[]{Feelings.INSECURE_SCARED}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.EXCITED_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.SEARCH_TARGETS_LOCALLY, 
                        new Feelings[]{Feelings.EXCITED_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.ENERGETIC_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.IDENTIFY_AND_PERFORM_TASK, 
                        new Feelings[]{Feelings.ENERGETIC_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.CREATIVE_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.ACT_CREATIVELY, 
                        new Feelings[]{Feelings.CREATIVE_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.FAITHFUL_POWERFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.IDENTIFY_AND_PERFORM_TASK, 
                        new Feelings[]{Feelings.FAITHFUL_POWERFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.IMPORTANT_POWERFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FIND_NEIGHBORS_LOCALLY, 
                        new Feelings[]{Feelings.IMPORTANT_POWERFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.CONTENT_PEACEFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.IDENTIFY_AND_PERFORM_TASK, 
                        new Feelings[]{Feelings.CONTENT_PEACEFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.TRUSTING_PEACEFUL) ) {
                useActionForFeelingsOrTryAnother(
                        ActionType.IDENTIFY_AND_PERFORM_TASK, 
                        new Feelings[]{Feelings.TRUSTING_PEACEFUL}, 
                        candidateActions, highestPriorityTrigger);
            }  

        } else if( Arrays.hashCode(highestPriorityTrigger.getSignals()) == Arrays.hashCode((new Enum[]{Senses.NO_TARGETS, TimeDiff.VERY_BIG})) ||
                   Arrays.hashCode(highestPriorityTrigger.getSignals()) == Arrays.hashCode((new Enum[]{Senses.NO_TARGETS, TimeDiff.BIG})) ) {
            
            // Actions that have potential to achieve a gain
            candidateActions = new ActionType[]{ActionType.ACT_CREATIVELY, ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN,
                        ActionType.KEEP_CURRENT_ACTION, ActionType.KEEP_CURRENT_PLACE, ActionType.SCURRY_RANDOMLY, 
                        ActionType.SEARCH_TARGETS_GLOBALLY, ActionType.SEARCH_TARGETS_LOCALLY, ActionType.THE_MOST_REWARDED,
                        ActionType.FIND_NEIGHBORS_LOCALLY};
            
            // Emotions affect selected acion ...
            if( dominantEmotion.contains(Feelings.LONELY_SAD) ) {

                useActionForFeelingsOrTryAnother(
                        ActionType.ACT_CREATIVELY, 
                        new Feelings[]{Feelings.LONELY_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.BORED_SAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.SEARCH_TARGETS_GLOBALLY, 
                        new Feelings[]{Feelings.BORED_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.DEPRESSED_SAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.KEEP_CURRENT_PLACE, 
                        new Feelings[]{Feelings.DEPRESSED_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.ANGREY_MAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.SCURRY_RANDOMLY, 
                        new Feelings[]{Feelings.ANGREY_MAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.SELFISH_MAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN, 
                        new Feelings[]{Feelings.SELFISH_MAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.SUBMISSIVE_SCARED) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.KEEP_CURRENT_ACTION, 
                        new Feelings[]{Feelings.SUBMISSIVE_SCARED}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.INSECURE_SCARED) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FIND_NEIGHBORS_LOCALLY, 
                        new Feelings[]{Feelings.INSECURE_SCARED}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.EXCITED_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.SEARCH_TARGETS_LOCALLY, 
                        new Feelings[]{Feelings.EXCITED_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.ENERGETIC_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.SEARCH_TARGETS_GLOBALLY, 
                        new Feelings[]{Feelings.ENERGETIC_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.CREATIVE_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.ACT_CREATIVELY, 
                        new Feelings[]{Feelings.CREATIVE_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.FAITHFUL_POWERFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.THE_MOST_REWARDED, 
                        new Feelings[]{Feelings.FAITHFUL_POWERFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.IMPORTANT_POWERFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN, 
                        new Feelings[]{Feelings.IMPORTANT_POWERFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.CONTENT_PEACEFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.KEEP_CURRENT_ACTION, 
                        new Feelings[]{Feelings.CONTENT_PEACEFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.TRUSTING_PEACEFUL) ) {
                useActionForFeelingsOrTryAnother(
                        ActionType.KEEP_CURRENT_ACTION, 
                        new Feelings[]{Feelings.TRUSTING_PEACEFUL}, 
                        candidateActions, highestPriorityTrigger);
            }  

        } else if( Arrays.hashCode(highestPriorityTrigger.getSignals()) == Arrays.hashCode((new Enum[]{Senses.NEIGHBOR_COLLISION, TimeDiff.SMALL})) ||
                   Arrays.hashCode(highestPriorityTrigger.getSignals()) == Arrays.hashCode((new Enum[]{Senses.NEIGHBOR_COLLISION, TimeDiff.VERY_SMALL})) ) {
            
            // Actions that have potential to achieve a gain
            candidateActions = new ActionType[]{ActionType.ESCAPE_CROWD, ActionType.FLOCK_THEN_LEAD, ActionType.KEEP_CURRENT_ACTION,
                        ActionType.THE_MOST_REWARDED, ActionType.ACT_CREATIVELY, ActionType.FLOCK};
            
            // Emotions affect selected acion ...
            if( dominantEmotion.contains(Feelings.LONELY_SAD) ) {

                useActionForFeelingsOrTryAnother(
                        ActionType.FLOCK, 
                        new Feelings[]{Feelings.LONELY_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.BORED_SAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.ACT_CREATIVELY, 
                        new Feelings[]{Feelings.BORED_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.DEPRESSED_SAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.KEEP_CURRENT_ACTION, 
                        new Feelings[]{Feelings.DEPRESSED_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.ANGREY_MAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.ESCAPE_CROWD, 
                        new Feelings[]{Feelings.ANGREY_MAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.SELFISH_MAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FLOCK_THEN_LEAD, 
                        new Feelings[]{Feelings.SELFISH_MAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.SUBMISSIVE_SCARED) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.KEEP_CURRENT_ACTION, 
                        new Feelings[]{Feelings.SUBMISSIVE_SCARED}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.INSECURE_SCARED) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FLOCK, 
                        new Feelings[]{Feelings.INSECURE_SCARED}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.EXCITED_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FLOCK_THEN_LEAD, 
                        new Feelings[]{Feelings.EXCITED_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.ENERGETIC_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.ESCAPE_CROWD, 
                        new Feelings[]{Feelings.ENERGETIC_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.CREATIVE_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.ACT_CREATIVELY, 
                        new Feelings[]{Feelings.CREATIVE_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.FAITHFUL_POWERFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.THE_MOST_REWARDED, 
                        new Feelings[]{Feelings.FAITHFUL_POWERFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.IMPORTANT_POWERFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FLOCK_THEN_LEAD, 
                        new Feelings[]{Feelings.IMPORTANT_POWERFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.CONTENT_PEACEFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.KEEP_CURRENT_ACTION, 
                        new Feelings[]{Feelings.CONTENT_PEACEFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.TRUSTING_PEACEFUL) ) {
                useActionForFeelingsOrTryAnother(
                        ActionType.FLOCK, 
                        new Feelings[]{Feelings.TRUSTING_PEACEFUL}, 
                        candidateActions, highestPriorityTrigger);
            }  

        } else if( Arrays.hashCode(highestPriorityTrigger.getSignals()) == Arrays.hashCode((new Enum[]{Senses.FEW_TARGETS, Senses.MANY_NEIGHBORS})) ) {
            
            // Actions that have potential to achieve a gain
            candidateActions = new ActionType[]{ActionType.ACT_CREATIVELY, ActionType.ESCAPE_CROWD,
                        ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN, ActionType.IDENTIFY_AND_PERFORM_TASK, 
                        ActionType.SCURRY_RANDOMLY, ActionType.SEARCH_TARGETS_LOCALLY, ActionType.THE_MOST_REWARDED};
            
            // Emotions affect selected acion ...
            if( dominantEmotion.contains(Feelings.LONELY_SAD) ) {

                useActionForFeelingsOrTryAnother(
                        ActionType.SEARCH_TARGETS_LOCALLY, 
                        new Feelings[]{Feelings.LONELY_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.BORED_SAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.IDENTIFY_AND_PERFORM_TASK, 
                        new Feelings[]{Feelings.BORED_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.DEPRESSED_SAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.THE_MOST_REWARDED, 
                        new Feelings[]{Feelings.DEPRESSED_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.ANGREY_MAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.ESCAPE_CROWD, 
                        new Feelings[]{Feelings.ANGREY_MAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.SELFISH_MAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.IDENTIFY_AND_PERFORM_TASK, 
                        new Feelings[]{Feelings.SELFISH_MAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.SUBMISSIVE_SCARED) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN, 
                        new Feelings[]{Feelings.SUBMISSIVE_SCARED}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.INSECURE_SCARED) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN, 
                        new Feelings[]{Feelings.INSECURE_SCARED}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.EXCITED_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.SEARCH_TARGETS_LOCALLY, 
                        new Feelings[]{Feelings.EXCITED_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.ENERGETIC_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.SCURRY_RANDOMLY, 
                        new Feelings[]{Feelings.ENERGETIC_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.CREATIVE_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.ACT_CREATIVELY, 
                        new Feelings[]{Feelings.CREATIVE_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.FAITHFUL_POWERFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.SEARCH_TARGETS_LOCALLY, 
                        new Feelings[]{Feelings.FAITHFUL_POWERFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.IMPORTANT_POWERFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.ACT_CREATIVELY, 
                        new Feelings[]{Feelings.IMPORTANT_POWERFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.CONTENT_PEACEFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.IDENTIFY_AND_PERFORM_TASK, 
                        new Feelings[]{Feelings.CONTENT_PEACEFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.TRUSTING_PEACEFUL) ) {
                useActionForFeelingsOrTryAnother(
                        ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN, 
                        new Feelings[]{Feelings.TRUSTING_PEACEFUL}, 
                        candidateActions, highestPriorityTrigger);
            }

        } else if( Arrays.hashCode(highestPriorityTrigger.getSignals()) == Arrays.hashCode((new Enum[]{Senses.HEADING_AWAY_FROM_NEIGHBORS})) ) {

                       // Actions that have potential to achieve a gain
            candidateActions = new ActionType[]{ActionType.CATCH_UP_WITH_NEIGHBORS, ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN,
                        ActionType.KEEP_CURRENT_ACTION, ActionType.THE_MOST_REWARDED, ActionType.ACT_CREATIVELY};
            
            // Emotions affect selected acion ...
            if( dominantEmotion.contains(Feelings.LONELY_SAD) ) {

                useActionForFeelingsOrTryAnother(
                        ActionType.CATCH_UP_WITH_NEIGHBORS, 
                        new Feelings[]{Feelings.LONELY_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.BORED_SAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.CATCH_UP_WITH_NEIGHBORS, 
                        new Feelings[]{Feelings.BORED_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.DEPRESSED_SAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.THE_MOST_REWARDED, 
                        new Feelings[]{Feelings.DEPRESSED_SAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.ANGREY_MAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.KEEP_CURRENT_ACTION, 
                        new Feelings[]{Feelings.ANGREY_MAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.SELFISH_MAD) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN, 
                        new Feelings[]{Feelings.SELFISH_MAD}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.SUBMISSIVE_SCARED) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.CATCH_UP_WITH_NEIGHBORS, 
                        new Feelings[]{Feelings.SUBMISSIVE_SCARED}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.INSECURE_SCARED) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.CATCH_UP_WITH_NEIGHBORS, 
                        new Feelings[]{Feelings.INSECURE_SCARED}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.EXCITED_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.ACT_CREATIVELY, 
                        new Feelings[]{Feelings.EXCITED_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.ENERGETIC_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.ACT_CREATIVELY, 
                        new Feelings[]{Feelings.ENERGETIC_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.CREATIVE_JOYFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.ACT_CREATIVELY, 
                        new Feelings[]{Feelings.CREATIVE_JOYFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.FAITHFUL_POWERFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN, 
                        new Feelings[]{Feelings.FAITHFUL_POWERFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.IMPORTANT_POWERFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.KEEP_CURRENT_ACTION, 
                        new Feelings[]{Feelings.IMPORTANT_POWERFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.CONTENT_PEACEFUL) ) {
                
                useActionForFeelingsOrTryAnother(
                        ActionType.FOLLOW_PREDEF_MISSION_STAGE_PLAN, 
                        new Feelings[]{Feelings.CONTENT_PEACEFUL}, 
                        candidateActions, highestPriorityTrigger);

            } else if( dominantEmotion.contains(Feelings.TRUSTING_PEACEFUL) ) {
                useActionForFeelingsOrTryAnother(
                        ActionType.CATCH_UP_WITH_NEIGHBORS, 
                        new Feelings[]{Feelings.TRUSTING_PEACEFUL}, 
                        candidateActions, highestPriorityTrigger);
            }
            
        }

    }
    
    private void useActionForFeelingsOrTryAnother(ActionType action, Feelings[] feelings, 
            ActionType[] candidateActions, Trigger highestPriorityTrigger) {
        
        boolean behaviorAssessed;
        float behaviorScore;
        
        selectedBehavior = new Behavior(
                highestPriorityTrigger, 
                new Emotion(feelings), 
                new Action(action)
        );

        // Check if behavior has been previously assessed
        behaviorAssessed = behaviorAssessor.behaviorAssessed(selectedBehavior);

        if( behaviorAssessed ) {

            // Decision can be influenced by previous reward
            behaviorScore = behaviorAssessor.getScore(selectedBehavior);

            if( behaviorScore > MIN_ACCEP_BEHAVIOR_SCORE ) {

                // Notice the Epsilon-greedy approach below (Epsilon = 0.01)
                if( FastMath.rand.nextFloat() > 0.01f  ){
                    selectedAction = new Action(action);
                } else {
                    selectedAction = new Action(selectRandActionFromCandidates(candidateActions));
                }

            } else {
                selectedAction = new Action(selectRandActionFromCandidates(candidateActions));
            }

        } else { // not previously assessed ... let's try it!
            selectedAction = new Action(action); 
        }  

        //re-define behavior if action is different
        if( !selectedAction.getType().equals(action) ) {
            selectedBehavior = new Behavior(
                highestPriorityTrigger, 
                new Emotion(feelings), 
                selectedAction
            );
        }
        
        // Set the action's start time before pushing its encapsulating behavior
        // onto the bahavior assessment queue. The start time is expected to be
        // on the next simulation tick as a FSM state change request is fulfilled
        // on the next tick.
        selectedBehavior.getSelectedAction().setStartTime(agentControl.getCurrentTick() + 1);

    }
    
    private void checkTimeDifferences() {
        
        // Loop over all tracked time variables, checking each for 
        // classification purposes
        for( Duration duration : Duration.values() ) {
            if( !duration.equals(Duration.PRESET) &&
                !duration.equals(Duration.PRESET) &&
                !duration.equals(Duration.PRESET) ) {
                
            }
        }
        
        
    }
    
    private float weightByResidualEnergy( float score, ResidualEnergyLevel level ) {

        switch(level) {
            case VERY_LOW:
                score *= 0.01;
                break;
            case LOW:
                score *= 0.25;
                break;
            case MEDIUM:
                score *= 0.5;
                break;
            case HIGH:
                score *= 0.75;
        }
        
        return score;
    }
    
    private ActionType selectRandActionFromCandidates(ActionType[] candidateActions) {
        // implement ...
        return null;
    }

    public Trigger getHighestPriorityTrigger() {
        return highestPriorityTrigger;
    }
    
    public Action getHighestRewActForCurrSituation() {
        
        // Just a wrapper because "Situation" requires ArrayList
        ArrayList<Trigger> triggArr = new ArrayList<Trigger>(1);
        triggArr.add(highestPriorityTrigger);
        
        // A situation that holds the highest priority trigger and the dominant
        // brain emotion
        Situation situat = new Situation(triggArr, dominantEmotion);
        
        // Get the list of all behaviors that include this situation
        // Remember:
        // Situation = trigger(s) + emotion
        // Behavior = situation + action
        ArrayList<Behavior> sitBehavList = behaviorAssessor.getBehaviorsList(situat);
        
        // Find the behavior with highest score and return its action
        float highestScore = 0, currentScore;
        Action action = null;
        
        if( sitBehavList != null && sitBehavList.size() > 0 ){
             for( Behavior behavior : sitBehavList ) {
                 currentScore = behaviorAssessor.getScore(behavior);
                 highestScore = Math.max(highestScore, currentScore);
                 if( highestScore == currentScore ) {
                     action = behavior.getSelectedAction();
                 }
             }
        }
        
        return action;
    }
    
    private Reward getCurrentReward() {
        // Implement ...
        return null;  
    }
    
    private Reward getPreviousReward() {
        // Implement ...
        return null; 
    }
    
    public boolean isRewardLevelPersistent() {
        // Implement ...
        return true;
    }
    
    public MissionStage getCurrentStage() {
        return activeMissionStage;
    }
    
    public MissionStage getStartMissionStage() {
        return missionStageActionPairs.get(START_MISSION_STAGE).key;
    }
    
}

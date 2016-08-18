package swim.sim.uwswarm.intelli.brain.struct.artif;

import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import java.util.ArrayList;
import swim.core.State;
import swim.physics.Collisions;
import swim.sim.uwswarm.UWVehicleControl;
import swim.sim.uwswarm.intelli.resource.ElapsedTime;
import swim.sim.uwswarm.intelli.resource.TimeDiff;
import swim.sim.uwswarm.intelli.sense.Senses;

/**
 *
 * @author Sherif
 * A trigger can be an emotion, sense, any of them over time or a combination
 * thereof
 */
public class MiniTriggerSynthesizer extends TriggerSynthesizer {

    // Constants - START
    private final int MAX_ALLOW_VEH_COLL_IN_PERIOD = 10;
    private final int MAX_ACCEP_TIME_WITHOUT_NEIGH = 50,    // ticks
                      MAX_ACCEP_SEARCH_TIME = 5000, // = max. acceptable time without finding target
                      NORMALLY_EXPECTED_NEIGH_COUNT = 3;
    
    // ETST: Expected Trigger Satisfaction Time 
    
    private final int ETST_NO_TARGS_BEGINNING_OF_TIME = MAX_ACCEP_SEARCH_TIME, // ticks
                      ETST_HEADING_AWAY_FROM_NEIGHS = 10,    
                      ETST_MANY_NEIGHS_BIG_TIME_DIFF = 10,
                      ETST_FEW_NEIGHS_BIG_TIME_DIFF = 10,
                      ETST_ONE_NEIGH_BIG_TIME_DIFF = 30,
                      ETST_NO_NEIGHS_BIG_TIME_DIFF = 500,
                      ETST_MANY_TARGS_VERY_BIG_TIME_DIFF = 10,
                      ETST_ONE_TARG_VERY_BIG_TIME_DIFF = 30,
                      ETST_NO_TARGS_VERY_BIG_TIME_DIFF = 500,
                      ETST_MANY_TARGS_BIG_TIME_DIFF = 20,
                      ETST_ONE_TARG_BIG_TIME_DIFF = 30,
                      ETST_NO_TARGS_BIG_TIME_DIFF = 600,
                      ETST_NEIGH_COLLISION_SMALL_TIME_DIFF = 20,
                      ETST_NEIGH_COLLISION_VERY_SMALL_TIME_DIFF = 10,
                      ETST_FEW_TARGS_MANY_HEIGHS = 50,
                      ETST_NO_TARGS_MEDIUM_TIME_DIFF = 1200,
                
                      PRIORITY_NO_TARGS_BEGINNING_OF_TIME = 0,
                      PRIORITY_HEADING_AWAY_FROM_NEIGHS = 1,
                      PRIORITY_MANY_NEIGHS_BIG_TIME_DIFF = 2,
                      PRIORITY_FEW_NEIGHS_BIG_TIME_DIFF = 3,
                      PRIORITY_ONE_NEIGH_BIG_TIME_DIFF = 4,
                      PRIORITY_NO_NEIGHS_BIG_TIME_DIFF = 11,
                      PRIORITY_MANY_TARGS_VERY_BIG_TIME_DIFF = 5,
                      PRIORITY_ONE_TARG_VERY_BIG_TIME_DIFF = 6,
                      PRIORITY_NO_TARGS_VERY_BIG_TIME_DIFF = 12,
                      PRIORITY_MANY_TARGS_BIG_TIME_DIFF = 7,
                      PRIORITY_ONE_TARG_BIG_TIME_DIFF = 8,
                      PRIORITY_NO_TARGS_BIG_TIME_DIFF = 13,
                      PRIORITY_NEIGH_COLLISION_SMALL_TIME_DIFF = 10,
                      PRIORITY_NEIGH_COLLISION_VERY_SMALL_TIME_DIFF = 9,
                      PRIORITY_FEW_TARGS_MANY_HEIGHS = 14,
                      PRIORITY_NO_TARGS_MEDIUM_TIME_DIFF = 15;
                      
            
    // Constants - END
    
    private UWVehicleControl agentController;
    private ArrayList<Trigger> triggers;
    private ArrayList<Integer> triggerHashes;
    
    public MiniTriggerSynthesizer( UWVehicleControl agentController ) {
        this.agentController = agentController;
        triggers = new ArrayList<Trigger>();
        triggerHashes = new ArrayList<Integer>();
    }
    
    @Override
    public void SynthesizeTriggers() {

        // Clear old triggers
        triggers.clear();
        
        // Shared
        Trigger trigger;
        
        // ==================================================================================
        // Check and trigger "no neighbors for a while" or "many neighbors after long search"
        // ==================================================================================
        float ratio = ((float)getDurationSince(TriggeringEvent.LAST_NEIGHBOR_ENCOUNTER)) / MAX_ACCEP_TIME_WITHOUT_NEIGH;
        TimeDiff timeCheckRes = (TimeDiff)calcTriggerLevel(TimeDiff.values(), ratio);
        
        if( timeCheckRes.equals(TimeDiff.VERY_BIG) ||
            timeCheckRes.equals(TimeDiff.BIG)) {
            
            int neighCount = agentController.getSensedNeighCount();
            
            if( neighCount > NORMALLY_EXPECTED_NEIGH_COUNT ) {
                trigger = new Trigger(
                        PRIORITY_MANY_NEIGHS_BIG_TIME_DIFF,
                        agentController.getCurrentTick(),
                        agentController.getCurrentTick() + ETST_MANY_NEIGHS_BIG_TIME_DIFF,
                        new Enum[]{Senses.MANY_NEIGHBORS, TimeDiff.BIG}
                );
            } else if( neighCount > 1 && neighCount <= NORMALLY_EXPECTED_NEIGH_COUNT ) {
                trigger = new Trigger(
                        PRIORITY_FEW_NEIGHS_BIG_TIME_DIFF,
                        agentController.getCurrentTick(),
                        agentController.getCurrentTick() + ETST_FEW_NEIGHS_BIG_TIME_DIFF,
                        new Enum[]{Senses.FEW_NEIGHBORS, TimeDiff.BIG}
                );
            } else if( neighCount == 1 ) {
                
                trigger = new Trigger(
                        PRIORITY_ONE_NEIGH_BIG_TIME_DIFF,
                        agentController.getCurrentTick(),
                        agentController.getCurrentTick() + ETST_ONE_NEIGH_BIG_TIME_DIFF,
                        new Enum[]{Senses.ONE_NEIGHBOR, TimeDiff.BIG}
                );
            } else {
                trigger = new Trigger(
                        PRIORITY_NO_NEIGHS_BIG_TIME_DIFF,
                        agentController.getCurrentTick(),
                        agentController.getCurrentTick() + ETST_NO_NEIGHS_BIG_TIME_DIFF,
                        new Enum[]{Senses.NO_NEIGHBORS, TimeDiff.BIG}
                );
            }
            
            triggers.add(trigger);
            triggerHashes.add(trigger.hashCode());
            
        }
        // ==================================================================================
        // Check and trigger "targets found after long search" or 
        //                   "no targets for a while" or 
        //                   "many targets after long search"
        // ==================================================================================
        ratio = ((float)getDurationSince(TriggeringEvent.START_OF_SEARCH)) / MAX_ACCEP_SEARCH_TIME;
        timeCheckRes = (TimeDiff)calcTriggerLevel(TimeDiff.values(), ratio);
        
        if( timeCheckRes.equals(TimeDiff.VERY_BIG) ) {
            if( agentController.isTargetFound() ) {
                if( agentController.getSensedTargetsCount() > 1 ) {
                    trigger = new Trigger(
                            PRIORITY_MANY_TARGS_VERY_BIG_TIME_DIFF,
                            agentController.getCurrentTick(),
                            agentController.getCurrentTick() + ETST_MANY_TARGS_VERY_BIG_TIME_DIFF,
                            new Enum[]{Senses.MANY_TARGETS, TimeDiff.VERY_BIG}
                    );
                } else {
                    trigger = new Trigger(
                            PRIORITY_ONE_TARG_VERY_BIG_TIME_DIFF,
                            agentController.getCurrentTick(),
                            agentController.getCurrentTick() + ETST_ONE_TARG_VERY_BIG_TIME_DIFF,
                            new Enum[]{Senses.A_TARGET_NEARBY, TimeDiff.VERY_BIG}
                    );
                }
            } else {
                trigger = new Trigger(
                        PRIORITY_NO_TARGS_VERY_BIG_TIME_DIFF,
                        agentController.getCurrentTick(),
                        agentController.getCurrentTick() + ETST_NO_TARGS_VERY_BIG_TIME_DIFF,
                        new Enum[]{Senses.NO_TARGETS, TimeDiff.VERY_BIG}
                );
            }
            
            triggers.add(trigger);
            triggerHashes.add(trigger.hashCode());
            
        } else if( timeCheckRes.equals(TimeDiff.BIG) ) { 
            // Check and trigger "targets found after long search"
            if( agentController.isTargetFound() ) {
                if( agentController.getSensedTargetsCount() > 1 ) {
                    trigger = new Trigger(
                            PRIORITY_MANY_TARGS_BIG_TIME_DIFF,
                            agentController.getCurrentTick(),
                            agentController.getCurrentTick() + ETST_MANY_TARGS_BIG_TIME_DIFF,
                            new Enum[]{Senses.MANY_TARGETS, TimeDiff.BIG}
                    );
                } else {
                    trigger = new Trigger(
                            PRIORITY_ONE_TARG_BIG_TIME_DIFF,
                            agentController.getCurrentTick(),
                            agentController.getCurrentTick() + ETST_ONE_TARG_BIG_TIME_DIFF,
                            new Enum[]{Senses.A_TARGET_NEARBY, TimeDiff.BIG}
                    );
                }
            } else { // trigger "no targets for a while"
                trigger = new Trigger(
                        PRIORITY_NO_TARGS_BIG_TIME_DIFF,
                        agentController.getCurrentTick(),
                        agentController.getCurrentTick() + ETST_NO_TARGS_BIG_TIME_DIFF,
                        new Enum[]{Senses.NO_TARGETS, TimeDiff.BIG}
                );
            }
            
            triggers.add(trigger);
            triggerHashes.add(trigger.hashCode());
        } else if( timeCheckRes.equals(TimeDiff.MEDIUM) ) {
            // No targets found yet
            if( !agentController.isTargetFound() ) {
                trigger = new Trigger(
                        PRIORITY_NO_TARGS_MEDIUM_TIME_DIFF,
                        agentController.getCurrentTick(),
                        agentController.getCurrentTick() + ETST_NO_TARGS_MEDIUM_TIME_DIFF,
                        new Enum[]{Senses.NO_TARGETS, TimeDiff.MEDIUM}
                );
            }
        }
        // ==================================================================================
        // Check and trigger "many neighbor collisions"
        // ==================================================================================
        ratio = ((float)agentController.getNumVehVehCollInTestPeriod()) / MAX_ALLOW_VEH_COLL_IN_PERIOD;
        Collisions collTestRes = (Collisions)calcTriggerLevel(Collisions.values(), ratio);
        
        if( collTestRes.equals(Collisions.MANY) ) {
            trigger = new Trigger(
                    PRIORITY_NEIGH_COLLISION_SMALL_TIME_DIFF,
                    agentController.getCurrentTick(),
                    agentController.getCurrentTick() + ETST_NEIGH_COLLISION_SMALL_TIME_DIFF,
                    new Enum[]{Senses.NEIGHBOR_COLLISION, TimeDiff.SMALL}
            );
            triggers.add(trigger);
            triggerHashes.add(trigger.hashCode());
        } else if( collTestRes.equals(Collisions.SO_MANY) ) {
            trigger = new Trigger(
                    PRIORITY_NEIGH_COLLISION_VERY_SMALL_TIME_DIFF,
                    agentController.getCurrentTick(),
                    agentController.getCurrentTick() + ETST_NEIGH_COLLISION_VERY_SMALL_TIME_DIFF,
                    new Enum[]{Senses.NEIGHBOR_COLLISION, TimeDiff.VERY_SMALL}
            );
            triggers.add(trigger);
            triggerHashes.add(trigger.hashCode());
        }
        // ==================================================================================
        // Check and trigger "few targets and many neighbors"
        // ==================================================================================
        if( agentController.isTargetFound() ) {
            ratio = ((float)agentController.getSensedNeighCount()) / agentController.getSensedTargetsCount();
            TriggerLevel triggerTestRes = (TriggerLevel)calcTriggerLevel(TriggerLevel.values(), ratio);
            if( triggerTestRes.equals(TriggerLevel.HIGH) || triggerTestRes.equals(TriggerLevel.VERY_HIGH) ) {
                trigger = new Trigger(
                        PRIORITY_FEW_TARGS_MANY_HEIGHS,
                        agentController.getCurrentTick(),
                        agentController.getCurrentTick() + ETST_FEW_TARGS_MANY_HEIGHS,
                        new Enum[]{Senses.FEW_TARGETS, Senses.MANY_NEIGHBORS}
                );
                triggers.add(trigger);
                triggerHashes.add(trigger.hashCode());
            }
        }
        // ==================================================================================
        // Check and trigger "neighbors heading away" or "heading away from neighbors"
        // ==================================================================================
        if( agentController.getSensedVehiclesList() != null && agentController.getSensedVehiclesList().length > 0 ) {
            Vector3f avgNeighDir = agentController.getAvgNeighDirection().normalize();
            Vector3f auvDir = agentController.getCurrentVehicleVelocity().normalize();
            if( FastMath.abs(auvDir.angleBetween(avgNeighDir)) > 0.75*FastMath.PI ) {
                trigger = new Trigger(
                        PRIORITY_HEADING_AWAY_FROM_NEIGHS,
                        agentController.getCurrentTick(),
                        agentController.getCurrentTick() + ETST_HEADING_AWAY_FROM_NEIGHS,
                        new Enum[]{Senses.HEADING_AWAY_FROM_NEIGHBORS}
                );
                triggers.add(trigger);
                triggerHashes.add(trigger.hashCode());
            }
        }
        // ==================================================================================
        // Create initial trigger (which should drive the whole mission)
        // ==================================================================================
        if( agentController.getElapsedMissionTime().equals(ElapsedTime.SHORT) || 
                agentController.getElapsedMissionTime().equals(ElapsedTime.VERY_SHORT) ) {
            trigger = new Trigger(
                    PRIORITY_NO_TARGS_BEGINNING_OF_TIME,
                    agentController.getCurrentTick(),
                    agentController.getCurrentTick() + ETST_NO_TARGS_BEGINNING_OF_TIME,
                    new Enum[]{Senses.NO_TARGETS, TimeDiff.VERY_SMALL}
            );
            triggers.add(trigger);
            triggerHashes.add(trigger.hashCode());
        }

    }
    
    public ArrayList<Integer> getTrigHashes() {
        return triggerHashes;
    }
    
    private Enum calcTriggerLevel(Enum[] triggerValues, float triggerRelativeToMaxAllow) {
        
        if( triggerRelativeToMaxAllow >= 0.8 ) {
            return triggerValues[4];
        } else if( triggerRelativeToMaxAllow >= 0.6 && triggerRelativeToMaxAllow < 0.8 ) {
            return triggerValues[3];
        } else if( triggerRelativeToMaxAllow >= 0.4 && triggerRelativeToMaxAllow < 0.6 ) {
            return triggerValues[2];
        } else if( triggerRelativeToMaxAllow >= 0.2 && triggerRelativeToMaxAllow < 0.4 ) {
            return triggerValues[1];
        } else {
            return triggerValues[0];
        }
    }

    @Override
    public int getDurationSince( TriggeringEvent trigger ) {
        
        int timeDiff = 0;
        
        //System.out.println("Trigger info:" + trigger.name());
        
        if( agentController.isTimeSet(trigger) ) {
           timeDiff = agentController.getCurrentTick() 
                   - agentController.getStartTimeOf(trigger);
        } else {
           System.out.println("Time for "+trigger+" trigger not tracked!");
        }

        return timeDiff;
        
    }   

    @Override
    public ArrayList<Trigger> getTriggers() {
        return triggers;
    }
    
}

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.struct.artif;

import swim.exception.CallOrderException;
import swim.sim.uwswarm.UWVehicleControl;
import swim.sim.uwswarm.intelli.brain.constr.ActionType;
import swim.sim.uwswarm.intelli.brain.constr.Behavior;
import swim.sim.uwswarm.intelli.brain.constr.TriggerOccurrence;

/**
 *
 * @author Sherif
 * Takes results from ActionTracker and uses them to assess actions
 */
public abstract class BehaviorAssessor {
    
    public abstract void fetchPreviousBehavior();
    public abstract void fetchTargetShortTermReward();
    public abstract void fetchTargetLongTermReward();
    public abstract void fetchActualReward();
    
    protected abstract void rewardPenalizeBehavior();
    
    public abstract boolean behaviorAssessed(Behavior behavior);
    public abstract float getScore(Behavior behavior);

}

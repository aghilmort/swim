/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.struct.artif;

import java.util.ArrayList;
import swim.sim.uwswarm.intelli.brain.constr.RewardSize;
import swim.sim.uwswarm.intelli.brain.state.ActionReinforcements;
import swim.sim.uwswarm.intelli.resource.BatteryLevel;
import swim.sim.uwswarm.intelli.resource.TimeDiff;
import swim.sim.uwswarm.intelli.sense.Senses;

/**
 *
 * @author Sherif
 */
public class Trigger {

    //private TriggeringEvent name;
    private Enum[] signals;
    private ArrayList<Senses> senses;
    private ArrayList<TimeDiff> timeDiffs;
    private ArrayList<RewardSize> rewardSizes;
    private ArrayList<ActionReinforcements> actionReinforcements;
    private ArrayList<BatteryLevel> batteryLevels;
    
    private int priority;
    
    private int startTime, deadline;
    
    public Trigger(Enum[] signals) {    //TriggeringEvent triggerName, 
        this.signals = signals;
        //this.name = triggerName;
    }
    
    public Trigger(int priority, int startTime, int deadline, Enum[] signals) { 
        this.startTime = startTime;
        this.deadline = deadline;
        this.signals = signals;
        this.priority = priority;
    }

    public ArrayList<Senses> getSensesList() {
        
        if( senses == null ) {
            senses = new ArrayList<Senses>();
            for(Enum signal : signals) {
                if( signal instanceof Senses ) {
                    senses.add((Senses)signal);
                }
            }
        }
        
        return senses;
    }
    
    public ArrayList<TimeDiff> getTimeDiffsList() {
        
        if( timeDiffs == null ) {
            timeDiffs = new ArrayList<TimeDiff>();
        
            for(Enum signal : signals) {
                if( signal instanceof TimeDiff ) {
                    timeDiffs.add((TimeDiff)signal);
                }
            }
        }

        return timeDiffs;
    }
    
    public ArrayList<RewardSize> getRewardSizesList() {
        
        if( rewardSizes == null ) {
            rewardSizes = new ArrayList<RewardSize>();
        
            for(Enum signal : signals) {
                if( signal instanceof RewardSize ) {
                    rewardSizes.add((RewardSize)signal);
                }
            }
        }

        return rewardSizes;
    }
    
    public ArrayList<ActionReinforcements> getActionReinforcementsList() {
        
        if( actionReinforcements == null ) {
          actionReinforcements = new ArrayList<ActionReinforcements>();
        
            for(Enum signal : signals) {
                if( signal instanceof ActionReinforcements ) {
                    actionReinforcements.add((ActionReinforcements)signal);
                }
            }  
        }
        
        return actionReinforcements;
    }
    
    public ArrayList<BatteryLevel> getBatteryLevelsList() {
        
        if( batteryLevels == null ) {
            batteryLevels = new ArrayList<BatteryLevel>();
        
            for(Enum signal : signals) {
                if( signal instanceof BatteryLevel ) {
                    batteryLevels.add((BatteryLevel)signal);
                }
            }
        }

        return batteryLevels;
    }
    

//    @Override
//    public TriggeringEvent getName() {
//        return name;
//    }
//
//    @Override
//    public void setName(TriggeringEvent triggerName) {
//        this.name = triggerName;
//    }

    public int getStartTime() {
        return startTime;
    }

    public void setStartTime(int startTime) {
        this.startTime = startTime;
    }

    public int getDeadline() {
        return deadline;
    }

    public void setDeadline(int deadline) {
        this.deadline = deadline;
    }

    public int getPriority() {
        return priority;
    }

    public void setPriority(int priority) {
        this.priority = priority;
    }
    
    public Enum[] getSignals() {
        return signals;
    }
    
}

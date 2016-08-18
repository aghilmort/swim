/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.constr;

import java.util.ArrayList;
import swim.sim.uwswarm.intelli.brain.struct.artif.Trigger;

/**
 *
 * @author Sherif
 */
public class Situation extends Trigger {
    
    private int timeOfTriggers;
    private ArrayList<Trigger> triggers;
    private Emotion stimulatedEmotion;
    
    public Situation(ArrayList<Trigger> triggers, Emotion stimulatedEmotion) {
        super(new Enum[1]);
        this.triggers = triggers;
        this.stimulatedEmotion = stimulatedEmotion;
        // All triggers are synthesized in the same simulation tick
        this.timeOfTriggers = triggers.get(0).getStartTime();
    }

    public Situation() {
        super(new Enum[1]);
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    public ArrayList<Trigger> getTriggers() {
        return triggers;
    }

    public void setTriggers(ArrayList<Trigger> triggers) {
        this.triggers = triggers;
    }

    public Emotion getStimulatedEmotion() {
        return stimulatedEmotion;
    }

    public void setStimulatedEmotion(Emotion stimulatedEmotion) {
        this.stimulatedEmotion = stimulatedEmotion;
    }

    public int getTimeOfTriggers() {
        return timeOfTriggers;
    }

}

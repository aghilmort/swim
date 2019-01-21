/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.constr;

import swim.sim.uwswarm.intelli.brain.struct.artif.Trigger;
import swim.sim.uwswarm.intelli.brain.struct.bio.Feelings;

/**
 *
 * @author Sherif
 */
public class Behavior {
    
    private long uniqueID;
    private Trigger highestPriTrigger;
    private Emotion dominantEmotion;
    private Action  selectedAction;
    
    public Behavior(Trigger highestPriTrigger, Emotion dominantEmotion, Action selectedAction) {
        this.uniqueID = System.currentTimeMillis();
        this.highestPriTrigger = highestPriTrigger;
        this.dominantEmotion = dominantEmotion;
        this.selectedAction = selectedAction;
    }

    public Trigger getHighestPriTrigger() {
        return highestPriTrigger;
    }

    public Emotion getDominantEmotion() {
        return dominantEmotion;
    }
    
    public Feelings[] getFeelingsOfDominantEmotion() {
        return dominantEmotion.getFeelingsList();
    }

    public Action getSelectedAction() {
        return selectedAction;
    }

    public long getUniqueID() {
        return uniqueID;
    }

}

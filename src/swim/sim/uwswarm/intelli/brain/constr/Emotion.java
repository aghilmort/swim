/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.constr;

import java.util.Arrays;
import swim.sim.uwswarm.intelli.brain.constr.BrainConstruct;
import swim.sim.uwswarm.intelli.brain.struct.artif.Trigger;
import swim.sim.uwswarm.intelli.brain.struct.bio.Feelings;

/**
 *
 * @author Sherif
 */
public class Emotion extends Trigger implements BrainConstruct {
    
    private String name = "Emotion";
    //private TriggeringEvent name;
    private Feelings[] feelings;
    
    private float strength = 0;
    
    public Emotion(Feelings[] feelings) { //TriggeringEvent emotionName, 
        super(feelings);
        
        this.feelings = feelings;
        //this.name = emotionName;
    }

    public Feelings[] getFeelingsList() {
        return feelings;
    }   
    
    public boolean contains(Feelings feeling) {
        return Arrays.asList(feelings).contains(feeling);
    }

//    @Override
//    public TriggeringEvent getName() {
//        return name;
//    }
//
//    @Override
//    public void setName(TriggeringEvent triggerName) {
//        this.name = name;
//    }

    @Override
    public String getConstructName() {
        return name;
    }

    @Override
    public void setConstructName(String name) {
        this.name = name;
    }

    public float getStrength() {
        return strength;
    }

    public void setStrength(float strength) {
        this.strength = strength;
    }
    
    
}

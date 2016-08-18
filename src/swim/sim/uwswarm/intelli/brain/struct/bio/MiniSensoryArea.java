/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.struct.bio;

import java.util.ArrayList;
import java.util.Arrays;
import swim.sim.uwswarm.UWVehicleControl;
import swim.sim.uwswarm.intelli.brain.MiniBrain;
import swim.sim.uwswarm.intelli.brain.constr.Emotion;
import swim.sim.uwswarm.intelli.brain.struct.artif.MiniTriggerSynthesizer;
import swim.sim.uwswarm.intelli.brain.struct.artif.Trigger;
import swim.sim.uwswarm.intelli.brain.struct.artif.TriggeringEvent;
import swim.sim.uwswarm.intelli.sense.Senses;

/**
 *
 * @author Sherif
 */
public class MiniSensoryArea extends SensoryArea {

    // =========================================================================
    // Constants
    // =========================================================================
    private static final int NUM_SUB_AREAS = 1;
    // =========================================================================
    // Variables
    // =========================================================================
    private MiniTriggerSynthesizer triggerSynthesizer;
    private UWVehicleControl agentControl;
    
    private ArrayList<Trigger> stimuli;
    
    private MiniBrain brain;
    // =========================================================================
    
    public MiniSensoryArea(MiniBrain brain, UWVehicleControl auvController) {
        super(NUM_SUB_AREAS);
        
        // Reference to brain and vehicle controller
        this.agentControl = auvController;
        this.brain = brain;
        
        // Create a sensed-stimuli (environmental & internal) tracker 
        triggerSynthesizer = new MiniTriggerSynthesizer(agentControl);

    }
    
    public int getDurationSince( TriggeringEvent trigger ) {
        return triggerSynthesizer.getDurationSince(trigger);
    }
    
    public ArrayList<Trigger> getStimuli() {
        return stimuli;
    }

    @Override
    protected void function() {
        
        // 1) Get the stimuli from the trigger synthesizer
        triggerSynthesizer.SynthesizeTriggers();
        stimuli = triggerSynthesizer.getTriggers();
        
        System.out.println("===============================================");
        System.out.println("Number of generated triggers: " + stimuli.size());
        Trigger highPriTrigg = getHighestPriorityTrigger(stimuli);
        
        if( highPriTrigg != null ) {
           for(Enum signal : highPriTrigg.getSignals()) {
                System.out.println("Trigger (current) ("+agentControl.getVehicleName() + "): "+ signal);
            } 
        } else {
            System.out.println("No highest priority trigger!");
        }
        
        System.out.println("===============================================");
        
        // 2) Use these stimuli to excite Higher Mental Function by firing them
        //    to it
        if( !stimuli.isEmpty() ) {
            fire();
        }
    }

    @Override
    public void excite(ExcitationSignal signal) {
        function();
    }
    
    public void excite() {
        function();
    }

    @Override
    protected void fire() {
        
        // 1) Excite emotional area:
        // Map the stimuli to the corresponding feelings (to be stimulated) 
        // using brain's "SensesToEmotionsMap" and use the feelings to excite  
        // their corresponding neural networks in emotional area
        
        // Emotions excited by the triggers
        Emotion stimulatedEmotion;
        Feelings[] stimulatedFeelings;

        for( Trigger stimulus : stimuli ) {
            
            stimulatedEmotion = (Emotion)brain.getTriggersToEmotionsMap().getAssociation(Arrays.hashCode(stimulus.getSignals()));
            stimulatedFeelings = stimulatedEmotion.getFeelingsList();
            
            for( Feelings feeling : stimulatedFeelings ) {
                brain.getEmotionalArea().excite(new ExcitationSignal(feeling, brain.DEFAULT_EXCITATION_SIGNAL_STRENGTH)); 
            }
        }
        
        // 2) Excite higher mental function (to select an action to execute)
        brain.getHigherMentalFunc().excite(new HMFExcitationSignal(stimuli));
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
    
}

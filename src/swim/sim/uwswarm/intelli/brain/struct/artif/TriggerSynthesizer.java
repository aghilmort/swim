/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.struct.artif;

import java.util.ArrayList;

/**
 *
 * @author Sherif
 */
public abstract class TriggerSynthesizer {
    public abstract void SynthesizeTriggers();
    public abstract ArrayList<Trigger> getTriggers();
    public abstract int getDurationSince( TriggeringEvent durationSince );
}
/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.struct.artif;

/**
 *
 * @author Sherif
 */
public class MotionPeriodicity {
    
    private Type periodicityType;
    private int repititions;
    
    public MotionPeriodicity(Type periodicityType, int repititions) {
        this.periodicityType = periodicityType;
        this.repititions = repititions;
    }
    
    public MotionPeriodicity(Type periodicityType) {
        this.periodicityType = periodicityType;
        this.repititions = 0;
    } 

    public Type type() {
        return periodicityType;
    }

    public int repititions() {
        return repititions;
    }

    public void setType(Type periodicityType) {
        this.periodicityType = periodicityType;
    }
 
    
    
    public enum Type {
        PERIODIC,
        APERIODIC,
        SAME_AS_BEFORE,
        RANDOM_AMONG_TYPES,
        OF_BEST_REWARDED_ACTION
    }
 
}

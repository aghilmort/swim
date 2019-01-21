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
public enum TriggeringEvent {
    
    LAST_NEIGHBOR_ENCOUNTER,
    NUM_NEIGH_DROPPED_BELOW_THRES,
    LAST_NEIGHBOR_COLLISION,
    START_OF_MISSION,
    START_OF_SELF_ORG,
    START_OF_TASK_ALLOC,
    START_OF_SOURCE_SEEKING,
    START_OF_SEQUENCE,      // TURN SEQUENCE
    START_OF_TRIGGER,       // e.g. selfishness trigger
    START_OF_CHASING,
    START_OF_SEARCH,        // e.g. random walk  
    TARGET_FOUND,
    FIRST_TARGET_ENCOUNTER
}

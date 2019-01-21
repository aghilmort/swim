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
public enum MotionAlgorithm {
    
    LOG_SPIRAL,
    FLOCKING,
    TOWARDS_NEIGHBORS_AVERAGE,
    RANDOM_WALK_BIG_JUMPS,
    RANDOM_WALK_SMALL_JUMPS,
    SENSE_TARGET_THEN_RANDOM_WALK,
    RANDOM_AMONG_ALGS,
    SAME_AS_BEFORE,
    PREDEFINED_MISSION_STAGE_ALG,
    OF_BEST_REWARDED_ACTION,
    SURFACING
    
}

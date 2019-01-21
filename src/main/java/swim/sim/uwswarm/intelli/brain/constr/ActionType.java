/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.constr;

/**
 *
 * @author Sherif
 */
public enum ActionType {
    
    FOLLOW_PREDEF_MISSION_STAGE_PLAN,
    
    FIND_NEIGHBORS_LOCALLY,
    FIND_NEIGHBORS_GLOBALLY,
    FLOCK,
    FLOCK_THEN_LEAD,
    ESCAPE_CROWD,
    SEARCH_TARGETS_LOCALLY,
    SEARCH_TARGETS_GLOBALLY,
    NOTIFY_OTHERS_ABOUT_TARGETS,
    IDENTIFY_AND_PERFORM_TASK,
    ACT_CREATIVELY,
    CHASE_TARGETS,
    CHASE_NEIGHBORS,
    CATCH_UP_WITH_NEIGHBORS,
    SCURRY_RANDOMLY,
    SURFACE,
    IDLE,
    KEEP_CURRENT_ACTION,
    KEEP_CURRENT_PLACE,
    THE_MOST_REWARDED,
    
    DECLARE_MISSION_COMPLETED_WITHIN_CONSTRAINT,
    DECLARE_PROCESSED_N_TARGETS,
    DECLARE_MAINTAINED_LONG_TERM_COHESION
}

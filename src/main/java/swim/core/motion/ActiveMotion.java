/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.core.motion;

/**
 *
 * @author Sherif
 */
public enum ActiveMotion {
    SELF_ORG_ALG,                   // Self-Organization algorithm in Self-Organization stage
    TARG_SEARCH_SEARCH_ALG,         // Search algorithm in target search stage
    TARG_SEARCH_SEARCH_ALG_SEQ,     // Search algorithm sequence in target search stage
    TARG_SEARCH_MOTION_ALG,         // Motion algorithm in target search stage
    TARG_SEARCH_MOTION_ALG_SEQ,     // Motion algorithm sequence in target search stage
    TASK_ALLOC_MOTION_ALG,          // Motion algorithm in task allocation stage
    TASK_ALLOC_MOTION_ALG_SEQ,      // Motion algorithm sequence in task allocation stage 
}

package swim.sim.uwswarm.intelli.brain.struct.artif;

import java.util.HashMap;
import java.util.Map;
import swim.sim.uwswarm.intelli.brain.struct.bio.MotorFunction;

/**
 *
 * @author Sherif
 */
public class MotorFuncsRepository {
    
    private Map<String, MotorFunction> motorFuncsRepo;
    
    public MotorFuncsRepository() {
      motorFuncsRepo = new HashMap<String, MotorFunction>();
      buildRepository();
    }
    
    private void buildRepository() {
        
        // Shared
        String sequenceDesc, algorithmDesc;
        
        sequenceDesc = "Reconnaissance";
        motorFuncsRepo.put(sequenceDesc, new MotorFunction(
                    new MotorFuncParams(
                        sequenceDesc,    
                        MotionPace.PROP_TO_LAST_NEIGH_ENCOUNTER_TIME,
                        new MotorFuncType(MotionType.SEQUENCE, sequenceDesc),
                        new MotionPeriodicity(MotionPeriodicity.Type.PERIODIC, 20),
                        MotionDirection.RANDOM,
                        Duration.PRESET)
            ));
        
        algorithmDesc = "WideAreaNeighborsSearch";
        motorFuncsRepo.put(algorithmDesc, new MotorFunction(
                    new MotorFuncParams(
                        algorithmDesc, 
                        MotionPace.PROP_TO_LAST_NEIGH_ENCOUNTER_TIME,
                        new MotorFuncType(MotionType.ALGORITHM, MotionAlgorithm.LOG_SPIRAL),
                        new MotionPeriodicity(MotionPeriodicity.Type.APERIODIC),
                        MotionDirection.DECIDED,
                        Duration.INDEFINITE)    
            ));
        
        algorithmDesc = "NeighborsFlocking";
        motorFuncsRepo.put(algorithmDesc, new MotorFunction(
                    new MotorFuncParams(
                        algorithmDesc,    
                        MotionPace.AVERAGE_OF_NEIGHBORS,
                        new MotorFuncType(MotionType.ALGORITHM, MotionAlgorithm.FLOCKING),
                        new MotionPeriodicity(MotionPeriodicity.Type.APERIODIC),
                        MotionDirection.DECIDED,
                        Duration.INDEFINITE)
            ));
        
        sequenceDesc = "CreativeMoves";
        motorFuncsRepo.put(sequenceDesc, new MotorFunction(
                    new MotorFuncParams(
                        sequenceDesc,   
                        MotionPace.FAST,
                        //@TODO: Specify turn sequence 
                        new MotorFuncType(MotionType.SEQUENCE, sequenceDesc),
                        new MotionPeriodicity(MotionPeriodicity.Type.APERIODIC),
                        MotionDirection.RANDOM,
                        Duration.DUR_OF_SEQUENCE)
            ));
        
        sequenceDesc = "NeighborsEscape";
        motorFuncsRepo.put(sequenceDesc, new MotorFunction(
                    new MotorFuncParams(
                        sequenceDesc,    
                        MotionPace.FAST_THEN_SLOW,
                        new MotorFuncType(MotionType.SEQUENCE, sequenceDesc),
                        new MotionPeriodicity(MotionPeriodicity.Type.APERIODIC),
                        MotionDirection.DIR_OF_SEQUENCE,
                        Duration.DUR_OF_SEQUENCE)
            ));
        
        algorithmDesc = "TargetChasing";
        motorFuncsRepo.put(algorithmDesc, new MotorFunction(
                    new MotorFuncParams(
                        algorithmDesc,    
                        MotionPace.FAST,
                        new MotorFuncType(MotionType.ALGORITHM, MotionAlgorithm.SENSE_TARGET_THEN_RANDOM_WALK),
                        new MotionPeriodicity(MotionPeriodicity.Type.APERIODIC),
                        MotionDirection.DIR_OF_TARGETS,
                        Duration.TIME_TO_REACH_TARGETS)
            ));
        
        algorithmDesc = "NeighborsChasing";
        motorFuncsRepo.put(algorithmDesc, new MotorFunction(
                    new MotorFuncParams(
                        algorithmDesc,    
                        MotionPace.FAST,
                        new MotorFuncType(MotionType.ALGORITHM, MotionAlgorithm.TOWARDS_NEIGHBORS_AVERAGE),
                        new MotionPeriodicity(MotionPeriodicity.Type.APERIODIC),
                        MotionDirection.DIR_OF_AVERAGE,
                        Duration.TIME_TO_REACH_VEHICLES)
            ));
        
        algorithmDesc = "RandomScurrying";
        motorFuncsRepo.put(algorithmDesc, new MotorFunction(
                    new MotorFuncParams(
                        algorithmDesc,    
                        MotionPace.FAST,
                        new MotorFuncType(MotionType.ALGORITHM, MotionAlgorithm.RANDOM_WALK_BIG_JUMPS),
                        new MotionPeriodicity(MotionPeriodicity.Type.PERIODIC, 50),
                        MotionDirection.RANDOM,
                        Duration.PRESET)    //TODO: TIME TO ENCOUNTER
            ));
        
        algorithmDesc = "ShortRandomExploration";
        motorFuncsRepo.put(algorithmDesc, new MotorFunction(
                    new MotorFuncParams(
                        algorithmDesc,    
                        MotionPace.SLOW,
                        new MotorFuncType(MotionType.ALGORITHM, MotionAlgorithm.SENSE_TARGET_THEN_RANDOM_WALK),
                        new MotionPeriodicity(MotionPeriodicity.Type.APERIODIC),
                        MotionDirection.RANDOM,
                        Duration.TIME_TO_SENSE_TARGETS) 
            ));
        
        algorithmDesc = "NeighborsFlocking_2";
        motorFuncsRepo.put(algorithmDesc, new MotorFunction(
                    new MotorFuncParams(
                        algorithmDesc,    
                        MotionPace.AVERAGE_OF_NEIGHBORS,
                        new MotorFuncType(MotionType.ALGORITHM, MotionAlgorithm.FLOCKING),
                        new MotionPeriodicity(MotionPeriodicity.Type.APERIODIC),
                        MotionDirection.DIR_AVG_ALIGNMENT,
                        Duration.INDEFINITE)    
            ));
        
        algorithmDesc = "KeepingItUp";
        motorFuncsRepo.put(algorithmDesc, new MotorFunction(
                    new MotorFuncParams(
                        algorithmDesc,    
                        MotionPace.SAME,
                        new MotorFuncType(MotionType.ALGORITHM, MotionAlgorithm.SAME_AS_BEFORE),
                        new MotionPeriodicity(MotionPeriodicity.Type.SAME_AS_BEFORE),
                        MotionDirection.SAME,
                        Duration.WHILE_TARGETS_BELOW_THRES)
            ));
        
        algorithmDesc = "CreativeMoves_2";
        motorFuncsRepo.put(algorithmDesc, new MotorFunction(
                    new MotorFuncParams(
                        algorithmDesc,    
                        MotionPace.RANDOM_AMONG_PACES,
                        new MotorFuncType(MotionType.ALGORITHM, MotionAlgorithm.RANDOM_AMONG_ALGS),
                        new MotionPeriodicity(MotionPeriodicity.Type.RANDOM_AMONG_TYPES),
                        MotionDirection.RANDOM,
                        Duration.RANDOM)
            ));
        
        algorithmDesc = "KeepingItUp_2";
        motorFuncsRepo.put(algorithmDesc, new MotorFunction(
                    new MotorFuncParams(
                        algorithmDesc,    
                        MotionPace.SAME,
                        new MotorFuncType(MotionType.ALGORITHM, MotionAlgorithm.SAME_AS_BEFORE),
                        new MotionPeriodicity(MotionPeriodicity.Type.SAME_AS_BEFORE),
                        MotionDirection.SAME,
                        Duration.WHILE_REWARD_LVL_PERSISTS)
            ));
        
        algorithmDesc = "ShortRandomJumps";
        motorFuncsRepo.put(algorithmDesc, new MotorFunction(
                    new MotorFuncParams(
                        algorithmDesc,
                        MotionPace.SLOW,
                        new MotorFuncType(MotionType.ALGORITHM, MotionAlgorithm.RANDOM_WALK_SMALL_JUMPS),
                        new MotionPeriodicity(MotionPeriodicity.Type.PERIODIC, 50),
                        MotionDirection.RANDOM,
                        Duration.TIME_TO_SATISFY_GOAL)
            ));
        
        sequenceDesc = "Circling";
        motorFuncsRepo.put(sequenceDesc, new MotorFunction(
                    new MotorFuncParams(
                        sequenceDesc,    
                        MotionPace.FAST,
                        //@TODO: Specify turn sequence 
                        new MotorFuncType(MotionType.SEQUENCE, sequenceDesc),
                        new MotionPeriodicity(MotionPeriodicity.Type.PERIODIC, 15),
                        MotionDirection.DECIDED,
                        Duration.DUR_OF_SEQUENCE)
            ));
        
        algorithmDesc = "StableExploration";
        motorFuncsRepo.put(algorithmDesc, new MotorFunction(
                    new MotorFuncParams(
                        algorithmDesc,    
                        MotionPace.MEDIUM,
                        new MotorFuncType(MotionType.ALGORITHM, MotionAlgorithm.PREDEFINED_MISSION_STAGE_ALG),
                        new MotionPeriodicity(MotionPeriodicity.Type.APERIODIC),
                        MotionDirection.SENSING_BASED,
                        Duration.TIME_TO_SATISFY_GOAL)
            ));
        
        algorithmDesc = "MotionOfMostRewardedAction";
        motorFuncsRepo.put(algorithmDesc, new MotorFunction(
                    new MotorFuncParams(
                        algorithmDesc,    
                        MotionPace.OF_BEST_REWARDED_ACTION,
                        new MotorFuncType(MotionType.ALGORITHM, MotionAlgorithm.OF_BEST_REWARDED_ACTION),
                        new MotionPeriodicity(MotionPeriodicity.Type.OF_BEST_REWARDED_ACTION),
                        MotionDirection.OF_BEST_REWARDED_ACTION,
                        Duration.OF_BEST_REWARDED_ACTION)
            ));
        
        algorithmDesc = "MissionStageAction";
        motorFuncsRepo.put(algorithmDesc, new MotorFunction(
                    new MotorFuncParams(
                        algorithmDesc,    
                        MotionPace.INITIAL,
                        new MotorFuncType(MotionType.ALGORITHM, MotionAlgorithm.PREDEFINED_MISSION_STAGE_ALG),
                        new MotionPeriodicity(MotionPeriodicity.Type.APERIODIC),
                        MotionDirection.DECIDED,
                        Duration.TIME_TO_SATISFY_GOAL)
            ));
        
        algorithmDesc = "Surface";
        motorFuncsRepo.put(algorithmDesc, new MotorFunction(
                    new MotorFuncParams(
                        algorithmDesc,    
                        MotionPace.FAST,
                        new MotorFuncType(MotionType.ALGORITHM, MotionAlgorithm.SURFACING),
                        new MotionPeriodicity(MotionPeriodicity.Type.APERIODIC),
                        MotionDirection.DECIDED,
                        Duration.TIME_TO_SATISFY_GOAL)
            ));
       
    }
    
    public MotorFunction get(String name) {
        return motorFuncsRepo.get(name);
    }
    
    public MotorFunction add(String name, MotorFunction motorFunction) {
        return motorFuncsRepo.put(name, motorFunction);
    }
    
    public boolean contains(String name) {
        return motorFuncsRepo.containsKey(name);
    }
}

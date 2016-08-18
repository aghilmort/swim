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
public class MotorFuncParams {
    public String name;
    public MotionPace pace;
    public MotorFuncType type;
    public MotionPeriodicity periodicity;
    public Enum direction;
    public Duration duration;
    
    public MotorFuncParams(String name, MotionPace pace, MotorFuncType type, MotionPeriodicity periodicity, 
            Enum direction, Duration duration) {
        this.name = name;
        this.pace = pace;
        this.type = type;
        this.periodicity = periodicity;
        this.direction = direction;
        this.duration = duration; 
    }
}

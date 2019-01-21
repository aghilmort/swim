/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.struct.artif;

import com.jme3.math.FastMath;
import java.util.ArrayList;
import java.util.HashMap;
import swim.core.AgentControl;
import swim.core.motion.CompositeTurnSequence;
import swim.sim.uwswarm.UWVehicleControl;
import swim.util.Pair;

/**
 *
 * @author Sherif
 */
public class SequenceRepository {
    
    // Constants - START
    private final float MAX_TRAVEL_DIST = 30;
    private final int MAX_NUM_OF_SWEEPS = 5;
    private final float MAX_SWEEP_LENGTH = 30;
    private final int MAX_NUM_OF_CIRCLINGS = 10;
    
    // Constants - END
    
    private HashMap<String, CompositeTurnSequence> sequences;
    private CompositeTurnSequence sequence;
    
    private ArrayList<String> seqToDelete, sequenceNames;
    
    private AgentControl agentController;
    
    public SequenceRepository( AgentControl agentController ) {
        
        this.sequences = new HashMap<String, CompositeTurnSequence>();
        this.seqToDelete = new ArrayList<String>();
        this.sequenceNames = new ArrayList<String>();
        this.agentController = agentController;

        createSequences();
    }
    
    private void createSequences() {
        
        // Shared
        float angle, distance;
        
        // Turn backwards
        sequence = new CompositeTurnSequence();
        sequence.add("turn_direction", (FastMath.rand.nextFloat() > 0.5f ? "right" : "left"));
        sequence.add("turn_angle", FastMath.PI);
        sequence.setName("Backwards Turn");
        sequences.put(sequence.getName(), sequence);
        sequenceNames.add(sequence.getName());
        
        // Turn right
        sequence = new CompositeTurnSequence();
        sequence.add("turn_direction", "right");
        sequence.add("turn_angle", FastMath.HALF_PI);
        sequence.setName("Right Turn");
        sequences.put(sequence.getName(), sequence);
        sequenceNames.add(sequence.getName());
        
        // Turn left
        sequence = new CompositeTurnSequence();
        sequence.add("turn_direction", "left");
        sequence.add("turn_angle", FastMath.HALF_PI);
        sequence.setName("Left Turn");
        sequences.put(sequence.getName(), sequence);
        sequenceNames.add(sequence.getName());
        
        // Turn right with random angle
        angle = FastMath.PI * FastMath.rand.nextFloat();
        sequence = new CompositeTurnSequence();
        sequence.add("turn_direction", "right");
        sequence.add("turn_angle", angle);
        sequence.setName("Random Right Turn");
        sequence.setRandAngle(angle);
        sequences.put(sequence.getName(), sequence);
        sequenceNames.add(sequence.getName());
        
        // Turn left with random angle
        angle = FastMath.PI * FastMath.rand.nextFloat();
        sequence = new CompositeTurnSequence();
        sequence.add("turn_direction", "left");
        sequence.add("turn_angle", angle);
        sequence.setName("Random Left Turn");
        sequence.setRandAngle(angle);
        sequences.put(sequence.getName(), sequence);
        sequenceNames.add(sequence.getName());
        
        // Maneuver right (right with random angle less than 45 deg then revert it twice)
        angle = FastMath.QUARTER_PI * FastMath.rand.nextFloat();
        sequence = new CompositeTurnSequence();
        sequence.add("turn_direction", "right");
        sequence.add("turn_angle", angle);
        sequence.add("turn_direction", "left");
        sequence.add("turn_angle", 2*angle);
        sequence.add("turn_direction", "right");
        sequence.add("turn_angle", angle);
        sequence.setName("Random Right Maneuver");
        sequence.setRandAngle(angle);
        sequences.put(sequence.getName(), sequence);
        sequenceNames.add(sequence.getName());
        
        // Maneuver left (left with random angle less than 45 deg then revert it twice)
        angle = FastMath.QUARTER_PI * FastMath.rand.nextFloat();
        sequence = new CompositeTurnSequence();
        sequence.add("turn_direction", "left");
        sequence.add("turn_angle", angle);
        sequence.add("turn_direction", "right");
        sequence.add("turn_angle", 2*angle);
        sequence.add("turn_direction", "left");
        sequence.add("turn_angle", angle);
        sequence.setName("Random Left Maneuver");
        sequence.setRandAngle(angle);
        sequences.put(sequence.getName(), sequence);
        sequenceNames.add(sequence.getName());
        
        // Diverge right (right with random angle less than 45 deg then revert it once)
        angle = FastMath.QUARTER_PI * FastMath.rand.nextFloat();
        sequence = new CompositeTurnSequence();
        sequence.add("turn_direction", "right");
        sequence.add("turn_angle", angle);
        sequence.add("turn_direction", "left");
        sequence.add("turn_angle", angle);
        sequence.setName("Random Right Divergence");
        sequence.setRandAngle(angle);
        sequences.put(sequence.getName(), sequence);
        sequenceNames.add(sequence.getName());
        
        // Diverge left (left with random angle less than 45 deg then revert it once)
        angle = FastMath.QUARTER_PI * FastMath.rand.nextFloat();
        sequence = new CompositeTurnSequence();
        sequence.add("turn_direction", "left");
        sequence.add("turn_angle", angle);
        sequence.add("turn_direction", "right");
        sequence.add("turn_angle", angle);
        sequence.setName("Random Left Divergence");
        sequence.setRandAngle(angle);
        sequences.put(sequence.getName(), sequence);
        sequenceNames.add(sequence.getName());
        
        // Travel random distance then return
        distance = MAX_TRAVEL_DIST * FastMath.rand.nextFloat();
        sequence = new CompositeTurnSequence();
        sequence.add("travel", distance);
        sequence.add("turn_direction", (FastMath.rand.nextFloat() > 0.5f ? "right" : "left"));
        sequence.add("turn_angle", FastMath.PI);
        sequence.add("travel", distance);
        sequence.setName("Reconnaissance");
        sequence.setRandDist(distance);
        sequences.put(sequence.getName(), sequence);
        sequenceNames.add(sequence.getName());
        
        // Sweep along AUV's travel direction 
        // ----====
        //     ====
        //@ToDo: implement
        
        
        // Sweep normal to AUV's travel direction
        // ----||||||
        //     ||||||
        boolean turnRight = false, 
                firstSweep = true; 
        int numTurns = MAX_NUM_OF_SWEEPS - 1;
        String turnDir;
        distance = MAX_TRAVEL_DIST * FastMath.rand.nextFloat();
        sequence = new CompositeTurnSequence();
        
        for(int i = 1; i <= MAX_NUM_OF_SWEEPS; i++) {

            turnDir = (turnRight ? "right" : "left");

            if(firstSweep) {
                distance = MAX_SWEEP_LENGTH/2; 
                firstSweep = false;
            } else {
                distance = MAX_SWEEP_LENGTH;
            }

            sequence.add(new Pair<String,Object>("travel", distance));

            if(i < MAX_NUM_OF_SWEEPS) {
                sequence.add("turn_direction", turnDir);
                sequence.add("turn_angle", FastMath.PI);
                turnRight = !turnRight;
            } else {
                turnRight = !turnRight;
                turnDir = (turnRight ? "right" : "left");
                sequence.add("turn_direction", turnDir);
                sequence.add("turn_angle", FastMath.HALF_PI);

                if(MAX_NUM_OF_SWEEPS % 2 == 0) {    // Do further investigation here
                    distance = (numTurns - 1) * ((UWVehicleControl)agentController).getActiveTurnRadius() 
                            * ((UWVehicleControl)agentController).getMinVehicleSpeed();
                } else {
                    distance = numTurns * ((UWVehicleControl)agentController).getActiveTurnRadius() 
                            * ((UWVehicleControl)agentController).getMinVehicleSpeed(); 
                }

                sequence.add("travel", distance);
                sequence.add("turn_direction", turnDir);
                sequence.add("turn_angle", FastMath.HALF_PI);
                sequence.add("travel", MAX_SWEEP_LENGTH/2);
            }                

        }
        sequence.setName("Simple Sweeping");
        sequence.setRandDist(distance);
        sequences.put(sequence.getName(), sequence);
        sequenceNames.add(sequence.getName());
        
        // Circle for a number of times
        sequence = new CompositeTurnSequence();
        turnDir = (FastMath.rand.nextFloat() > 0.5f ? "right" : "left");
        
        for(int i = 1; i <= MAX_NUM_OF_CIRCLINGS; i++) {
            sequence.add("turn_direction", turnDir);
            sequence.add("turn_angle", FastMath.TWO_PI);
        }
        
        sequence.setName("Repeated Circling");
        sequences.put(sequence.getName(), sequence);
        sequenceNames.add(sequence.getName());
        
    }
    
    public void addSequence(CompositeTurnSequence sequence) {
        sequences.put(sequence.getName(), sequence);
    }
    
    public CompositeTurnSequence getRandomSequence() {
        
        if( sequenceNames.isEmpty() ) {
            return null;
        }
        
        int index = FastMath.rand.nextInt(sequenceNames.size());
        CompositeTurnSequence seq = sequences.get(sequenceNames.get(index));
        
        while( seqToDelete.contains(seq.getName()) ) {
            sequences.remove(sequenceNames.get(index));
            sequenceNames.remove(index);
            index = FastMath.rand.nextInt(sequenceNames.size());
            seq = sequences.get(sequenceNames.get(index));
        }

        return seq;
    }
    
    public void deleteSequence(String name) {
        seqToDelete.add(name);
    }
    
    public CompositeTurnSequence getSequence( String name ) {
        return sequences.get(name);
    }
    
    public int getSequencesCount() {
        return sequences.size();
    }
    
}

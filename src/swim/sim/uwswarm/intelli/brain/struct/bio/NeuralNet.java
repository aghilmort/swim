/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.struct.bio;

import com.jme3.math.FastMath;
import java.util.ArrayList;
import java.util.List;

/**
 *
 * @author Sherif
 */
public class NeuralNet {
    
    private List<Neuron> neurons;
    private List<Enum> pairedNets;
    
    private Enum name;
    
    private SubBrainArea parentBrainArea;
    
    public NeuralNet( SubBrainArea subBrainArea , int numNeurons ) {
        
        parentBrainArea = subBrainArea;
        
        neurons = new ArrayList<Neuron>(numNeurons);
        pairedNets = new ArrayList<Enum>();
        
        // Add neurons to the neural network (unconnected)
        for( int i = 1; i <= numNeurons; ++i ) {
            neurons.add(new Neuron(this, (numNeurons - 1), FastMath.rand.nextFloat()));
        }
        
        // Connect the neurons (assuming a fully-connected network)
        for( int i = 0; i < neurons.size(); ++i ) {
            for( int j = 0; j < neurons.size(); ++j ) {
                if( i != j ) {
                    neurons.get(i).addNeighbor(neurons.get(j));
                }
            }
        }
        
    }
    
    public void pairNet( Enum netName ) {
        pairedNets.add(netName);
    }
    
    public boolean hasPairedNets() {
        return ( pairedNets.size() > 0 );
    }
    
    public List<Enum> getPairedNetsList() {
        return pairedNets;
    }
    
    public NeuralNet getPairedNet( Enum netName ) {
        return parentBrainArea.getNeuralNet(netName);
    }
    
    public int size() {
        return neurons.size();
    }
    
    public Neuron getNeuron( int index ) {
        return neurons.get(index);
    }
    
    public float getNeuralActivityLevel() {
        float sumLevels = 0;
        for(Neuron neuron : neurons) {
           sumLevels += neuron.getNeuralContentLevel();
        }
        return sumLevels;
    }

    public Enum getName() {
        return name;
    }

    public void setName(Enum name) {
        this.name = name;
    }
    
    public void exciteRandNeuron(float excitationStrength) {
        // A signal is fired to a random neuron in the 
        int netSize = size();
        getNeuron(FastMath.rand.nextInt(netSize)).excite(excitationStrength);
    }
    
    public List<Neuron> getNeurons() {
        return neurons;
    }
    
}

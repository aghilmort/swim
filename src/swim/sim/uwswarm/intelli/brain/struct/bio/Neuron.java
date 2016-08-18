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
public class Neuron {
    
    private final float MAX_DATA_MAGNITUDE = 1000f;
    private float data;
    private List<Neuron> neighbors;
    private NeuralNet parentNet;
    
    public Neuron( NeuralNet parentNet, int numNeighbors, float data ){
        this.parentNet = parentNet;
        this.data = data;
        this.neighbors = new ArrayList<Neuron>(numNeighbors);
    }
    
    public void addNeighbor( Neuron neuron ) {
        neighbors.add(neuron);
    }
    
    public List<Neuron> getNeighbors() {
        return neighbors;
    }
    
    public void influenceNeuralContent( float excitationStrength ) {
        //@todo: add some validation mechanism
        this.data += excitationStrength * this.data;
        
        if( FastMath.rand.nextFloat() >= 0.95f && parentNet.hasPairedNets() ) {
            for( Enum net : parentNet.getPairedNetsList() ) {
                if( FastMath.rand.nextFloat() >= 0.8f ) {
                    exciteRandNeuronInNet( parentNet.getPairedNet(net) , excitationStrength);
                }
            }
        }
    }
    
    public void fadeNeuralContent( float excitationStrength ) {
        this.data -= excitationStrength * this.data;
    }
    
    public void exciteNeighbors( float excitationStrength ) {
        for( Neuron neighborNeuron : neighbors ) {
            neighborNeuron.influenceNeuralContent(excitationStrength);
        }
    }
    
    /**
     * Excites this neuron, which in-turn excites other neurons in its network 
     * with a signal strength proportional to the magnitude of the data content
     * of this neuron.
     */
    public void excite(float exStrength) {
        
        // Affect neural content of this neuron
        influenceNeuralContent( exStrength );
        
        float excitationStrength = data/MAX_DATA_MAGNITUDE;
        exciteNeighbors( excitationStrength );
        
        if( FastMath.rand.nextFloat() >= 0.95f && parentNet.hasPairedNets() ) {
            for( Enum net : parentNet.getPairedNetsList() ) {
                if( FastMath.rand.nextFloat() >= 0.8f ) {
                    exciteRandNeuronInNet( parentNet.getPairedNet(net) , excitationStrength);
                }
            }
        }
    }
    
    /**
     * Excites a random neuron in the specified network with a probability of
     * 0.2 (preset). The excited neuron in that network then excites its 
     * neighbors with a signal strength proportional to its data content.
     * @param neuralNet network in which to excite a random neuron
     */
    public void exciteRandNeuronInNet( NeuralNet neuralNet, float excitationStrength ) {
        // A signal is fired to a random neuron in the 
        if( FastMath.rand.nextFloat() >= 0.8f ) {
            int netSize = neuralNet.size();
            neuralNet.getNeuron(FastMath.rand.nextInt(netSize)).excite(excitationStrength);
        }
    }
    
    public float getNeuralContentLevel() {
        return data;
    }
    
}

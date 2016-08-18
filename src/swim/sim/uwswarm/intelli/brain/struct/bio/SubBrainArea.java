/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.struct.bio;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import swim.util.Pair;

/**
 *
 * @author Sherif
 */
public abstract class SubBrainArea {
    
    protected Map<Enum,NeuralNet> neuralNets;
    protected List<Enum> pairedAreas;
    protected BrainArea parentBrainArea;
    protected Enum[] netNames;
    
    public SubBrainArea( BrainArea parentBrainArea, int numNeuralNets, int nodesPerNet, Enum[] netNames ) {
        
        neuralNets = new HashMap<Enum,NeuralNet>(numNeuralNets);
        pairedAreas = new ArrayList<Enum>();
        this.parentBrainArea = parentBrainArea;
        this.netNames = netNames;
        
        for( int i = 0; i < numNeuralNets; ++i ) {
            neuralNets.put(netNames[i], new NeuralNet(this, nodesPerNet));
        }
        
    }
    
    public void pairArea( Enum areaName ) {
        pairedAreas.add(areaName);
    }
    
    public boolean hasPairedAreas() {
        return ( pairedAreas.size() > 0 );
    }
    
    public List<Enum> getPairedAreasList() {
        return pairedAreas;
    }
    
    public SubBrainArea getPairedArea( Enum areaName ) {
        return parentBrainArea.getSBArea(areaName);
    }
    
    public NeuralNet getNeuralNet( Enum name ) {
        return neuralNets.get(name);
    }
    
    public Enum[] getNeuralNetsNames() {
        return netNames;
    }
    
    public Pair<NeuralNet, Float> getNetWithHighestActivityLevel() {
        
        float highestActLevel = 0, netActLevel;
        NeuralNet currNet, selectedNet = null;
        Enum currNetName, selectedNetName = null;
        
        for( Entry<Enum, NeuralNet> entry : neuralNets.entrySet() ) {
            
            currNetName = entry.getKey();
            currNet = entry.getValue();
            
            netActLevel = currNet.getNeuralActivityLevel();
            highestActLevel = Math.max(highestActLevel, netActLevel);
            
            if( netActLevel == highestActLevel ) {
                selectedNet = currNet;
                selectedNetName = currNetName;
            }
        }
        
        if( selectedNet != null && selectedNetName != null ) {
            selectedNet.setName(selectedNetName);
        }
       
        return new Pair(selectedNet, highestActLevel);
    }
    
    /**
     * Core function of this brain area. Implementation should use it to execute
     * the main operations performed by this area.
     */
    protected abstract void function();
    
    /**
     * Takes an exciation signal (used to excite this specific brain area) 
     * as input and implements how the excitation process takes place.
     * @param signal input excitation signal
     */
    public abstract void excite(ExcitationSignal signal);
    
    /**
     * Any signals that this brain area may fire/send to other brain areas after
     * functioning and takening the necessary actions.
     */
    protected abstract void fire();
    
}

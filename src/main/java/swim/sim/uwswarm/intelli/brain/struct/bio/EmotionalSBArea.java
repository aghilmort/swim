/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.struct.bio;

/**
 *
 * @author Sherif
 */
public class EmotionalSBArea extends SubBrainArea {
    
    public EmotionalSBArea(EmotionalArea emotionalArea, int numNeuralNets, int nodesPerNet, Enum[] netNames) {
        super(emotionalArea, numNeuralNets, nodesPerNet, netNames);
    }
    
    @Override
    protected void function() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public void excite(ExcitationSignal signal) {
        
        Feelings netName = (Feelings)signal.getExcitationNeuralNet();
        float exStrength = signal.getStrength();
        
        //System.out.println("Net Name: "+netName);
        
        this.getNeuralNet(netName).exciteRandNeuron(exStrength);
        
    }

    @Override
    public void fire() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }
    
}

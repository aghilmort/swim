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
public interface Signal {
    
    public void setExcitationNeuralNet( Enum neuralNetName );
    public Enum getExcitationNeuralNet();
    
    public void setStrength( float strength );
    public float getStrength();
    
    
}

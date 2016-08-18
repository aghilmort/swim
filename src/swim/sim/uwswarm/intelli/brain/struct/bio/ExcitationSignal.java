package swim.sim.uwswarm.intelli.brain.struct.bio;

/**
 *
 * @author Sherif
 */
public class ExcitationSignal implements Signal {

    private float strength = 0;
    private Enum neuralNetName;
    
    public ExcitationSignal(Enum neuralNetName, float strength) {
        this.neuralNetName = neuralNetName;
        this.strength = strength;
    }

    public ExcitationSignal() {
        
    }

    @Override
    public void setExcitationNeuralNet(Enum neuralNetName) {
        this.neuralNetName = neuralNetName;
    }

    @Override
    public Enum getExcitationNeuralNet() {
        return this.neuralNetName;
    }

    @Override
    public void setStrength(float strength) {
        this.strength = strength;
    }

    @Override
    public float getStrength() {
        return this.strength;
    }
    
}

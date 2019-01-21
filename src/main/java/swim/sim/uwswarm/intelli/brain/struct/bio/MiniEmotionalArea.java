/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.struct.bio;

import com.jme3.math.FastMath;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import swim.sim.uwswarm.UWVehicleControl;
import swim.sim.uwswarm.intelli.brain.MiniBrain;
import swim.sim.uwswarm.intelli.brain.constr.Emotion;
import swim.sim.uwswarm.intelli.brain.constr.EmotionalStrength;
import swim.util.Pair;

/**
 *
 * @author Sherif
 */
public class MiniEmotionalArea extends EmotionalArea {

    private static final int NUM_SUB_AREAS = 6;
    private final float MAX_EMOTIONAL_STRENGTH = 10;
    private MiniBrain brain;
    private UWVehicleControl agentControl;
    
    private Map<String,EmotionalSBAreas> majorFeelingToSubAreaMap;
    
    private Feelings neuralNetToExcite;
    private float neuralNetExStrength;
    
    public MiniEmotionalArea(MiniBrain brain) {

        super(NUM_SUB_AREAS);
        
        this.brain = brain;
        this.agentControl = brain.getAgentController();
        
        // Construct sub-areas of the emotional area
        subBrainAreas.put(EmotionalSBAreas.POWERFULNESS_AREA, new MiniEmotionalSBArea(this, 2, 5, new Feelings[]{Feelings.IMPORTANT_POWERFUL, Feelings.FAITHFUL_POWERFUL}));
        subBrainAreas.put(EmotionalSBAreas.PEACEFULNESS_AREA, new MiniEmotionalSBArea(this, 2, 5, new Feelings[]{Feelings.CONTENT_PEACEFUL, Feelings.TRUSTING_PEACEFUL}));
        subBrainAreas.put(EmotionalSBAreas.JOYFULNESS_AREA,   new MiniEmotionalSBArea(this, 3, 5, new Feelings[]{Feelings.CREATIVE_JOYFUL, Feelings.EXCITED_JOYFUL, Feelings.ENERGETIC_JOYFUL}));
        subBrainAreas.put(EmotionalSBAreas.SADNESS_AREA,      new MiniEmotionalSBArea(this, 3, 5, new Feelings[]{Feelings.BORED_SAD, Feelings.LONELY_SAD, Feelings.DEPRESSED_SAD}));
        subBrainAreas.put(EmotionalSBAreas.SCAREDNESS_AREA,   new MiniEmotionalSBArea(this, 2, 5, new Feelings[]{Feelings.SUBMISSIVE_SCARED, Feelings.INSECURE_SCARED}));
        subBrainAreas.put(EmotionalSBAreas.MADNESS_AREA,      new MiniEmotionalSBArea(this, 2, 5, new Feelings[]{Feelings.SELFISH_MAD, Feelings.ANGREY_MAD}));
        
        // Associate the related neural nets
        subBrainAreas.get(EmotionalSBAreas.POWERFULNESS_AREA).getNeuralNet(Feelings.IMPORTANT_POWERFUL).pairNet(Feelings.FAITHFUL_POWERFUL);
        subBrainAreas.get(EmotionalSBAreas.PEACEFULNESS_AREA).getNeuralNet(Feelings.TRUSTING_PEACEFUL) .pairNet(Feelings.CONTENT_PEACEFUL);
        subBrainAreas.get(EmotionalSBAreas.JOYFULNESS_AREA)  .getNeuralNet(Feelings.ENERGETIC_JOYFUL)  .pairNet(Feelings.EXCITED_JOYFUL);
        subBrainAreas.get(EmotionalSBAreas.JOYFULNESS_AREA)  .getNeuralNet(Feelings.EXCITED_JOYFUL)    .pairNet(Feelings.CREATIVE_JOYFUL);
        subBrainAreas.get(EmotionalSBAreas.SADNESS_AREA)     .getNeuralNet(Feelings.LONELY_SAD)        .pairNet(Feelings.BORED_SAD);
        subBrainAreas.get(EmotionalSBAreas.SADNESS_AREA)     .getNeuralNet(Feelings.BORED_SAD)         .pairNet(Feelings.DEPRESSED_SAD);
        subBrainAreas.get(EmotionalSBAreas.SCAREDNESS_AREA)  .getNeuralNet(Feelings.INSECURE_SCARED)   .pairNet(Feelings.SUBMISSIVE_SCARED);
        subBrainAreas.get(EmotionalSBAreas.MADNESS_AREA)     .getNeuralNet(Feelings.ANGREY_MAD)        .pairNet(Feelings.SELFISH_MAD);
        
        // Associate the related sub-brain areas
        subBrainAreas.get(EmotionalSBAreas.POWERFULNESS_AREA).pairArea(EmotionalSBAreas.SCAREDNESS_AREA);
        subBrainAreas.get(EmotionalSBAreas.SCAREDNESS_AREA)  .pairArea(EmotionalSBAreas.POWERFULNESS_AREA);
        
        subBrainAreas.get(EmotionalSBAreas.JOYFULNESS_AREA)  .pairArea(EmotionalSBAreas.SADNESS_AREA);
        subBrainAreas.get(EmotionalSBAreas.SADNESS_AREA)     .pairArea(EmotionalSBAreas.JOYFULNESS_AREA);
        
        subBrainAreas.get(EmotionalSBAreas.PEACEFULNESS_AREA).pairArea(EmotionalSBAreas.MADNESS_AREA);
        subBrainAreas.get(EmotionalSBAreas.MADNESS_AREA)     .pairArea(EmotionalSBAreas.PEACEFULNESS_AREA);
        
        buildMajorFeelingToSubAreaMap();
    }
    
    @Override
    protected void function() {
        
        // 1) Excite targetted neural networks
        String feelingName, majorFeelingName;
        MiniEmotionalSBArea emotionalSBArea;
        
        feelingName = neuralNetToExcite.toString();
        majorFeelingName = feelingName.substring(feelingName.indexOf("_") + 1);
        emotionalSBArea = (MiniEmotionalSBArea)getEmotionalSubArea(majorFeelingName);
        
        emotionalSBArea.excite(new ExcitationSignal(neuralNetToExcite, neuralNetExStrength));
    
        // 2) Apply a random neural activity weakening characteristic to all 
        //    neurons in all nets
        Collection<SubBrainArea> subBrainAreasSet;
        Collection<NeuralNet> neuralNetsSet;
        List<Neuron> neuronsSet;
        
        subBrainAreasSet = subBrainAreas.values(); 
        
        for( SubBrainArea sbArea : subBrainAreasSet ) {
            neuralNetsSet = ((MiniEmotionalSBArea)sbArea).neuralNets.values();
            for( NeuralNet net : neuralNetsSet ) {
                neuronsSet = net.getNeurons();
                for( Neuron neuron : neuronsSet ) {
                    neuron.fadeNeuralContent(FastMath.rand.nextFloat() * 0.11f);
                }
            } 
        }
        
        
        System.out.println("===============================================");
        for(Feelings feel : getDominantEmotion().getFeelingsList()) {
            System.out.println("Feeling (after current trigger) ("+agentControl.getVehicleName() + "): "+ feel);
        }
        System.out.println("Strength of Dominant Emotion (current) ("+agentControl.getVehicleName() + "): "+ getDominantEmotion().getStrength());
        System.out.println("===============================================");
        
    }

    @Override
    public void excite(ExcitationSignal signal) {
        
        // Extract feelings and corresponding excitation strengthes from the 
        // signal
        neuralNetToExcite = (Feelings)signal.getExcitationNeuralNet();
        neuralNetExStrength = signal.getStrength();
        
        function();
        
    }
    
    /**
     * 
     * @return 
     */
    public Emotion getDominantEmotion() {
        
        float highestActLevel = 0, netActLevel;
        NeuralNet selectedNet = null, currNet;
        Pair<NeuralNet, Float> NetLevelPair;
        SubBrainArea sbArea;
        
        for( Entry<Enum, SubBrainArea> entry : subBrainAreas.entrySet() ) {
            
            sbArea = entry.getValue();
        
            NetLevelPair = sbArea.getNetWithHighestActivityLevel();
            currNet = NetLevelPair.key;
            netActLevel = NetLevelPair.val;
            
            highestActLevel = Math.max(highestActLevel, netActLevel); 
            if( netActLevel == highestActLevel ) {
                selectedNet = currNet;
            }
        }
        
        Emotion dominantEmotion = null;
        
        if( selectedNet != null ) {
            Feelings dominantFeeling = (Feelings)selectedNet.getName();
            dominantEmotion = new Emotion(new Feelings[]{dominantFeeling});
            dominantEmotion.setStrength(highestActLevel);
        }
        
        return dominantEmotion;
    }

    @Override
    protected void fire() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }
    
    private void buildMajorFeelingToSubAreaMap() {
        
        majorFeelingToSubAreaMap = new HashMap<String,EmotionalSBAreas>();
     
        majorFeelingToSubAreaMap.put("POWERFUL", EmotionalSBAreas.POWERFULNESS_AREA);
        majorFeelingToSubAreaMap.put("PEACEFUL", EmotionalSBAreas.PEACEFULNESS_AREA);
        majorFeelingToSubAreaMap.put("JOYFUL", EmotionalSBAreas.JOYFULNESS_AREA);
        majorFeelingToSubAreaMap.put("SAD", EmotionalSBAreas.SADNESS_AREA);
        majorFeelingToSubAreaMap.put("SCARED", EmotionalSBAreas.SCAREDNESS_AREA);
        majorFeelingToSubAreaMap.put("MAD", EmotionalSBAreas.MADNESS_AREA);

    }
    
    private EmotionalSBArea getEmotionalSubArea( String majorFeeling ) {
        return (EmotionalSBArea)subBrainAreas.get(majorFeelingToSubAreaMap.get(majorFeeling));
    }
    
    public EmotionalStrength getFeelingStrength(Feelings feeling) {
        
        String feelingName, majorFeelingName;
        EmotionalSBArea emotionalSBArea;
        
        feelingName = feeling.toString();
        majorFeelingName = feelingName.substring(feelingName.indexOf("_") + 1);
        emotionalSBArea = getEmotionalSubArea(majorFeelingName);
        
        float emotionalStrength = emotionalSBArea.getNeuralNet(feeling).getNeuralActivityLevel();
        
        if( emotionalStrength >= 0.8 * MAX_EMOTIONAL_STRENGTH ) {
            return EmotionalStrength.VERY_HIGH;
        } else if( emotionalStrength >= 0.6 * MAX_EMOTIONAL_STRENGTH ) {
            return EmotionalStrength.HIGH;
        } else if( emotionalStrength >= 0.4 * MAX_EMOTIONAL_STRENGTH ) {
            return EmotionalStrength.MEDIUM;
        } else if( emotionalStrength >= 0.2 * MAX_EMOTIONAL_STRENGTH ) {
            return EmotionalStrength.LOW;
        } else {
            return EmotionalStrength.VERY_LOW;
        }  
    }
    
}

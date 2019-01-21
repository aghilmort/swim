package swim.algorithm.common;

import com.jme3.math.Vector3f;
import java.util.ArrayList;
import java.util.HashMap;
import swim.core.motion.CompositeTurnSequence;
import swim.core.motion.MotionGoal;
import swim.sim.uwswarm.intelli.brain.struct.artif.Duration;
import swim.util.Pair;

/**
 *
 * @author Sherif
 */
public class AlgorithmState {
    
    // =========================================================================
    // Constants
    // =========================================================================
    private final int DEFAULT_INIT_CAPACITY = 5;
    private final int SMALL_DEFAULT_INIT_CAPACITY = 2;
    // =========================================================================
    // Variables
    // =========================================================================
    protected HashMap<String, Integer> intVars;
    protected HashMap<String, Float> floatVars;
    protected HashMap<String, Long> longVars;
    protected HashMap<String, Boolean> flagVars;
    
    protected HashMap<String, String> stringVars;
    protected HashMap<String, String[]> stringArrays;
    
    protected HashMap<String, CompositeTurnSequence> sequences;
    
    protected HashMap<String, Vector3f> vectors;
    protected HashMap<String, Vector3f[]> vectorArrays;
    
    
    protected Duration duration;
    protected MotionGoal goal;
    protected ArrayList<Pair<String, Object>> compositeTurnSeq;
    // =========================================================================
    
    public AlgorithmState() {}
    
    public int getIntVar(String name) {
        if( intVars == null ) { throw new NullPointerException(); }
        return intVars.get(name);
    }
    
    public float getFloatVar(String name) {
        if( floatVars == null ) { throw new NullPointerException(); }
        return floatVars.get(name);
    }
    
    public long getLongVar(String name) {
        if( longVars == null ) { throw new NullPointerException(); }
        return longVars.get(name);
    }
    
    public boolean getFlagVar(String name) {
        if( flagVars == null ) { throw new NullPointerException(); }
        return flagVars.get(name);
    }
    
    public CompositeTurnSequence getSequence(String name) {
        if( sequences == null ) { throw new NullPointerException(); }
        return sequences.get(name);
    }
    
    public Vector3f getVectorVar(String name) {
        if( vectors == null ) { throw new NullPointerException(); }
        return vectors.get(name);
    }

    public void storeIntVar(String name, Integer value) {
        if( intVars == null ) {
            intVars = new HashMap<String, Integer>(DEFAULT_INIT_CAPACITY);
        }
        intVars.put(name, value);
    }
    
    public void storeFloatVar(String name, Float value) {
        if( floatVars == null ) {
            floatVars = new HashMap<String, Float>(DEFAULT_INIT_CAPACITY);
        }
        floatVars.put(name, value);
    }
    
    public void storeLongVar(String name, Long value) {
        if( longVars == null ) {
            longVars = new HashMap<String, Long>(DEFAULT_INIT_CAPACITY);
        }
        longVars.put(name, value);
    }
    
    public void storeFlagVar(String name, boolean value) {
        if( flagVars == null ) {
            flagVars = new HashMap<String, Boolean>(DEFAULT_INIT_CAPACITY);
        }
        flagVars.put(name, value);
    }

    public void storeSequence(String name, CompositeTurnSequence sequence) {
        if( sequences == null ) {
            sequences = new HashMap<String, CompositeTurnSequence>(DEFAULT_INIT_CAPACITY);
        }
        sequences.put(name, sequence);
    }
    
    public void storeVectorVar(String name, Vector3f vector) {
        if( vectors == null ) {
            vectors = new HashMap<String, Vector3f>(DEFAULT_INIT_CAPACITY);
        }
        vectors.put(name, vector);
    } 

    public Duration getDuration() {
        return duration;
    }

    public void storeDuration(Duration duration) {
        this.duration = duration;
    }

    public MotionGoal getGoal() {
        return goal;
    }

    public void storeGoal(MotionGoal goal) {
        this.goal = goal;
    }

    public Vector3f[] getVectorArray(String name) {
        if( vectorArrays == null ) { throw new NullPointerException(); }
        return vectorArrays.get(name);
    }
    
    public String[] getStringArray(String name) {
        if( stringArrays == null ) { throw new NullPointerException(); }
        return stringArrays.get(name);
    }

    public void storeVectorArray(String name, Vector3f[] vectorArray) {
        if( vectorArrays == null ) {
            vectorArrays = new HashMap<String, Vector3f[]>(SMALL_DEFAULT_INIT_CAPACITY);
        }
        this.vectorArrays.put(name, vectorArray);
    }
    
    public void storeStringArray(String name, String[] stringArray) {
        if( stringArrays == null ) {
            stringArrays = new HashMap<String, String[]>(SMALL_DEFAULT_INIT_CAPACITY);
        }
        this.stringArrays.put(name, stringArray);
    }
    
    public void storeStringVar(String name, String string) {
        if( stringVars == null ) {
            stringVars = new HashMap<String, String>(DEFAULT_INIT_CAPACITY);
        }
        this.stringVars.put(name, string);
    }
    
    public String getStringVar(String name) {
        if( stringVars == null ) { throw new NullPointerException(); }
        return stringVars.get(name);
    }

    public ArrayList<Pair<String, Object>> getCompositeTurnSeq() {
        return compositeTurnSeq;
    }

    public void storeCompositeTurnSeq(ArrayList<Pair<String, Object>> compositeTurnSeq) {
        this.compositeTurnSeq = compositeTurnSeq;
    }
    
    
    
}

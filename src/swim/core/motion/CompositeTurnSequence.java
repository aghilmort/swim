/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.core.motion;

import java.util.ArrayList;
import swim.util.Pair;

/**
 *
 * @author Sherif
 */
public class CompositeTurnSequence extends ArrayList<Pair<String, Object>> {
    
    private String name;
    private float cachedRandAngle = 0,
                  cachedRandDist = 0,
                  sequenceScore = 0;
    
    public void add(String description, Object specification) {
        super.add(new Pair<String,Object>(description, specification));
    }
  
    public Pair<String, Object> getSequence(int i) {
        return this.get(i);
    }
    
    public void setName(String name) {
        this.name = name;
    }
    
    public String getName() {
        return name;
    }
    
    public void setRandAngle(float angle) {
        this.cachedRandAngle = angle;
    }
    
    public float getCachedRandAngle() {
        return cachedRandAngle;
    }
    
    public void setRandDist(float distance) {
        this.cachedRandDist = distance;
    }
    
    public float getCachedRandDist() {
        return cachedRandDist;
    }
    
    public void setScore(float score) {
        this.sequenceScore = score;
    }
    
    public float getScore() {
        return this.sequenceScore;
    }
    
    public void addSequenceSet(CompositeTurnSequence sequenceSet) {
        
        for( int i = 0; i < sequenceSet.size(); ++i ) {
            super.add(sequenceSet.getSequence(i));
        }
    }
}

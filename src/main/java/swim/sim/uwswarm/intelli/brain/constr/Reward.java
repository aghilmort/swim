/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.constr;

/**
 *
 * @author Sherif
 * @param <Type>
 */
public class Reward<Type> implements BrainConstruct {
    
    private RewardSpan span;
    private Enum type;
    private RewardSize rewardSize;
    private String name = "Reward";
    
    private boolean shortTermReward = true;
    
    public Reward(RewardSpan span, Type type) {
        
      this.span = span;  
        
      if( type instanceof ShortTermRewardType && span.equals(RewardSpan.SHORT_TERM) ) {

          this.type = (ShortTermRewardType)type;
          
      } else if( type instanceof LongTermRewardType && span.equals(RewardSpan.LONG_TERM) ) {
          
          shortTermReward = false;
          this.type = (LongTermRewardType)type;
          
      } else {
          throw new IllegalArgumentException(
                  "A short-term reward can only be of ShortTermRewardType type and a"
                + " long-term reward can only be of LongTermRewardType type");
      }
      
    }

    public Enum getType() {
        return ( shortTermReward ?  (ShortTermRewardType)this.type : (LongTermRewardType)this.type );
    }
    
    public RewardSpan getSpan() {
        return this.span;
    }
    
    public void setSize(RewardSize rewardSize) {
        this.rewardSize = rewardSize;
    }
    
    public RewardSize getSize() {
        return (RewardSize)this.rewardSize;
    }
    
    @Override
    public String getConstructName() {
        return name;
    }

    @Override
    public void setConstructName(String name) {
        this.name = name;
    }
    
}

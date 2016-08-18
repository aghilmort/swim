/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm;

import java.util.HashMap;
import java.util.Iterator;
import swim.core.Agent;

/**
 *
 * @author Sherif
 */
public class Swarms implements Iterable {
    
    private HashMap<String, Swarm> swarmsList;
    private int number;
    
    public Swarms() {
        this(1);    // Default: one swarm
    }
    
    public Swarms(int numSwarms) {
        swarmsList = new HashMap<String, Swarm>(numSwarms);
    }

    public Iterator iterator() {
        return swarmsList.entrySet().iterator();
    }
    
    public String addSwarm(HashMap<String, Agent> vehicles) {
        Swarm swarm = new Swarm(vehicles);
        swarmsList.put(swarm.getID(), swarm);
        return swarm.getID();
    }
    
    public Swarm getSwarm(String ID) {
        return swarmsList.get(ID);
    }

     public int getNumber() {
         return number;
     }
            
    public class Swarm implements Iterable {
        
        private HashMap<String, Agent> agents;
        private int swarmSize = 0;
        private String id;
        
        public Swarm() {
            this.agents = new HashMap<String, Agent>();
            id = "swarm_" + (number + 1);
        }
        
        public Swarm(HashMap<String, Agent> agents) {
            this.agents = agents;
            swarmSize = agents.size();
            id = "swarm_" + (number + 1);
        }
        
        public void addAgent(Agent agent) {
           agents.put(agent.getName(), agent);
           swarmSize++;
        }
        
        public int getSize() {
            return swarmSize;
        }
        
        public void excludeAgent(String name) {
           agents.remove(name);
        }

        public Iterator iterator() {
            return agents.entrySet().iterator();
        }
        
        public String getID() {
            return id;
        }
    }
    
}

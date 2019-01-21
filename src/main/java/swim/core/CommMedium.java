/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.core;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Vector;
import java.util.logging.Level;
import java.util.logging.Logger;
import swim.core.network.UWSensorNetwork;
import swim.sim.Simulator;

/**
 *
 * @author Sherif
 */
public class CommMedium /*implements Runnable*/ {
    
    // Constants - START
    private final boolean COMM_MEDIUM_DEBUG = false,
                          DEBUG_PSO_SEARCH = false,
                          DEBUG_CSF_SEARCH = false;
 
    // Constants - END
    
    //HashMap<String, HashMap<String, MessageBuffer<Message>>> toMessages;
    HashMap<String, HashMap<String, MessageBuffer<Message>>> fromMessages;
    
    private HashMap<Long, UWSensorNetwork> networks;
    
    //private HashMap<String, HashMap<String, Float>> medium;
    
    private final Simulator sim;
    
    private Integer numMsgsInMedium = 0;
    
    private final String[] vehicleNames;
    
    private final int numAgents;
    
    private HashMap<String, MessageBuffer<Message>> tempHashMap;
    
    public CommMedium (Simulator sim) {
        
        this.sim = sim;
        networks = new HashMap<Long, UWSensorNetwork>();
        numAgents = Simulator.getNumAgents();
        
        //toMessages = new HashMap<String, HashMap<String, MessageBuffer<Message>>>(Simulator.getNumAgents());
        fromMessages = new HashMap<String, HashMap<String, MessageBuffer<Message>>>(numAgents);
        
        tempHashMap = new HashMap<String, MessageBuffer<Message>>(numAgents);
        
        String vehName;
        vehicleNames = sim.getVehicleNames();
              
        for(int i = 0; i < numAgents; i++) {
            vehName = vehicleNames[i];
            //toMessages.put(vehName, (HashMap<String, MessageBuffer<Message>>)tempHashMap.clone());
            fromMessages.put(vehName, (HashMap<String, MessageBuffer<Message>>)tempHashMap.clone());
            //fromMessages.put(vehName, new HashMap<String, MessageBuffer<Message>>(numAgents));
            for(int j = 0; j < numAgents; j++) {
                //toMessages.get(vehName).put(sim.getVehicleNames()[j], new MessageBuffer<Message>());
                fromMessages.get(vehName).put(vehicleNames[j], new MessageBuffer<Message>());
                //fromMessages.get(vehName).put(vehicleNames[j], new MessageBuffer<Message>());
            }
        }
    }
    
    public /*synchronized*/ void addNetwork(UWSensorNetwork network, Long networkID) {
        networks.put(networkID, network);
    }
    
    public /*synchronized*/ UWSensorNetwork getNetwork(Long networkID) {
        return networks.get(networkID);
    }
    
    public /*synchronized*/ void removeNetwork(Long networkID) {
        networks.remove(networkID);
    }
    
    /**
     * Structure is as follows:<br />
     * <pre>toMessages = {
     *                to1 =  {from = message1, from2 = message2, ...},
     *                to2 =  {from = message1, from2 = message2, ...},
     *                ... }</pre>
     * @param message the message to be sent
     */
    public /*synchronized*/ void conveyMessage(Message message) {
        
        Integer from = (Integer)message.getHeader("From");
        
        Long netID = (Long)message.getHeader("NetworkID");
        
        String fromName = getNetwork(netID).getName(from);
        String toName;
        
        UWSensorNetwork net = (UWSensorNetwork)getNetwork(netID);
        Object dst;
        Integer to;
        
        MsgTransmitType msgType = (MsgTransmitType)message.getHeader("Type");
        
        switch(msgType) {
            case unicast:
                
                dst = (Integer)message.getHeader("To");
                to = (Integer)dst;
                
                toName = (String)getNetwork(netID).getName(to);

                if( net.linkExists(from, to) ) {
//                    if( toMessages.get(toName) == null ) {
//                        toMessages.put(toName, new HashMap<String, MessageBuffer<Message>>());
//                        toMessages.get(toName).put(fromName, new MessageBuffer<Message>());
//                    }
                    if( fromMessages.get(fromName).get(toName).size() < MessageBuffer.DEFAULT_BUFF_SIZE ) {
                        try {
                            fromMessages.get(fromName).get(toName).put(message);
                            numMsgsInMedium++;
                        } catch (InterruptedException ex) {
                            Logger.getLogger(CommMedium.class.getName()).log(Level.SEVERE, "Failed to store message", ex);
                        }
                        
                    } else {
                        if(COMM_MEDIUM_DEBUG) {
                            System.out.println("Message dropped");
                        }
                    }
                    
                } else {
                    if(COMM_MEDIUM_DEBUG) {
                        System.out.println("No link exists between "+from+" and "+to+"!");
                    }
                    System.exit(-1);
                }
                break;
                
            case broadcast:
                
                dst = (Vector<Integer>)message.getHeader("To");
                
                Iterator destItr = ((Vector<Integer>)dst).iterator();

                while(destItr.hasNext()) {
                    to = (Integer)destItr.next();
                    toName = (String)getNetwork(netID).getName(to);
                    
                    //System.out.println("Medium Accessed by: "+fromName+ " to send message to "+toName);
                    
                    if( net.linkExists(from, to) ) {
//                        if( toMessages.get(toName) == null ) {
//                            toMessages.put(toName, new HashMap<String, MessageBuffer<Message>>());
//                            toMessages.get(toName).put(fromName, new MessageBuffer<Message>());
//                        }

                        
                        
                        if( fromMessages.get(fromName).get(toName).size() < MessageBuffer.DEFAULT_BUFF_SIZE ) {
                            try {
                                fromMessages.get(fromName).get(toName).add(message);
                                //System.out.println("Message should have been stored in medium now ...");
                                if(COMM_MEDIUM_DEBUG) {
                                    if(DEBUG_PSO_SEARCH) {
                                        System.out.println("Message in medium: From: "+fromName+", To: "+toName+", Self-Best Pos: "+message.getData("SELF_BEST_POINTER")+", Self-Best Sol: "+message.getData("SELF_BEST_SOLUTION"));
                                    } else if(DEBUG_CSF_SEARCH) {
                                        System.out.println("Message in medium: From: "+fromName+", To: "+toName+", Content: "+message.getData("TARGET_FOUND"));
                                    }
                                }
                                numMsgsInMedium++;
                            } catch(IllegalStateException ex) {
                                System.out.println("Failed to add message to buffer!");
                            }
                            
                        } else {
                            //System.out.println("Message dropped");
                        }
                            
                        
                    } else {
                        if(COMM_MEDIUM_DEBUG) {
                            System.out.println("No link exists between "+from+" and "+to+"!");
                        }
                        System.exit(-1);
                    }
                }        
                
                
                
                //System.out.println("Medium entry for 'from' ("+fromName+"): "+fromMessages.get(fromName).toString());
                
                break;
                
            case multicast:
                //TODO: future work ... implement multicast
                break;
        }
        
    }
    
    public /*synchronized*/ Integer getInMediumMsgNumber() {
        return numMsgsInMedium;
    }
    
    public /*synchronized*/ Message getMessage(String from, String to) {
        
        //System.out.println("Medium accessed by "+to+" to get message from: "+from);
        
        //Message msg = (Message)toMessages.get(to).get(from).poll();

        if(COMM_MEDIUM_DEBUG) {
            System.out.println("Medium entry 'from' ("+from+"): "+fromMessages.get(from).toString());
        }
        
        Message msg = (Message)fromMessages.get(from).get(to).poll();

        if(COMM_MEDIUM_DEBUG && DEBUG_CSF_SEARCH && msg != null) {
            System.out.println("Message in medium: From: "+from+", To: "+to+", Vehicle that found target: "+msg.getData("TARGET_FOUND"));
        }
        
        if(COMM_MEDIUM_DEBUG && DEBUG_PSO_SEARCH && msg != null) {
            System.out.println("Message in medium: From: "+from+", To: "+to+", Self-Best Pos: "+msg.getData("SELF_BEST_POINTER")+", Self-Best Sol: "+msg.getData("SELF_BEST_SOLUTION"));
        }
        //System.out.println("Returned message is: "+ (msg == null ? "NULL" : msg.toString()));
        
        
        return msg;
//        try {
//            
//            //System.out.println("Here is the message: "+msg);
//            return msg;
//        } catch(Exception ex) {
//            if( !(ex instanceof NullPointerException) ) {
//                System.out.println(ex.getMessage());
//                System.exit(-1);
//            } else {
//                System.out.println("Null Pointer encountered!");
//            }
//            return null;
//        } 
    }

//    @Override
//    public void run() {
//    }
}

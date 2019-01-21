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
public class CommChannel {
    
    //HashMap<String, HashMap<String, MessageBuffer<Message>>> toMessages;
    HashMap<String, HashMap<String, MessageBuffer<Message>>> fromMessages;
    
    private final Simulator sim;
    
    private Integer numMsgsInMedium = 0;
    
    private UWSensorNetwork network;
    
    private int numVehicles;
    
    public CommChannel (UWSensorNetwork network, Simulator sim) {
        
        this.sim = sim;
        this.network = network;
        numVehicles = network.getNodeCount();
        
        //toMessages = new HashMap<String, HashMap<String, MessageBuffer<Message>>>(Simulator.getNumAgents());
        fromMessages = new HashMap<String, HashMap<String, MessageBuffer<Message>>>(numVehicles);
        
        String vehName;
              
        for(int i = 1; i <= numVehicles; i++) {
            vehName = this.network.getName(i);
            //toMessages.put(vehName, (HashMap<String, MessageBuffer<Message>>)tempHashMap.clone());
            fromMessages.put(vehName, new HashMap<String, MessageBuffer<Message>>(numVehicles));
            for(int j = 1; j <= numVehicles; j++) {
                //toMessages.get(vehName).put(sim.getVehicleNames()[j], new MessageBuffer<Message>());
                fromMessages.get(vehName).put(this.network.getName(j), new MessageBuffer<Message>());
            }
        }
    }

    /**
     * Structure is as follows:<br />
     * <pre>toMessages = {
     *                to1 =  {from = message1, from2 = message2, ...},
     *                to2 =  {from = message1, from2 = message2, ...},
     *                ... }</pre>
     * @param message the message to be sent
     */
    public synchronized void conveyMessage(Message message) {
        
        Integer from = (Integer)message.getHeader("From");
        
        Long netID = (Long)message.getHeader("NetworkID");
        
        System.out.println("Network: "+netID);
        
        String fromName = network.getName(from);
        String toName;
        
        Object dst;
        Integer to;
        
        MsgTransmitType msgType = (MsgTransmitType)message.getHeader("Type");
        
        switch(msgType) {
            case unicast:
                
                dst = (Integer)message.getHeader("To");
                to = (Integer)dst;
                
                toName = (String)network.getName(to);

                if( network.linkExists(from, to) ) {
//                    if( toMessages.get(toName) == null ) {
//                        toMessages.put(toName, new HashMap<String, MessageBuffer<Message>>());
//                        toMessages.get(toName).put(fromName, new MessageBuffer<Message>());
//                    }
                    if( fromMessages.get(fromName).get(toName).size() < MessageBuffer.DEFAULT_BUFF_SIZE ) {
                        try {
                            fromMessages.get(fromName).get(toName).put(message);
                            numMsgsInMedium++;
                        } catch (InterruptedException ex) {
                            Logger.getLogger(CommMedium.class.getName()).log(Level.SEVERE, null, ex);
                        }
                        
                    } else {
                        System.out.println("Message dropped");
                    }
                    
                } else {
                    System.out.println("No link exists between "+from+" and "+to+"!");
                    System.exit(-1);
                }
                break;
                
            case broadcast:
                
                dst = (Vector<Integer>)message.getHeader("To");
                
                Iterator destItr = ((Vector<Integer>)dst).iterator();

                System.out.println("fromMessages: "+fromMessages.toString());
                
                while(destItr.hasNext()) {
                    to = (Integer)destItr.next();
                    toName = (String)network.getName(to);
                    
                    System.out.println("Medium Accessed by: "+fromName+ " to send message to "+toName);
                    
                    //System.out.println("Look here: from ==> "+fromName+", to ==> "+toName);
                    
                    //System.out.println("To: "+to);
                    //System.out.println("ToName: "+toName);
                    
                    if( network.linkExists(from, to) ) {
//                        if( toMessages.get(toName) == null ) {
//                            toMessages.put(toName, new HashMap<String, MessageBuffer<Message>>());
//                            toMessages.get(toName).put(fromName, new MessageBuffer<Message>());
//                        }

                        
                        
                        if( fromMessages.get(fromName).get(toName).size() < MessageBuffer.DEFAULT_BUFF_SIZE ) {
                            try {
                                fromMessages.get(fromName).get(toName).add(message);
                                System.out.println("Message should have been stored in medium now ...");
                                numMsgsInMedium++;
                            } catch(IllegalStateException ex) {}
//                            } catch (InterruptedException ex) {
//                                Logger.getLogger(CommMedium.class.getName()).log(Level.SEVERE, null, ex);
//                                
//                            } 
                            //System.out.println("CHECK: "+toMessages.get(toName).toString());
                            
                            
                        } else {
                            System.out.println("Message dropped");
                        }
                            
                        
                    } else {
                        System.out.println("No link exists between "+from+" and "+to+"!");
                        System.exit(-1);
                    }
                }        
                
                
                
                System.out.println("Medium entry for 'from' ("+fromName+"): "+fromMessages.get(fromName).toString());
                
                break;
                
            case multicast:
                //TODO: future work ... implement multicast
                break;
        }
        
    }
    
    public synchronized Integer getInChannelMsgNumber() {
        return numMsgsInMedium;
    }
    
    public synchronized Message getMessage(String from, String to) {
        
        System.out.println("Medium accessed by "+to+" to get message from: "+from);
        
//        System.out.println("Message from "+from+" to "+to);
//        System.out.println(toMessages.toString());
        
        //System.out.println("To: "+to);
        //System.out.println(toMessages.get(to).toString());
        //System.out.println(toMessages.toString());
        //System.out.println(toMessages.toString());
        
        //Message msg = (Message)toMessages.get(to).get(from).poll();
        
        System.out.println("Medium entry for 'from' ("+from+"): "+fromMessages.get(from).toString());
        Message msg = (Message)fromMessages.get(from).get(to).poll();
        
        
        
        System.out.println("Returned message is: "+ (msg == null ? "NULL" : msg.toString()));
        
        
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
}

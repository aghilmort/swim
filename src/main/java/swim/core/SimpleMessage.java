/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.core;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map.Entry;

/**
 *
 * @author Sherif
 */
public class SimpleMessage extends Message {
    
     public SimpleMessage() {
         super();
     }
    
     
    /**
     * 
     * @param from    message sender
     * @param type    unicast, broadcast, or multicast
     * @param headers additional headers (should at least contain the 
     *                receiver(s) of the message
     * 
     * Source: http://serverfault.com/questions/279482/what-is-the-difference-
     * between-unicast-anycast-broadcast-and-multicast-traffic
     * ------------------------------------------------------------
     * | TYPE      | ASSOCIATIONS     | SCOPE           | EXAMPLE |
     * ------------------------------------------------------------
     * | Unicast   | 1 to 1           | Whole network   | HTTP    | 
     * ------------------------------------------------------------
     * | Broadcast | 1 to Many        | Subnet          | ARP     |
     * ------------------------------------------------------------
     * | Multicast | One/Many to Many | Defined horizon | SLP     |
     * ------------------------------------------------------------
     * | Anycast   | Many to Few      | Whole network   | 6to4    |
     * ------------------------------------------------------------
     */
    public SimpleMessage(HashMap<String, Object> headers) {

        Iterator headersItr = headers.entrySet().iterator();
        Entry<String, String> header;
        
        while(headersItr.hasNext()) {
            header = (Entry<String, String>) headersItr.next();
            addHeader(header.getKey(), header.getValue()); 
        }
        
        if( getHeader("To") == null ) {
            System.out.println("Message destination not set!");
            System.exit(-1);
        }
        
        if( getHeader("From") == null ) {
            System.out.println("Message source not set!");
            System.exit(-1);
        }
        
        if( getHeader("Type") == null ) {
            System.out.println("Message type not set!");
            System.exit(-1);
        }
    }    
 
}

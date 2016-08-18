/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.core;

import java.util.concurrent.ArrayBlockingQueue;

/**
 *
 * @author Sherif
 */
public class MessageBuffer<E> extends ArrayBlockingQueue<E> {
    
    public final static int DEFAULT_BUFF_SIZE = 5;
    
    public MessageBuffer() {
        super(DEFAULT_BUFF_SIZE);
    }
    
    public MessageBuffer(int capacity) {
        super(capacity);
    }
    
    public MessageBuffer(int capacity, boolean fair) {
        super(capacity, fair);
    }
    
    

}

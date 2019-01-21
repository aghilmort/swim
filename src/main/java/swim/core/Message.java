/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.core;

import java.util.HashMap;
import swim.exception.HeadersNotSetException;

/**
 *
 * @author Sherif
 */
public abstract class Message {
    
    protected HashMap<String, Object> headers;
    protected HashMap<String, Object> data;
    
    public Message() {
        data = new HashMap<String, Object>();
        headers = new HashMap<String, Object>();
    }
    
    public void send() throws HeadersNotSetException {
        if(headers == null) {
            throw new HeadersNotSetException();
        }
    }
    
    public void addHeader(String headerName, Object headerValue) {
        headers.put(headerName, headerValue);
    }

    public void addHeaders(HashMap<String, Object> headers) {
        this.headers = headers;
    }
    
    public Object getHeader(String headerName) {
        return headers.get(headerName);
    }
    
    public HashMap<String, Object> getHeaders() {
        return headers;
    }
    
    public void addData( HashMap<String, Object> data) {
        this.data = data;
    }
    
    public void addData( String descriptor, Object data) {
        this.data.put(descriptor, data);
    }
    
    public Object getData(String dataDescriptor) {
        return data.get(dataDescriptor);
    }
    
    public HashMap<String, Object> getData() {
        return data;
    }
    
    
}

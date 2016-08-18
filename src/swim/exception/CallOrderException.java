/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.exception;

/**
 *
 * @author Sherif
 */
public class CallOrderException extends Exception {
   
    private String message = "";
    
    public CallOrderException(String message) {
        this.message = message;
    }
    
    @Override
    public String getMessage() {
        super.getMessage();
        return (message.equals("") ? "Method called in wrong order. Check method for expected call order!" :
                "Method called in wrong order: "+message);
        
    } 
}

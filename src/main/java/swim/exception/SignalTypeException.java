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
public class SignalTypeException extends Exception {
    
    private String message = "";
    
    public SignalTypeException(String message) {
        this.message = message;
    }
    
    @Override
    public String getMessage() {
        super.getMessage();
        return (message.equals("") ? "Wrong signal type used to excite brain area. Check 'excite' method for correct signal type!" :
                "Wrong signal type used to excite brain area: "+message);
        
    } 
    
}

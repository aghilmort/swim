/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.exception;

/**
 *
 * @author Sherif
 */
public class HeadersNotSetException extends Exception {
    
    @Override
    public String getMessage() {
        super.getMessage();
        return "Massage headers have not been set!";
        
    }
    
}

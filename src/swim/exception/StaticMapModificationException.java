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
public class StaticMapModificationException extends Exception {
    
    @Override
    public String getMessage() {
        super.getMessage();
        return "Attempt to modify a built static map!";
        
    } 
}

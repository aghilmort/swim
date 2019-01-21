/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.util.math;

/**
 *
 * @author Sherif
 */
public class SwimMath {
    
    public static float gaussian(float x, float mu, float sigma, float amplitude) {
        
        float A, B;
        
        A = (float)( 1 / (sigma * Math.sqrt(2*Math.PI)) );
        B = (float)(Math.exp(-(Math.pow(x - mu, 2)/(2*Math.pow(sigma, 2)))));
        
        return amplitude * A * B;
        
    }
    
}

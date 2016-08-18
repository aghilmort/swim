/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.core;

import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.control.GhostControl;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import java.util.concurrent.Callable;
import java.util.concurrent.Future;
import java.util.logging.Level;
import java.util.logging.Logger;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public class SensorControl extends GhostControl {
    
    // Multi-threading
    //The future that is used to check the execution status:
    private Future future = null;
    
    private Simulator sim;
    private float tpfCopy;
    private SensorControl controlRef;
    
    public SensorControl(Simulator sim) {
        super();
        this.sim = sim;
        
    }
    
    public SensorControl(CollisionShape sensingPattern, Simulator sim) {
        super(sensingPattern);
        this.sim = sim;
    }
    
    @Override
    public void update(float tpf) {
        
        tpfCopy = tpf;
        controlRef = this;
        
        String result = "";

        try {
             //If we have not started a callable yet, do so!
            if(future == null){

                //start the callable on the executor
                future = sim.getExecutor().submit(updateCollisions);    //  Thread starts!

            }else if(future != null){
                //Get the waylist when its done
                if(future.isDone()){
                    result = (String)future.get();
                    future = null;
                }
                else if(future.isCancelled()){
                    //Set future to null. Maybe we succeed next time...
                    future = null;
                }
            }

        } catch (Exception ex) {
            Logger.getLogger(UWVehicleControl.class.getName()).log(Level.SEVERE, null, ex);
        }
//            
            //System.out.println("Result: " + result);
            
//            move(tpf);
            
//            if( vehicle.getSensorReading() > 0 ) {
//                System.out.println("=============================================");
//                System.out.println("Num Particles: " + vehicle.getSensorReading());
//                System.out.println("=============================================");
//            }
            

        
    }
    
    private void superUpdate(float tpf) {
        super.update(tpfCopy);
    }
    
    private Callable<String> updateCollisions = new Callable<String>(){
        public String call() throws Exception {
            controlRef.superUpdate(tpfCopy);
            return "Checked Collision";
        }
    };    
    
}

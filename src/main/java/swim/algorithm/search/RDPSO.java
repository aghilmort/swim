/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.algorithm.search;

import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public class RDPSO extends SearchAlgorithm {

    public RDPSO(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
    }

    protected void search() {
        
    }

    protected boolean targetFound() {
        return true;
    }

    public void setMaxIterations(int maxNumIter) {
    }

    @Override
    protected void saveState() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    protected void restoreState() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

}

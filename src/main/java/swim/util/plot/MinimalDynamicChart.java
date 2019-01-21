package swim.util.plot;

import info.monitorenter.gui.chart.Chart2D;
//import info.monitorenter.gui.chart.IAxis;
import info.monitorenter.gui.chart.ITrace2D;
import info.monitorenter.gui.chart.traces.Trace2DLtd;
//import info.monitorenter.gui.chart.views.ChartPanel;

import java.awt.Color;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JFrame;

/**
 * Demonstrates minimal effort to create a dynamic chart.
 * 
 * @author Achim Westermann
 * 
 */

public class MinimalDynamicChart {
  
  public static void main(String[]args){
    // Create a chart:  
    Chart2D chart = new Chart2D();
    // Create an ITrace: 
    // Note that dynamic charts need limited amount of values!!! 
    final ITrace2D trace = new Trace2DLtd(200); 
    trace.setColor(Color.RED);
 
    // Add the trace to the chart. This has to be done before adding points (deadlock prevention): 
    chart.addTrace(trace);
    
    // Make it visible:
    // Create a frame. 
    JFrame frame = new JFrame("MinimalDynamicChart");
    // add the chart to the frame: 
    frame.getContentPane().add(chart);
    frame.setSize(400,300);
    // Enable the termination button [cross on the upper right edge]: 
    frame.addWindowListener(
        new WindowAdapter(){
          public void windowClosing(WindowEvent e){
              System.exit(0);
          }
        }
      );
    frame.setVisible(true); 
   
    /* 
     * Now the dynamic adding of points. This is just a demo! 
     * 
     * Use a separate thread to simulate dynamic adding of date. 
     * Note that you do not have to copy this code. Dynamic charting is just about 
     * adding points to traces at runtime from another thread. Whenever you hook on 
     * to a serial port or some other data source with a polling Thread (or an event 
     * notification pattern) you will have your own thread that just has to add points 
     * to a trace. 
     */

    Timer timer = new Timer(true);
    TimerTask task = new TimerTask(){

      private double m_y = 0;
      private long m_starttime = System.currentTimeMillis();
      /**
       * @see java.util.TimerTask#run()
       */
      @Override
      public void run() {
        // This is just computation of some nice looking value.
        double rand = Math.random();
        boolean add = (rand >= 0.5) ? true : false;
        this.m_y = (add) ? this.m_y + Math.random() : this.m_y - Math.random();
        // This is the important thing: Point is added from separate Thread.
        trace.addPoint(((double) System.currentTimeMillis() - this.m_starttime), this.m_y);
      }
      
    };
    // Every 20 milliseconds a new value is collected.
    timer.schedule(task, 1000, 20);
  }

  /** Defcon. */
  private MinimalDynamicChart() {
    // nop
  }
}
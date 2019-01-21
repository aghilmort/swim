/**
 * 
 */
package swim.core.network.misc.geometry;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;


/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class Geometry {
	
	public final static double inf = Double.POSITIVE_INFINITY;
	public final static double epsilon = 1e-6;
	
	private static String fileName = "drawing.sci";
	
	public Geometry(){
		Phi.thePhi = new Phi();
	}
	
	public static void printDrawing(Object obj, String fileName) throws IOException{
		
		File drawingFile = new File(fileName);
		
		drawingFile.createNewFile();
		
		Drawing d = new Drawing(drawingFile, 0, 0, 600, 600);
		
		d.draw(obj);
		
	}
	
	public static void printDrawing(Object obj) throws IOException{
		
		printDrawing(obj, fileName);
		
	}
	
	public static int solveQuadratic(double a, double b, double c, double x1, double x2){
		
        double D = Math.pow(b, 2) - 4*a*c;
        if(Math.abs(D) < epsilon)
        {
            x1 = x2 = -b/2.0/a;
            return 1;
        }
        else if(D < 0)
        {
            return 0;
        }
        else
        {
            double d = Math.sqrt(D);
            x1 = (- b - d)/2/a;
            x2 = (- b + d)/2/a;

            return 2;
        }
	}
	
}

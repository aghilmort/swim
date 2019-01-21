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
public class Drawing {

	File outFile;
	FileWriter out;
	public final double xMin, xMax, yMin, yMax;
	
	public Drawing(File outFile, double xMin, double xMax, double yMin, double yMax) throws IOException{
		this.outFile = outFile;
		this.xMin = xMin;
		this.xMax = xMax;
		this.yMin = yMin;
		this.yMax = yMax;
		
		out = new FileWriter(outFile);
		String str = String.format("clf();\nplot2d(0,0,-1,\"031\", \" \", [%f,%f,%f,%f]);\n", xMin, yMin, xMax, yMax);
		out.write(str);	
	}
	
	public Drawing draw(Object obj) throws IOException{
		obj.draw(this);
		return this;
	}
	
	public Drawing draw(Color color) throws IOException{
		String str = String.format("xset(\"color\",%d);\n", color.getColor());
		out.write(str);
		out.close();
		return this;
	}
	
	public FileWriter getReference(){
		return out;
	}	
	
}

/**
 * 
 */
package swim.core.network.misc.geometry;

import java.io.IOException;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class VerticalLine extends Line{
	
	double x;
	
	public VerticalLine(double x){
		super(Geometry.inf, Geometry.inf);
		this.x = x;
	}
	
	public double getX(double y){
		return x;
	}
	
	public double getX(){
		return getX(0.0);
	}
	
	public void draw(Drawing drawing) throws IOException{
		String str = String.format("plot2d([%f,%f],[%f,%f]);\n", x, x, drawing.yMin, drawing.yMax);
		drawing.getReference().write(str);
	}
	
	public Object intersect(Object obj){

		Point p = null;
		Line l = null;
		
		try{
			p = (Point)obj;
		}catch(Exception e1){
			try{
				l = (Line)obj;
			}catch(Exception e2){
			}
		}
		
		if(p != null){
			return intersect((Point)obj);
		}else if(l != null){
			return intersect((Line)obj);
		}else{
			return obj.intersect(this);
		}
	
	}
	
	public Object intersect(Point p){
		if(Math.abs(p.getX() - this.x) < Geometry.epsilon){
			return new Point(p);
		}else{
			return Phi.thePhi;
		}
	}
	
	public Object intersect(Line l){

		VerticalLine vl = null;
		
		try{
			vl = (VerticalLine)l;
		}catch(Exception e1){}
		
		if(vl != null){
			return intersect((VerticalLine)l);
		}else{
			return new Point(this.x, l.getY(x));
		}
		
	}
	
	public Object intersect(VerticalLine l){
        if(Math.abs(this.x - l.x) < Geometry.epsilon){
        	return new VerticalLine(x);
        }else
            return Phi.thePhi;
    }
}

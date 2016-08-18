/**
 * 
 */
package swim.core.network.misc.geometry;

import java.io.IOException;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class Line implements Object{
	
	private double a, b;
	
	public Line(Line l){
		this.a = l.a;
		this.b = l.b;
	}
	
	public Line(double slope, double y0){
		this.a = slope;
		this.b = y0;
	}
	
	public static Line create(Point p1, Point p2){
		double dx = p1.getX() - p2.getX();
		if(Math.abs(dx) < Geometry.epsilon){
			return new VerticalLine(p1.getX());
		}else{
			double s = (p1.getY() - p2.getY())/dx;
			return new Line(s, p1.getY() - s * p1.getX());
		}
	}
	
	public double getSlope(){
		return this.a;
	}
	
	public double getY0(){
		return this.b;
	}
	
	public double getY(double x){
		return a * x + b;
	}
	
	public double getX(double y){
		return (y - b) / a;
	}
	
	public void draw(Drawing drawing) throws IOException{
		String str = String.format("plot2d([%f,%f],[%f,%f]);\n", drawing.xMin, drawing.xMax, getY(drawing.xMin), getY(drawing.xMin));
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
			return (Line)obj.intersect(this);		// Check
		}else{
			return obj.intersect(this);
		}
	
	}
	
	public Object intersect(Point p){
		if(Math.abs(getY(p.getX()) - p.getY()) < Geometry.epsilon){
			return new Point(p);
		}else{
			return Phi.thePhi;
		}
	}
	
	public Object intersect(Line l){
		if(Math.abs(a - l.a) < Geometry.epsilon){
			if(Math.abs(b - l.b) < Geometry.epsilon){
				return new Line(a, b);
			}else{
				return Phi.thePhi;
			}
		}else{
			double x = (b - l.b)/(l.a - a);
		    return new Point(x, getY(x));
		}
	}
	
}

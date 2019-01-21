/**
 * 
 */
package swim.core.network.misc.geometry;

import java.io.IOException;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class Point implements Object {
	
	private double x, y;
	
	public Point(double x, double y){
		this.x = x;
		this.y = y;
	}
	
	public Point(Point p){
		this.x = p.x;
		this.y = p.y;
	}
	
	public double getX(){
		return this.x;
	}
	
	public double getY(){
		return this.y;
	}
	
	public double getDistance(Point p){
		return Math.sqrt(Math.pow((this.x - p.x), 2) + Math.pow((this.y - p.y), 2));
	}
	
	public void draw(Drawing drawing) throws IOException{
		String str = String.format("plot(%d,%d,\"x\");\n", this.x, this.y);
		drawing.getReference().write(str);
	}

	public Object intersect(Object obj){
		
		Point p = null;
		
		try{
			p = (Point)obj;
		}catch(Exception e){}
		
		if(p != null){
			return intersect((Point)obj);
		}
		return obj.intersect(this);
		
	}

	public Object intersect(Point p){
		if(Math.abs(p.x - this.x) < Geometry.epsilon && Math.abs(p.y - this.y) < Geometry.epsilon){
			return new Point(this);
		}else{
			return Phi.thePhi;
		}
	}
}

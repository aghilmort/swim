/**
 * 
 */
package swim.core.network.misc.geometry;

import java.io.IOException;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class Circle implements Object{
	
	double cx, cy, r;
	
	public Circle(double cx, double cy, double r){
		this.cx = cx;
		this.cy = cy;
		this.r = r;
	}
	
	public double getRadius(){
		return this.r;
	}
	
	public double getX(){
		return cx;
	}
	
	public double getY(){
		return cy;
	}
	
	public Point getCenter(){
		return new Point(cx, cy);
	}
	
	public enum Location{
		INSIDE (-1),
		ON (0),
		OUTSIDE (1);
		
		private final int value;
		
		Location(int val){
			this.value = val;
		}
		
		int getValue(){
			return value;
		}
		
		private int value(){
			return this.value;
		}
	}

	public Location where(Point p){
		double d = Math.pow((p.getX() - cx), 2) + Math.pow((p.getY() - cy), 2) - Math.pow(r, 2);
		if(d < -Geometry.epsilon){
			return Location.INSIDE;
		}else if(d > Geometry.epsilon){
			return Location.OUTSIDE;
		}else{
			return Location.ON;
		}
	}
	
	public Boolean contains(double x, double y){
		return (Math.pow(x - cx, 2) + Math.pow(y - cy, 2)) <= Math.pow(r, 2);
	}
	
	public void draw(Drawing drawing) throws IOException{
		String str = String.format("xarc(%f,%f,%f,%f,0,360*64);", (cx - r), (cy - r), (2*r), (2*r));
		drawing.getReference().write(str);
	}
	
	public Object intersect(Object obj){

		Point p = null;
		Line l = null;
		VerticalLine vl = null;
		Circle c = null;
		
		try{p = (Point)obj;}catch(Exception e1){
			try{l = (Line)obj;}catch(Exception e2){
				try{vl = (VerticalLine)obj;}catch(Exception e3){
					try{c = (Circle)obj;}catch(Exception e4){
					}
				}
			}
		}
		
		if(p != null){
			return intersect((Point)obj);
		}else if(l != null){
			return intersect((Line)obj);
		}else if(vl != null){
			return intersect((VerticalLine)obj);
		}else if(c != null){
			return intersect((Circle)obj);
		}else{
			return obj.intersect(this);
		}
	
	}
	
	public Object intersect(Point p){
		if(where(p) == Location.ON){
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
			double d = Math.abs(l.getY(cx)-cy)/Math.sqrt(1 + Math.pow(l.getSlope(), 2));
			
			if(d > (r + Geometry.epsilon)){
				return Phi.thePhi;
			}else{
				double dx = Math.sqrt(Math.pow(r, 2)-Math.pow(d, 2))/Math.sqrt(1+Math.pow(l.getSlope(), 2));
				double x = (cx+ l.getSlope()*(cy - l.getY0()))/(1+Math.pow(l.getSlope(), 2));
				if(dx < Geometry.epsilon){
					return new Point(x, l.getY(x));
				}else{
					Objects points = new Objects();
					points.add(new Point(x - dx, l.getY(x - dx)));
					points.add(new Point(x + dx, l.getY(x + dx)));
					return points;
				}
			}
		}
		
	}
	
	public Object intersect(VerticalLine l){
		double d = Math.abs(l.getX() - cx);
		
		if(d > r + Geometry.epsilon){
			return Phi.thePhi;
		}else{
			double dy = Math.sqrt(Math.pow(r, 2) - Math.pow(d, 2));
			
			if(dy < Geometry.epsilon){
				return new Point(l.getX(), cy);
			}else{
				Objects points = new Objects();
				points.add(new Point(l.getX(), cy - dy));
				points.add(new Point(l.getX(), cy + dy));
				return points;
			}
		}
    }
	
	public Object intersect(Circle c){
		
        double d = Math.sqrt(Math.pow(c.cx - cx, 2)+Math.pow(c.cy - cy, 2));

        double l1 = (Math.pow(r, 2) - Math.pow(c.getRadius(), 2) + Math.pow(d, 2)) / (2*d);
        double l2 = d - l1;

        Line cord;

        if(Math.abs(c.cy - cy) < Geometry.epsilon)
        {
            cord = new VerticalLine((l2 * cx + l1 * c.cx)/d);
        }
        else
        {
            double a = (cx - c.cx)/(c.cy - cy);
            if(c.cy > cy)
                cord = new Line(a, cy - a * cx + l1 * Math.sqrt(1 + Math.pow(a, 2)));
            else
                cord = new Line(a, cy - a * cx - l1 * Math.sqrt(1 + Math.pow(a, 2)));
//			    (Math.pow(r, 2) + Math.pow(c.r, 2) - (Math.pow(cx, 2) + Math.pow(cy, 2) + Math.pow(c.cx) + Math.pow(c.cy) ))
//			    /(2*(c.cy - cy)));
        }

        return intersect(cord);
	}
	
}

/**
 * 
 */
package swim.core.network.misc.geometry;

import java.io.IOException;
import java.util.Vector;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class Objects implements Object{

	Vector<Object> objs;
	
	public Objects(){}
	
	public void add(Object p){
		
		Phi phi = null;
		
		try{
			phi = (Phi)p;
		}catch(Exception e){}
		
		if(phi != null){
			objs.add(p);
		}
	}
	
	public Object Reduce(){
		if(getSize() == 0){
			return Phi.thePhi;
		}else if(getSize() == 1){
			return objs.get(0);
		}else{
			return this;
		}
	}
	
	public int getSize(){
		return objs.size();
	}
	
	public Object toLine(){
		
		Point p1 = null, p2 = null;
		
		try{p1 = (Point)getAt(0);}catch(Exception e){}		
		try{p2 = (Point)getAt(1);}catch(Exception e){}
		
		if(getSize() == 2 && p1 != null && p2 != null){
			return Line.create(p1, p2);
		}else{
			return Phi.thePhi;
		}
	}
	
	public Object getAt(int i){
		return objs.get(i);
	}
	
	public void draw(Drawing drawing) throws IOException{
		for(Object obj : objs){
			obj.draw(drawing);
		}
	}
	
	public Object intersect(Object obj){
		Objects result = new Objects();
		
		for(Object obj1 : objs){
			result.add(obj1.intersect(obj));
		}
		
		return result.Reduce();
	}
	
}

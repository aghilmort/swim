/**
 * 
 */
package swim.core.network.deployment;

import java.io.IOException;
import java.util.Vector;

import swim.core.network.Network;
import swim.core.network.UWSensorNetwork;
import swim.core.network.err.InvalidLinkId;
import swim.core.network.err.InvalidNodeId;
import swim.core.network.misc.PropertyMap;
import swim.core.network.misc.geometry.Circle;
import swim.core.network.misc.geometry.Geometry;
import swim.core.network.misc.geometry.Line;
import swim.core.network.misc.geometry.Objects;
import swim.core.network.misc.geometry.Phi;
import swim.core.network.misc.geometry.Point;
import swim.core.network.misc.geometry.VerticalLine;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class GeometricCandidates {
	
	double d;
	int n;
	private UWSensorNetwork network;
	
	public GeometricCandidates(UWSensorNetwork network, double d /* Minimum interspacing between nodes */){
		this.network = network;
		this.d = d;
	}
	
	public void printID(){
		String str = "Geometric(%0.1lf,%d)";
		String.format(str, d, n);
	}
	
	Vector<Double> findMiddleSurfacePoint(int node1, int node2){
		
		Vector<Double> point = new Vector<Double>();
		double x, y;
		
	    double x_1 = network.getX(node1), y_1 = network.getY(node1), z_1 = network.getZ(node1), 
	    R1 = network.getPowerModel().getCommunicationRange(node1, 0);
	    
		double x_2 = network.getX(node2), y_2 = network.getY(node2), z_2 = network.getZ(node2), 
		R2 = network.getPowerModel().getCommunicationRange(node2, 0);
		
	    double A = 2*(x_1-x_2), B = 2*(y_1-y_2), 
	    C = Math.pow(x_1, 2) + Math.pow(y_1, 2) + Math.pow(z_1, 2) - Math.pow(x_2, 2) - Math.pow(y_2, 2) - Math.pow(z_2, 2);

	    Line L1;
	    if(Math.abs(B) > Geometry.epsilon)
	        L1 = new Line(- A/B, C/B);
	    else
	        L1 = new VerticalLine(C/A);

	    Point p1 = new Point(x_1,y_1);
	    Point p2 = new Point(x_2,y_2);
	    Line L2 = Line.create(p1,p2);

	    Point p3 = null;
	    
	    try{
	    	p3 = (Point)(L1.intersect(L2));
	    }catch(Exception e){}

	    if(p3 == null) return null;

	    x = p3.getX();
	    y = p3.getY();
	    
	    point.add(x);
	    point.add(y);

	    p1 = null; p2 = null; p3 = null; L1 = null; L2 = null;
	    System.gc();
	    
	    double d1 = Math.sqrt(Math.pow(x - x_1, 2)+Math.pow(y - y_1, 2)+Math.pow(z_2, 2));
	    double d2 = Math.sqrt(Math.pow(x - x_2, 2)+Math.pow(y - y_2, 2)+Math.pow(z_2, 2));

	    if( d1 < R1 && d2 < R2)
	        return point;
	    else
	        return null;

/*	    {
	        double D = Math.sqrt(Math.pow(network.getX(node1) - network.getX(node2), 2)+Math.pow(network.getY(node1)- network.getY(node2), 2));
	        double d1 = network.getZ(node1), d2 = network.getZ(node2);

	        if(D>R1+R2) return false;

	        double R_2 = Math.pow(R1,2)+Math.pow(R2,2);
	        double D_3 = Math.pow(d1,2)+Math.pow(d2,2)+Math.pow(D,2);

	        double qA = Math.pow(R_2,2)-2*Math.pow(R1,2)*Math.pow(R2,2);
	        double qB = 2*(Math.pow(d1,2)*Math.pow(R2,2)+Math.pow(d2,2)*Math.pow(R1,2)+R_2*D_3);
	        double qC = Math.pow(D_3,2)-2*Math.pow(d1,2)*Math.pow(d2,2);

	        double C, temp;
	        switch(Geometry.solveQuadratic(qA, qB, qC, temp, C))
	        {
	        case 0:
	            return false;
	        case 1:
	        case 2:
	        default:
	            {
	                if(C<0) return false;
	                double c = Math.sqrt(C);
	                R1*=c;
	                R2*=c;
	                Circle C1 = new Circle(network.getX(node1), network.getY(node1), Math.sqrt(Math.pow(R1,2)-Math.pow(d1,2)));
	                Circle C2 = new Circle(network.getX(node2), network.getY(node2), Math.sqrt(Math.pow(R2,2)-Math.pow(d1,2)));

	                Object o = (C1.intersect(C2));

	                C1 = null; C2 = null;
	                System.gc();

	                if(o != Phi.thePhi)
	                {
	                    Point p = null;
	                    
	                    try{
	                    	p = (Point)o;
	                    }catch(Exception e){}
	                   

	                    if(p != null)
	                    {
	                        x = p.getX();
	                        y = p.getY();

	                        p = null;
	                        System.gc();
	                        
	                        return true;
	                    }
	                    o = null;
                        System.gc();
	                }
	                return false;
	            }
	        }
	    }*/
	}

	double crossProduct(double x1, double y1, double x2, double y2){
		return x1*y2-y1*x2;
	}
	
	void twoLineIntersection(Point p1, Point p2, Point p3, Point p4, double x, double y)
	{
	    Line L1 = Line.create(p1, p2);
	    Line L2 = Line.create(p3, p4);
	    Point i = (Point)(L1.intersect(L2));
	    x = i.getX();
	    y = i.getY();
	    i = null; L1 = null; L2 = null;
	    System.gc();
	}
	
	Vector<Double> findMiddleSurfacePoint(int n1, int n2, int n3)
	{
		Vector<Double> point = new Vector<Double>();
		double x, y;
		
	    double R1=network.getPowerModel().getCommunicationRange(n1, 0),
			   R2=network.getPowerModel().getCommunicationRange(n2, 0),
			   R3=network.getPowerModel().getCommunicationRange(n3, 0);
	    double x1,y1,z1,x2,y2,z2,x3,y3,z3;
	    x1 = network.getX(n1); y1=network.getY(n1); z1=network.getZ(n1);
	    x2 = network.getX(n2); y2=network.getY(n2); z2=network.getZ(n2);
	    x3 = network.getX(n3); y3=network.getY(n3); z3=network.getZ(n3);

	    //A12 *(Math.pow(x,2)+Math.pow(y,2))+ BX12 * x +BY12 * y + C12;
	    double  M1 = Math.pow(x1,2)+Math.pow(y1,2)+Math.pow(z1,2),
	            M2 = Math.pow(x2,2)+Math.pow(y2,2)+Math.pow(z2,2),
	            M3 = Math.pow(x3,2)+Math.pow(y3,2)+Math.pow(z3,2);

	    double A12 = (R2-R1),BX12 = 2*(R1*x2-R2*x1), BY12 = 2*(R1*y2-R2*y1), C12 = R2*M1-R1*M2;
	    double A13 = (R3-R1),BX13 = 2*(R1*x3-R3*x1), BY13 = 2*(R1*y3-R3*y1), C13 = R3*M1-R1*M3;

	    assert (A12==0 && A13==0) : "Case with different communication ranges is not implemented!";
	    if(A12!=0 || A13!=0)
	    {
	        if(Math.abs(BY13*BX12 - BY12*BX13)< Geometry.epsilon)
	        {
	            //three points on one line
	            x = (x1+x2+x3)/3;
	            y = (y1+y2+y3)/3;
	        }
	        else
	        {
	            x = (BY12*C13-BY13*C12) / (BY13*BX12 - BY12*BX13);
	            y = (-C12 - BX12*x)/BY12;

	            Point p = new Point(x,y);
	            Point p1 = new Point(x1,y1);
	            Point p2 = new Point(x2,y2);
	            Point p3 = new Point(x3,y3);

	            int CP12 = (crossProduct(x3-x1,y3-y1,x2-x1,y2-y1)*crossProduct(x-x1,y-y1,x2-x1,y2-y1)>0) ? 1 : 0;
	            int CP23 = (crossProduct(x1-x2,y1-y2,x3-x2,y3-y2)*crossProduct(x-x2,y-y2,x3-x2,y3-y2)>0) ? 1 : 0;
	            int CP31 = (crossProduct(x2-x3,y2-y3,x1-x3,y1-y3)*crossProduct(x-x3,y-y3,x1-x3,y1-y3)>0) ? 1 : 0;
	            //If the point is not inside the surface triangle
	            int section = CP12|(CP23<<1)|(CP31<<2);

	            switch(section)
	            {
	            case 1:
	                x = x3;
	                y=y3;
	                break;
	            case 2:
	                x = x1;
	                y = y1;
	                break;
	            case 3:
	                twoLineIntersection(p, p2, p1, p3, x, y);
	                break;
	            case 4:
	                x = x2;
	                y = y2;
	                break;
	            case 5:
	                twoLineIntersection(p, p1, p2, p3, x, y);
	                break;
	            case 6:
	                twoLineIntersection(p, p3, p2, p1, x, y);
	                break;
	            default:
	                //Within triangle; keep as is
	                break;
	            }
	        }
	        double d1 = Math.sqrt(Math.pow(x-x1,2)+Math.pow(y-y1,2)+Math.pow(z1,2));
	        double d2 = Math.sqrt(Math.pow(x-x2,2)+Math.pow(y-y2,2)+Math.pow(z2,2));
	        double d3 = Math.sqrt(Math.pow(x-x3,2)+Math.pow(y-y3,2)+Math.pow(z3,2));

	        if(d1<=R1 && d2<=R2 && d3<=R3){
	            point.add(x);
	            point.add(y);
	            return point;
	        }else{
	        	return null;
	        }
	        
	    }
	    return null;
	}

	protected Circle surfaceCircle(int node){
		double R = network.getPowerModel().getCommunicationRange(node, 0);
	    return new Circle(network.getX(node), network.getY(node), Math.sqrt(Math.pow(R,2)-Math.pow(network.getZ(node),2)));
	}
	
	int nearByNode(Vector<Integer> candidateNodes, Objects objs, double x, double y, double d)
	{
	    for(int c : candidateNodes)
	    {
	        if(Math.sqrt(Math.pow(x - network.getX(c),2)+Math.pow(y - network.getY(c),2)) < d)
	        {
	            Boolean inside = true;
	            for(int i = 0; i < objs.getSize(); i++)
	            {
	                Circle C = null;
	                C = (Circle)(objs.getAt(i));
	                if(C != null && !(C.contains(network.getX(c), network.getY(c)))){
	                    inside = false;
	                    break;
	                }
	            }
	            if(inside) return c;
	        }
	    }
	    return 0;
	}

	public Vector<Integer> generateNodes(Vector<Integer> fixedNodes, Vector<Integer> candidateNodes, Boolean nodeType) throws InvalidNodeId, InvalidLinkId, IOException
	{
		Objects bigDrawing = new Objects();

		int candidateId = 1000;

	    //Gateway serving three UW node
	    if(true)
	    for(int i : fixedNodes)
	    {
	        for(int j : fixedNodes)
	        {
	            if(i!=j)
	            {
	                for(int k : fixedNodes)
	                {
	                    if(i!=k && j!=k ){
	                        double x,y;
	                        Vector<Double> thePoint = findMiddleSurfacePoint(i, j, k);
	                        
	                        if(thePoint != null){
	                        	
	                        	x = thePoint.get(0);
	                        	y = thePoint.get(1);
	                        	
	                            Objects objs = new Objects();
	                            objs.add(surfaceCircle(i));
	                            objs.add(surfaceCircle(j));
	                            objs.add(surfaceCircle(k));

	                            int nearbyNode = nearByNode(candidateNodes, objs, x, y, d);
	                            if(nearbyNode == 0){
									int candidate = network.candidateNode(x, y);
									//candidate->set_Id(CandidateId++);           Check how to take care of the id's issue
									candidateNodes.add(candidate);

	                                objs.add(new Point(x,y));
	                                Geometry.printDrawing(objs,"drawing.sci");
									bigDrawing.add(objs);
	                            }
	                        }
	                    }
	                }
	            }
	        }
	    }

	    //Gateway serving two UW node
	    if(true)
	    for(int i : fixedNodes)
	    {
	    	for(int j : fixedNodes)
	        {
	            if(i != j)
	            {
	                double x,y;
	                Vector<Double> thePoint = findMiddleSurfacePoint(i, j);
	                
	                if(thePoint != null)
	                {
	                	x = thePoint.get(0);
                    	y = thePoint.get(1);
	                	
	                    Objects objs = new Objects();
	                    objs.add(surfaceCircle(i));
	                    objs.add(surfaceCircle(j));

	                    int nearbyNode = nearByNode(candidateNodes, objs, x, y, d);
	                    if(nearbyNode == 0){
							int candidate = network.candidateNode(x, y);
							//Candidate->set_Id(CandidateId++);					Check how to take care of the id's issue
							candidateNodes.add(candidate);

	                        objs.add(new Point(x,y));
	                        Geometry.printDrawing(objs,"drawing.sci");
							bigDrawing.add(objs);
	                    }
	                }
	            }
	        }
	    }

	    //Gateway serving one UW node
	    if(true)
	    for(int i : fixedNodes)
	    {
	        Objects  objs = new Objects();
	        objs.add(surfaceCircle(i));

	        double x,y;
	        x = network.getX(i);  y = network.getY(i);

	        int nearbyNode = nearByNode(candidateNodes, objs, x, y, d);
	        if(nearbyNode == 0){
				int candidate = network.candidateNode(x, y);
				//Candidate->set_Id(CandidateId++);					Check how to take care of the id's issue
				candidateNodes.add(candidate);

	            objs.add(new Point(x,y));
	            Geometry.printDrawing(objs,"drawing.sci");
				bigDrawing.add(objs);
	        }
	    }

	    Geometry.printDrawing(bigDrawing,"drawing.sci");
	    
	    return candidateNodes;
	}
	
}

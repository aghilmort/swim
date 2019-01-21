package swim.core.network;

import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import swim.core.network.err.InvalidLinkId;
import swim.core.network.err.InvalidNodeId;
import swim.core.network.misc.Property;
import swim.core.network.misc.PropertyMap;
import swim.core.network.misc.ReadOnlyProperty;

/**
 * @version 1.1<br />
 * @author Saleh Ibrahim<br />
 * Edited by: Sherif Tolba<br />
 * Last edit date: 9/24/2013
 *
 */
public class Network {

	/**
	 * Actual number of nodes.
	 * nodeId is in [1, nodeCount]
	 */
	private int nodeCount;
	
	/**
	 * Number of zones. A zone is a subset of nodes. 
	 * Some zones are terminal zones, that is they cannot be used
	 * for routing traffic going to other zones. Other zones can.
	 */
	private int zoneCount;
	
	/**
	 * Actual number of links.
	 * linkId is in [1, linkCount]
	 */
	private int linkCount;
	
	/**
	 * The set of links.
	 * [1, linkCount]
	 */
	private final Links links;
	
	private List<Integer>[] outLinks;

	private List<Integer>[] inLinks;

	/**
	 * The set of nodes
	 * [1, nodeCount]
	 */
	private final Nodes nodes;
	
	// lp: link property
	// np: node property
	
	/**
	 * Storage for link origin nodes
	 */
	protected final Property<Integer> lpFrom;
	
	/**
	 * Storage for link destination nodes
	 */
	protected final Property<Integer> lpTo;
	
	private final Property<LinkedList<Integer>> npInLinks;
	
	private final Property<LinkedList<Integer>> npOutLinks;

	/**
	 * Link property map.
	 */
	private final Map<String, ReadOnlyProperty<?>> linkProperties;
	
	
	/**
	 * Node property map.
	 */
	private final Map<String, ReadOnlyProperty<?>> nodeProperties;

	/**Network constructor
	 * @param nodeCount number of nodes
	 * @param zoneCount number of zones
	 * @param linkCount	number of links
	 */
	public Network(int nodeCount, int zoneCount, int linkCount){
		this.nodeCount = nodeCount;
		this.zoneCount = zoneCount;				
		this.linkCount = linkCount;
		this.linkProperties = new HashMap<String, ReadOnlyProperty<?>>();
		this.nodeProperties = new HashMap<String, ReadOnlyProperty<?>>();
		
		links = new Links();
		nodes = new Nodes();
				
		lpFrom = new PropertyMap<Integer>("link", "From", this, 0);
		lpTo = new PropertyMap<Integer>("link", "To", this, 0);
		
		npInLinks = new PropertyMap<LinkedList<Integer>>("node", "InLinks", this, null);
		npOutLinks = new PropertyMap<LinkedList<Integer>>("node", "OutLinks", this, null);
	}
	
	/**Network 'convenience' constructor
	 * Creates an empty network (has no nodes nor links). This allows the
	 * instantiation of a network while deferring the addition of nodes
	 * and links to a later time 
	 */
	public Network(){
		this.nodeCount = 0;
		this.zoneCount = 0;				
		this.linkCount = 0;
		this.linkProperties = new HashMap<String, ReadOnlyProperty<?>>();
		this.nodeProperties = new HashMap<String, ReadOnlyProperty<?>>();
		
		links = new Links();
		nodes = new Nodes();
				
		lpFrom = new PropertyMap<Integer>("link", "From", this, 0);
		lpTo = new PropertyMap<Integer>("link", "To", this, 0);
		
		npInLinks = new PropertyMap<LinkedList<Integer>>("node", "InLinks", this, null);
		npOutLinks = new PropertyMap<LinkedList<Integer>>("node", "OutLinks", this, null);
	}	
	
	/**Copy constructor
	 * @param o other network to copy from
	 */
	public Network(Network o){
		nodeCount = o.nodeCount;
		zoneCount = o.zoneCount;
		linkCount = o.linkCount;
		this.linkProperties = new HashMap<String, ReadOnlyProperty<?>>();
		this.nodeProperties = new HashMap<String, ReadOnlyProperty<?>>();
		
		links = new Links();						
		nodes = new Nodes();

		lpFrom = o.lpFrom.clone(this);
		lpTo = o.lpTo.clone(this);
		
		npInLinks = o.npInLinks.clone(this);
		npOutLinks = o.npOutLinks.clone(this);
	}
	

	/** add a link to the network
	 * @param from start-node id
	 * @param to end-node id
	 * @return added-link id
	 * @throws InvalidLinkId 
	 */
	public int addLink(int from, int to) throws InvalidNodeId, InvalidLinkId{
		
		int id = Links.firstLinkId+links.count;
		
		if(id<Links.firstLinkId || id >= Links.firstLinkId+linkCount){
			throw new InvalidLinkId(linkCount, id);				
		}else if(from<1 || from > nodeCount){
			throw new InvalidNodeId(nodeCount, from);				
		} else if(to<1 || to>nodeCount){
			throw new InvalidNodeId(nodeCount, to);				
		} else {
			this.lpFrom.set(id, from);
			this.lpTo.set(id, to);
		} 
	
		links.count++;
		return id;
	}
	
	/**
	 * add a node to the network
	 */
	public int addNode(LinkedList<Integer> inLinks, LinkedList<Integer> outLinks) throws InvalidNodeId, InvalidLinkId {
		
		int id = Nodes.firstNodeId+nodes.count;
		
		if(id<Nodes.firstNodeId || id >= Nodes.firstNodeId+nodeCount){
			throw new InvalidNodeId(nodeCount, id);				
		}
		
		for(int linkId : inLinks){
			if(linkId < 1 || linkId > linkCount){
				throw new InvalidLinkId(linkCount, linkId);
			}
		}
		
		for(int linkId : outLinks){
			if(linkId < 1 || linkId > linkCount){
				throw new InvalidLinkId(linkCount, linkId);
			}
		}
	
		this.npInLinks.set(id, inLinks);
		this.npOutLinks.set(id, outLinks);
		
		nodes.count++;
		return id;
	}
	
	/**Add link property association
	 * @param name property name
	 * @param property link property map
	 */
	public void addLinkProperty(String name, ReadOnlyProperty<?> property){
		linkProperties.put(name, property);
	}

	/**Add node property association
	 * @param name property name
	 * @param property node property map
	 */
	public void addNodeProperty(String name, ReadOnlyProperty<?> property){
		nodeProperties.put(name, property);
	}	
	
	/**Retrieve link property by name
	 * @param name property name
	 * @return link property map
	 */
	public ReadOnlyProperty<?> getLinkProperty(String name){
		return linkProperties.get(name);
	}
		
	public ReadOnlyProperty<?> getNodeProperty(String name){
		return nodeProperties.get(name);
	}
        
        public boolean linkExists(int from, int to) {
            
            Iterator linkItr = getLinks().iterator();
            Integer link;
            
            while(linkItr.hasNext()) {
                link = (Integer)linkItr.next();
                if( lpFrom.get(link) == from && lpTo.get(link) == to ) return true;
            }
            return false;
        }

	/**
	 * @return number of nodes 
	 */
	public int getNodeCount() {
		return nodeCount;
	}

	/**
	 * @return number of zones
	 */
	public int getZoneCount() {
		return this.zoneCount;
	}

	/**
	 * @return number of links
	 */
	public int getLinkCount() {
		return linkCount;
	}
	
	/**
	 * set number of nodes, zones, and links
	 */
	public void deferredInit(int nodeCount, int zoneCount, int linkCount){
		this.zoneCount = zoneCount;
		this.nodeCount = nodeCount;
		this.linkCount = linkCount;
	}
	
	/**
	 * set number of nodes and zones
	 */
	public void deferredNodeInit(int nodeCount, int zoneCount){
		this.zoneCount = zoneCount;
		this.nodeCount = nodeCount;
	}
	
	/**
	 * set number of links
	 */
	public void deferredLinkInit(int linkCount){
		this.linkCount = linkCount;
	}
	
	public Integer getDestination(int linkId) {
		return lpTo.get(linkId);
	}

	public Integer getOrigin(int linkId) {
		return lpFrom.get(linkId);
	}

	public Nodes getNodes() {
		return nodes;
	}
	
	public void resetNodes(){
		nodes.count = 0;
	}

	public Links getLinks() {
		return links;
	}
	
	public void resetLinks(){
		links.count = 0;
	}

	/**
	 * @return an array of lists, element i is a list of links leaving node i. 
	 * Element 0 is unused
	 */
	@SuppressWarnings("unchecked")
	public List<Integer>[] getOutLinks() {
		if(outLinks==null){
			outLinks = new List[getNodeCount()+1];
			for(int i:getNodes()){
				outLinks[i] = new LinkedList<Integer>();				
			}
			for (int e : getLinks())
				outLinks[getOrigin(e)].add(e);
		}
		return outLinks;
	}

	public List<Integer> getOutLinks(int nodeId) {
		return getOutLinks()[nodeId];
	}

	/**
	 * @return an array of lists, element i is a list of links entering node i 
	 * element 0 is unused
	 */
	@SuppressWarnings("unchecked")
	public List<Integer>[] getInLinks() {
		if(inLinks==null){
			inLinks = new List[getNodeCount()+1];
			for(int i:getNodes()){
				inLinks[i] = new LinkedList<Integer>();				
			}
			for (int e : getLinks())
				inLinks[getDestination(e)].add(e);
		}
		return inLinks;
	}

	public List<Integer> getInLinks(int nodeId) {
		return getInLinks()[nodeId];
	}

	public int getFirstNodeId() {
		return Network.Nodes.firstNodeId;
	}

	public int getLastNodeId() {
		return Network.Nodes.firstNodeId+nodeCount-1;
	}
	
	//@Override
	public String toString(String graphComponent) {
		
		StringBuilder sb = new StringBuilder();
		
		sb.append(toStringHeader());
		
		if(graphComponent.equals("nodes")){
			sb.append(nodes.toString());
		}else if(graphComponent.equals("links")){
			sb.append(links.toString());
		}
				
		sb.append("\n  ");
		
		return sb.toString();		
	}
	
	private String toStringHeader(){
		return String.format("Network has %d nodes and %d links\n", 
				getNodeCount(), getLinkCount());
	}
	
	public String toString(String graphComponent, String[] columns) {
		
		//TODO: Clean up this code 
		StringBuilder sb = new StringBuilder();
		
		sb.append(toStringHeader());
		
		if(graphComponent.equals("nodes")){
			sb.append(nodes.toString(columns));	
		}else if(graphComponent.equals("links")){
			sb.append(links.toString(columns));	
		}
			
		sb.append("\n  ");
		
		return sb.toString();		
	}	

	/**Set of links, indexed by linkId in [1, links.size()]
	 */
	public class Links implements Iterable<Integer>{

		public static final int firstLinkId = 1;
		
		int count = 0;
		
		public Links() {
		}

		public boolean contains(int linkId){
			return linkId>= firstLinkId && linkId<firstLinkId+count;
		}

		public String toString(String[] columns){
			StringBuilder sb = new StringBuilder("LinkId\tFrom\tTo");
			for(String column: columns){
				sb.append('\t'); sb.append(column);
			}
			sb.append('\n'); 
			
			
			for(int i:links){
				sb.append(String.format("#%d\t%d\t%d", 
						i, getOrigin(i), getDestination(i)));
				
				for(String column: columns){
					ReadOnlyProperty<?> property = linkProperties.get(column);
					sb.append('\t'); sb.append(property.get(i).toString());					 
				}
				sb.append('\n'); 
			}
			return sb.toString();
		}

		@Override
		public String toString() {
			StringBuilder sb = new StringBuilder("LinkId\tFrom\tTo");
			for(Entry<String, ReadOnlyProperty<?>> property : linkProperties.entrySet()){
				sb.append('\t'); sb.append(property.getKey());
			}
			sb.append('\n'); 
			
			
			for(int i:links){
				sb.append(String.format("#%d\t%d\t%d", 
						i, getOrigin(i), getDestination(i)));
				
				for(Entry<String, ReadOnlyProperty<?>> property : linkProperties.entrySet()){
					sb.append('\t'); sb.append(property.getValue().get(i).toString());					 
				}
				sb.append('\n'); 
			}
			return sb.toString();
		}

		@Override
		public Iterator<Integer> iterator() {
			class LinkIdIterator implements Iterator<Integer>{

				int i;
				
				public LinkIdIterator(){
					i=firstLinkId-1;
				}
				
				@Override
				public boolean hasNext() {
					return i<linkCount;
				}

				@Override
				public Integer next() {
					i++;
					return i;
				}

				@Override
				public void remove() {
					throw new IllegalArgumentException();
				}
				
			}

			return new LinkIdIterator();
		}
		
	}
	
	public class Nodes implements Iterable<Integer>{

		public static final int firstNodeId = 1;
		
		public int count = 0;
		
		public Nodes() {
		}

		public boolean contains(int nodeId){
			return nodeId>= firstNodeId && nodeId<firstNodeId+count;
		}

		public String toString(String[] columns){
			StringBuilder sb = new StringBuilder("NodeId");
			for(String column: columns){
				sb.append('\t'); sb.append(column);
			}
			sb.append('\n'); 
			
			
			for(int i:nodes){
				sb.append(String.format("#%d", i));
				
				for(String column: columns){
					ReadOnlyProperty<?> property = nodeProperties.get(column);
					sb.append('\t'); sb.append(property.get(i).toString());					 
				}
				sb.append('\n'); 
			}
			return sb.toString();
		}

		@Override
		public String toString() {
			StringBuilder sb = new StringBuilder("NodeId");
			for(Entry<String, ReadOnlyProperty<?>> property : nodeProperties.entrySet()){
				sb.append('\t'); sb.append(property.getKey());
			}
			sb.append('\n'); 
			
			
			for(int i:nodes){
				sb.append(String.format("#%d", i));
				
				for(Entry<String, ReadOnlyProperty<?>> property : nodeProperties.entrySet()){
					sb.append('\t'); sb.append(property.getValue().get(i).toString());					 
				}
				sb.append('\n'); 
			}
			return sb.toString();
		}		
		
		@Override
		public Iterator<Integer> iterator() {
		
			class NodeIdIterator implements Iterator<Integer>{

				int i;
				
				public NodeIdIterator(){
					i=firstNodeId-1;
				}
				
				@Override
				public boolean hasNext() {
					return i<nodeCount;
				}

				@Override
				public Integer next() {
					i++;
					return i;
				}

				@Override
				public void remove() {
					throw new IllegalArgumentException();
				}
			}
			
			return new NodeIdIterator();
		}
	}

	/**
	 * @param outLinks the outLinks to set
	 */
	public void setOutLinks(int node, List<Integer> outLinks) {
		this.outLinks[node] = outLinks;
	}

	/**
	 * @param inLinks the inLinks to set
	 */
	public void setInLinks(int node, List<Integer> inLinks) {
		this.inLinks[node] = inLinks;
	}
		
}

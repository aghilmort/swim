package swim.core;

import java.util.ArrayList;

public class AgentList extends ArrayList<Agent> {

	private static final long serialVersionUID = 4773246044857707036L;

	public AgentList(int numAgents) {
		super(numAgents);
	}
	
	synchronized public Agent getAgent(int i) {
		if( i < super.size() ) {
			return (Agent)get(i);
		} else {
			return null;
		}
	}
	
	synchronized public boolean removeAgent(int i) {
		if( i < super.size() ) {
			super.remove(i);
			return true;
		} else {
			return false;
		}
	}
}

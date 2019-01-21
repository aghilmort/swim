package swim.core.network;

public class ODPairAssignment {
	public Route route;
	public double volume;
	
	public ODPairAssignment(Route route, double volume) {
		super();
		this.route = route;
		this.volume = volume;
	}

	public ODPairAssignment(ODPairAssignment otherODPA) {
		this.route = new MultipathRoute(otherODPA.route);
		this.volume = otherODPA.volume;
	}

	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append(route.toString());
		sb.append("=");
		sb.append(volume);
		return route.toString();
	}
}
	

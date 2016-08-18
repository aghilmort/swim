/**
 * 
 */
package swim.core.network;

import swim.core.network.err.SanityCheckFailed;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class Packet {
	
	private int dataLength, headerLength, trailerLength, overheadLength, totalLength;
	
	public Packet(int dataLength, int headerLength, int trailerLength){
		this.dataLength = dataLength;
		this.headerLength = headerLength;
		this.trailerLength = trailerLength;
		this.overheadLength = headerLength + trailerLength;
		this.overheadLength = dataLength + headerLength + trailerLength;
	}
	
	public Packet(int dataLength){
		this.dataLength = dataLength;
		this.headerLength = 0;
		this.trailerLength = 0;
		this.overheadLength = headerLength + trailerLength;
		this.overheadLength = dataLength + headerLength + trailerLength;
	}

	/**
	 * @return the dataLength
	 */
	public int getDataLength() {
		return dataLength;
	}

	/**
	 * @return the headerLength
	 */
	public int getHeaderLength() {
		return headerLength;
	}

	/**
	 * @return the trailerLength
	 */
	public int getTrailerLength() {
		return trailerLength;
	}

	/**
	 * @return the overheadLength
	 */
	public int getOverheadLength() {
		return overheadLength;
	}

	/**
	 * @return the totalLength
	 */
	public int getTotalLength() {
		return totalLength;
	}
	
	public void sanityCheck() throws SanityCheckFailed{
		swim.core.network.misc.Util.sanityCheck((overheadLength == headerLength + trailerLength) && (totalLength == overheadLength + dataLength), "Inconsistent packet parameters. Overhead should equal Header+Trailer and Total should equal Data+Overhead");
	}
	
}

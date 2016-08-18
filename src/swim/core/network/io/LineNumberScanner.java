package swim.core.network.io;

import java.io.EOFException;
import java.util.Scanner;
import java.util.regex.Pattern;

/*
 * A scanner that keeps track of line number
 */
public class LineNumberScanner{
	Scanner sourceScanner, lineScanner;

	private int lineNumber;		
	private String line;
	private boolean endOfFile;
	
	
	public int getLineNumber() {
		return lineNumber;
	}

	public String getLine() {
		return line;
	}

	private void nextLine(){			
		if(sourceScanner.hasNextLine()){
			lineNumber++;
			line = sourceScanner.nextLine();
			lineScanner = new Scanner(line);
			
			if(!lineScanner.hasNext()){
				nextLine();
			}
		}
		else{
			lineScanner = null;
			line = null;
			endOfFile = true;				
		}
	}

	public LineNumberScanner(Readable reader){			
		sourceScanner = new Scanner(reader);
		lineScanner = null;
		lineNumber = 0;
		line = null;
		endOfFile = false;
		nextLine();
	}

	public int nextInt() throws EOFException {
		if(endOfFile)
			throw new java.io.EOFException("Attempt to read past end of file");

		if(lineScanner.hasNext()){				
			return lineScanner.nextInt();
		}
		else{
			nextLine();
			return lineScanner.nextInt();
		}
	}
	
	public double nextDouble() throws EOFException {
		if(endOfFile)
			throw new java.io.EOFException("Attempt to read past end of file");

		if(lineScanner.hasNext()){
			return lineScanner.nextDouble();
		}
		else{
			nextLine();
			return lineScanner.nextDouble();
		}
	}
	public void skipPattern(Pattern pattern){
		lineScanner.next(pattern);
	}
}

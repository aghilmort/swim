/**
 * 
 */
package swim.core.network.test;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;


/**
 * @author Sherif Tolba (manually translated and consolidated from  
 * 		   different files of a C++ code written by Saleh Ibrahim)
 *
 */
public class ScenarioGenerator {

	public char[] buffer;
	private File file;
	private FileReader rdr;
	
	public ScenarioGenerator() throws FileNotFoundException{
		buffer = new char[64*1024];
		file = new File("prolog.txt");
		rdr = new FileReader(file);
	}
	
	public void writeProlog() throws IOException{
		rdr.read(buffer, 0, 64*1024);
		rdr.close();
		System.out.print(buffer);		
	}
	
	public void writeScenario() throws IOException{
		this.writeProlog();
	}
	
	// Entry point for the application
	public static void main(String[] args) throws IOException{
		ScenarioGenerator SG = new ScenarioGenerator();
		SG.writeScenario();
	}
}

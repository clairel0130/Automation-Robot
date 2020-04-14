/*
*/
package ca.mcgill.ecse211.lab5;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

/**
 * Author: Bakti, Andi-Camille Email: andi-camille.bakti@mail.mcgill.ca
 * 
 * This class is used to create and get a log file on the brick.
 */
public class SensorLogger {

  private String filename = "Log.txt";

  /**
   * This methods writes the input string into the log file "Log.txt". *
   * 
   * @param data
   * 
   */
  public void writeToFile(String data) {
    try {
      PrintWriter writer = new PrintWriter(new FileWriter(this.filename, true));

      writer.println(data + System.lineSeparator());
      writer.close();
    } catch (IOException e) {
      // do something
    }
  }

  /**
   * This method return the log file createds
   * 
   * @return The log file created
   * 
   */
  public File getFile() {
    File file = new File(filename);
    return file;
  }
}

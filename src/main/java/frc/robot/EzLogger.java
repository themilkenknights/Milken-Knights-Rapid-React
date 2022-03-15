// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;


/** Add your docs here. */
public class EzLogger {
    public static String path = "";
    public static String day = "";
 

public static void writeLog(String path, String message)
{
    
    String dateStamp1 = new SimpleDateFormat("yyyy-MM-dd").format(new Date());
    try (PrintWriter writer = new PrintWriter(new FileWriter(path + "\\main_log.txt", true))) {
        writer.print(message + "          ");
        writer.print(new Date().toString());
        writer.println();
        writer.close();
      } catch (IOException e) {
        e.printStackTrace();
      }
}

public static void logRobotInit(String path) {
    String dateStamp1 = new SimpleDateFormat("yyyy-MM-dd").format(new Date());
    boolean test = new File(path + "\\logs\\" + dateStamp1).mkdirs();
    day = dateStamp1;
  }



public static void main(String[] args) {  
    //writeFolder();
    File file = new File("README.md");
    String pathfile = file.getAbsolutePath().substring(0, file.getAbsolutePath().indexOf("\\" + file.getName()));
    logRobotInit(pathfile);
    writeLog(pathfile + "\\logs\\" + day, "hello everybody myname is makruiplier");
}
}

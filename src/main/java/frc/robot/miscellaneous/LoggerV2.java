// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.miscellaneous;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.PrintWriter;
import java.net.URL;
import java.net.URLConnection;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;


/** Add your docs here. */
public class LoggerV2 {

    public static LoggerV2 getInstance()
    {
        return InstanceHolder.mInstance;
    }

    private int teamNumber = 6969;

    private String teamString = Integer.toString(teamNumber);
    private String teamNum = teamString.substring(0, teamString.length()/2) + "." + teamString.substring(teamString.length()/2);
    private static final int BUFFER_SIZE = 4096;
    private File todaysFile = new File("media//" + getDate() + "//hello.txt");


    public String getDate()
    {
        return new SimpleDateFormat("yyyy-MM-dd").format(new Date());
    }


    public void writeToRio(String message)
    {
        try (PrintWriter writer = new PrintWriter(new FileWriter("media//" + getDate() + "//hello.txt", true))) {
            writer.println(message);
            writer.print(new Date().toString());
            writer.println();
            writer.close();
            if(todaysFile.length() > 100000)
            {
                todaysFile.delete();
            }
          } catch (IOException e) {
            e.printStackTrace();
          }
    }

    public void goodMorning()
    {
        //for roborio
        boolean test = new File("media//" + getDate() + "//").mkdirs();
    }

    private static class InstanceHolder
    {
        private static final LoggerV2 mInstance = new LoggerV2();
    } 
}

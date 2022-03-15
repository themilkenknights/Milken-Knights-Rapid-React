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
    //public static String[] face = new String[11];
    public static String bully = ("                                    .7Y?^./n                                   .7P5?^.:.                     .::::          7Y7!75#B5YY~/n                   ^:^7?77~.         .::7@@@@@@&?./n                  !?^~!??Y7.    :?YYJ?!~:!Y#@@&&@BY~./n                .J#G5Y5!:.     !5Y??PB##BGJ^7B@@&&&&G?./n              .YB#&@@&!       :5?7775#&&&&&B! 7B@&&&#&G~/n              J@@&@@@!        :?YPY?75&&&B#@G   !B@&&&&B^/n             7@@@@@@?         ~J?!7?JJJJJJ#@5    :&&&&@@G./n            !@@@@@@?          .YY?JYY5Y5P&&J.    Y@@@@@@5./n           ~&@&@@@B.           !Y5PPBBPJJBJ     !@@@@@@@J/n          :B&&&@@@P:      .^^:.:!7Y&&##BB##? ^YG&@@@@@@B^/n          !@@&@@@&&#G5?!~?B&&###BPG@@&@@@&@&Y#@@@@@@@@&7/n          :Y&@@@@@@@@@@@&@#&&&&&&&@&&#&&&@&&##&@@@@@@@J/n            :?B@@@@@@@@@@@###&&##&&#BB#&&&##B#@@@@@@&Y./n               ^7YG#@@@@@&####&&&&####&&&&##&&&&&@@@Y/n                   .^?5GBB#B###&@&&@@@&&&&&&&&&&&&&J./n                        .J##BB&@@@&&&#&&&@@&&&&&&&G./n                        ^B###&@@&&###&&&@@&&&@&&&&G./n                        ?##&@@@&###&&&&&&&&&&@&&&&&J/n                       ^G##@@@&&##&&#&&#&&@@@@&&&&&&!/n                       J##@@@&##B#&####&&&&@&@&&&&&&G./n                      7B#@@@&#########&&&@@&&&&&&&&&&G!/n                     ~B&@@@&&#&&&&&&&&&&@@@&&&&&&&&&&&Y./n                    :P#@@@@&&&&&&&&&&&&@&##@&&&&&&&&&&G^/n                   :P&@@&&###&&&&&&&&&@&!.^&&&&&&&&&&&@5./n                  ~5&@@&######&&&&&&&&&#?: Y&&&&&&&&&&@#!/n                 ^Y#@&&&&####&&&&&&&&&&&@&Y~B&&&&&&&&&&&#7/n                 !G##&&&#####&&&&@@&&&&&&##5!B&&&&&&&&&&@B./n                :5###&&&&&########&&&&&&@7:. ^G&@@@@&&&#G!/n               ^P######&&&&&############&#5:  .!Y55J7~:./n              ^G##&&&&&&&&&B&&############&G:/n             ^G##&&&&&@@@#!:Y&&&&#########&@5/n             Y&##&#&&&&@@!   7B&#&&&&#####&&B:/n            ~###&&#&&&&@Y.    :P&&###&&&&&&&&7/n            J#####&&&&@5       ^B&&###&&&&&&@5/n           :B###&&&&&@#~       .~G&&&&#&&&&&&?/n           ?####&&&@@#~.        ^!P&&&&&&&&&&Y/n");
 

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
    /*
    face[0] = "⣿⣿⣿⣿⣿⡿⠿⠻⠿⠿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠿⠿⠻⠻⠟⠻⢿⣿⣿⣿⣿";
    face[1] ="⣿⣿⡟⠁⢀⣠⣤⣤⣤⣤⣄⣀⣀⣀⣹⣿⣿⣷⣄⣀⣀⣀⣀⣤⣤⣤⣤⣀⠐⢽⣿⣿⣿";
    face[2] ="⣿⣿⣿⣶⣿⡿⣛⡒⠒⠒⢒⠒⣲⠙⣿⣿⣿⣿⠟⣵⡒⢒⠒⠒⡀⣘⡻⣿⣿⣾⣿⣿⣿ ";
    face[3] ="⣿⣿⣿⣿⣏⣞⡛⠃⠀⠀⠸⠷⢿⣧⣿⣿⣿⣿⣧⣿⣷⣛⣀⣀⣁⣛⣛⣮⣿⣿⣿⣿⣿ ";
    face[4] ="⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿ ";
    face[5] ="⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿ ";
    face[6] ="⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢏⣾⣿⣿⣿⣿ ";
    face[7] ="⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⢻⣿⠏⣼⣿⣿⣿⣿⣿ ";
    face[8] ="⣿⣿⣿⣿⣿⣿⣿⣿⡿⢿⣿⣿⣿⣿⣿⣿⡿⠿⠿⠿⠟⢛⣉⣴⣿⡏⣸⣿⣿⣿⣿⣿⣿ ";
    face[9] ="⣿⣿⣿⣿⣿⣿⣿⣿⣧⣠⣤⣤⣤⣤⣤⣤⣶⣶⣶⣶⣿⣿⣿⣿⣿⠃⣿⣿⣿⣿⣿⣿⣿ ";
    face[10] = "⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿";
    */
    //face dont work :(
    File file = new File("README.md");
    String pathfile = file.getAbsolutePath().substring(0, file.getAbsolutePath().indexOf("\\" + file.getName()));
    logRobotInit(pathfile);
    //for(int i = 0; i < face.length; i++)
    //{

        //bully creator lol
        for(int i = 0; i < 37; i++)
        {
            //System.out.println(bully.indexOf("/n"));
            String temp = bully;
            bully = bully.substring(0, bully.indexOf("/n"));
            writeLog(pathfile + "\\logs\\" + day, bully);
            bully = temp;
            bully = bully.substring(bully.indexOf("/n")+2, bully.length());
        }
        //writeLog(pathfile + "\\logs\\" + day, bully);//face[i]);

    //}
}
}

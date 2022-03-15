// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.miscellaneous;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.UUID;


/**The NSA's official code (if they had one person working there)*/
public class EzLogger {
    private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();
    //public static String[] face = new String[11];
    public static String bully = ("                                    .7Y?^./n                                   .7P5?^.:.                     .::::          7Y7!75#B5YY~/n                   ^:^7?77~.         .::7@@@@@@&?./n                  !?^~!??Y7.    :?YYJ?!~:!Y#@@&&@BY~./n                .J#G5Y5!:.     !5Y??PB##BGJ^7B@@&&&&G?./n              .YB#&@@&!       :5?7775#&&&&&B! 7B@&&&#&G~/n              J@@&@@@!        :?YPY?75&&&B#@G   !B@&&&&B^/n             7@@@@@@?         ~J?!7?JJJJJJ#@5    :&&&&@@G./n            !@@@@@@?          .YY?JYY5Y5P&&J.    Y@@@@@@5./n           ~&@&@@@B.           !Y5PPBBPJJBJ     !@@@@@@@J/n          :B&&&@@@P:      .^^:.:!7Y&&##BB##? ^YG&@@@@@@B^/n          !@@&@@@&&#G5?!~?B&&###BPG@@&@@@&@&Y#@@@@@@@@&7/n          :Y&@@@@@@@@@@@&@#&&&&&&&@&&#&&&@&&##&@@@@@@@J/n            :?B@@@@@@@@@@@###&&##&&#BB#&&&##B#@@@@@@&Y./n               ^7YG#@@@@@&####&&&&####&&&&##&&&&&@@@Y/n                   .^?5GBB#B###&@&&@@@&&&&&&&&&&&&&J./n                        .J##BB&@@@&&&#&&&@@&&&&&&&G./n                        ^B###&@@&&###&&&@@&&&@&&&&G./n                        ?##&@@@&###&&&&&&&&&&@&&&&&J/n                       ^G##@@@&&##&&#&&#&&@@@@&&&&&&!/n                       J##@@@&##B#&####&&&&@&@&&&&&&G./n                      7B#@@@&#########&&&@@&&&&&&&&&&G!/n                     ~B&@@@&&#&&&&&&&&&&@@@&&&&&&&&&&&Y./n                    :P#@@@@&&&&&&&&&&&&@&##@&&&&&&&&&&G^/n                   :P&@@&&###&&&&&&&&&@&!.^&&&&&&&&&&&@5./n                  ~5&@@&######&&&&&&&&&#?: Y&&&&&&&&&&@#!/n                 ^Y#@&&&&####&&&&&&&&&&&@&Y~B&&&&&&&&&&&#7/n                 !G##&&&#####&&&&@@&&&&&&##5!B&&&&&&&&&&@B./n                :5###&&&&&########&&&&&&@7:. ^G&@@@@&&&#G!/n               ^P######&&&&&############&#5:  .!Y55J7~:./n              ^G##&&&&&&&&&B&&############&G:/n             ^G##&&&&&@@@#!:Y&&&&#########&@5/n             Y&##&#&&&&@@!   7B&#&&&&#####&&B:/n            ~###&&#&&&&@Y.    :P&&###&&&&&&&&7/n            J#####&&&&@5       ^B&&###&&&&&&@5/n           :B###&&&&&@#~       .~G&&&&#&&&&&&?/n           ?####&&&@@#~.        ^!P&&&&&&&&&&Y/n");
 
/**
 * writes a message in the daily log (creates new log if new daily log hasnt been created)
 * @param message your message
 */
public static void writeLog(String message)
{
    
    String dateStamp1 = new SimpleDateFormat("yyyy-MM-dd").format(new Date());
    try (PrintWriter writer = new PrintWriter(new FileWriter(getYourFilePath() + "\\logs\\" + dateStamp1 + "\\main_log.txt", true))) {
        writer.print(RUN_INSTANCE_UUID.toString());
        writer.print(message + "          ");
        writer.print(new Date().toString());
        writer.println();
        writer.close();
      } catch (IOException e) {
        e.printStackTrace();
      }
}

/**
 * because everyone has different directories, this one gets your specific one
 * @return your unique path. used in the {@link #writeLog(message)} and the {@link #logRobotInit()} function
 */
public static String getYourFilePath()
{
    File file = new File("README.md");
    return file.getAbsolutePath().substring(0, file.getAbsolutePath().indexOf("\\" + file.getName()));
}

/** runs everytime you boot up the robot. creates a new daily folder if one hasnt been created
 */
public static void logRobotInit() {
    String dateStamp1 = new SimpleDateFormat("yyyy-MM-dd").format(new Date());
    boolean test = new File(getYourFilePath() + "\\logs\\" + dateStamp1).mkdirs();
  }



public static void main(String[] args) {  
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
    
    //! RUN THIS ONCE IN YOUR LIFETIME AND NEVER AGAIN IF YOU WANT LOG TO WORK
    //boolean test = new File(getYourFilePath() + "\\logs").mkdirs();


    logRobotInit();
    //for(int i = 0; i < face.length; i++)
    //{

        //bully creator lol
        for(int i = 0; i < 37; i++)
        {
            //System.out.println(bully.indexOf("/n"));
            String temp = bully;
            bully = bully.substring(0, bully.indexOf("/n"));
            writeLog(bully);
            bully = temp;
            bully = bully.substring(bully.indexOf("/n")+2, bully.length());
        }
        //writeLog(pathfile + "\\logs\\" + day, bully);//face[i]);

    //}
}
}

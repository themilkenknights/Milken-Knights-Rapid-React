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

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.LOGS;


/**The NSA's official code (if they had one person working there)*/
public class EzLogger {
 /**Random thing in swerd code. may be data id*/
    private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();
    private static File file = new File("README.md");
    private static String ultimatePath = file.getAbsolutePath().substring(0, file.getAbsolutePath().indexOf("\\" + file.getName()));
    private static File todaysFolder = new File(staticGetYourFilePath() + "\\logs\\" + staticGetTodaysDate());
 /**le bully ascii art*/
    public static String bully = ("                                    .7Y?^./n                                   .7P5?^.:.                     .::::          7Y7!75#B5YY~/n                   ^:^7?77~.         .::7@@@@@@&?./n                  !?^~!??Y7.    :?YYJ?!~:!Y#@@&&@BY~./n                .J#G5Y5!:.     !5Y??PB##BGJ^7B@@&&&&G?./n              .YB#&@@&!       :5?7775#&&&&&B! 7B@&&&#&G~/n              J@@&@@@!        :?YPY?75&&&B#@G   !B@&&&&B^/n             7@@@@@@?         ~J?!7?JJJJJJ#@5    :&&&&@@G./n            !@@@@@@?          .YY?JYY5Y5P&&J.    Y@@@@@@5./n           ~&@&@@@B.           !Y5PPBBPJJBJ     !@@@@@@@J/n          :B&&&@@@P:      .^^:.:!7Y&&##BB##? ^YG&@@@@@@B^/n          !@@&@@@&&#G5?!~?B&&###BPG@@&@@@&@&Y#@@@@@@@@&7/n          :Y&@@@@@@@@@@@&@#&&&&&&&@&&#&&&@&&##&@@@@@@@J/n            :?B@@@@@@@@@@@###&&##&&#BB#&&&##B#@@@@@@&Y./n               ^7YG#@@@@@&####&&&&####&&&&##&&&&&@@@Y/n                   .^?5GBB#B###&@&&@@@&&&&&&&&&&&&&J./n                        .J##BB&@@@&&&#&&&@@&&&&&&&G./n                        ^B###&@@&&###&&&@@&&&@&&&&G./n                        ?##&@@@&###&&&&&&&&&&@&&&&&J/n                       ^G##@@@&&##&&#&&#&&@@@@&&&&&&!/n                       J##@@@&##B#&####&&&&@&@&&&&&&G./n                      7B#@@@&#########&&&@@&&&&&&&&&&G!/n                     ~B&@@@&&#&&&&&&&&&&@@@&&&&&&&&&&&Y./n                    :P#@@@@&&&&&&&&&&&&@&##@&&&&&&&&&&G^/n                   :P&@@&&###&&&&&&&&&@&!.^&&&&&&&&&&&@5./n                  ~5&@@&######&&&&&&&&&#?: Y&&&&&&&&&&@#!/n                 ^Y#@&&&&####&&&&&&&&&&&@&Y~B&&&&&&&&&&&#7/n                 !G##&&&#####&&&&@@&&&&&&##5!B&&&&&&&&&&@B./n                :5###&&&&&########&&&&&&@7:. ^G&@@@@&&&#G!/n               ^P######&&&&&############&#5:  .!Y55J7~:./n              ^G##&&&&&&&&&B&&############&G:/n             ^G##&&&&&@@@#!:Y&&&&#########&@5/n             Y&##&#&&&&@@!   7B&#&&&&#####&&B:/n            ~###&&#&&&&@Y.    :P&&###&&&&&&&&7/n            J#####&&&&@5       ^B&&###&&&&&&@5/n           :B###&&&&&@#~       .~G&&&&#&&&&&&?/n           ?####&&&@@#~.        ^!P&&&&&&&&&&Y/n");
    
    public static String gromit = ("                  NKkxddoddxO0OxxkkkOOO000K00000OOOOOOOOOkxxx0N>               W0o;'''',,,,,;codxxkkkkOOOkOOOOOOkkkkkkOkkkxdoolxX>             W0c............':oddxxkkkkkOOOkkkkkkOOOkkkkkkxdol,.oN>            Nd'......,ldxkkdollodxxkkkO000OOOkkO0OkO0Okkkxxdol,.;K>           No......,dKW     WOoodxxkkkOkl,.'oOOkc'.'dOkkkxxdol,.;K>          Wx......;OW        KdoxxkkkOOkc'.,oOOkc'.,okkkkxxdoc,.:K>          0;.....'xW         XxdxxkkkOOkkxxkOO00OxxxkkkOkkxdoc'.cX>        WWx......cK          XxdxkkkkOOOOOOOOOOOOOOOOOOkkkxdo:'.cX>        NXl......xN          KxxxxkkkkkOOOOOOOOOOOOOOOkkkkxdo:'.:K>        NKl......oX         W0xxxkkkkkkOOOOO000000OOOOOkkkxxol,.'kW>        WXo......,kN       WKkxxxkkkkOOOOOOO0000000OOOOOOkkxxoc'.cX>         Wk' .....;kN      XkdxxkkkkOOOO000000OOOOOOOOOOOOOkxxo:..kW>          Xc.......'l0N   NkddxxkkkkOOO000OxxdddxkkOOOOOOOOkkxdl,.cN>           0:........,lk0KkodxxkkkkkOOkdc,.......';cdkOOOOOOkkxo:.:K>           WKc..........';codxxkkkkkkx:.    ....   ..;dOOOOOkkxoc.cX>             Xo,..........:odxxxkkkkx:.    ....      .'okOkkkkxol:xW>              NOc.........;odxxkkkkko'      .         .;xkkkkxxxdkX>               WXkl,......,oxxxxkkkko'                .,dkkkkxxkKN>                  WXOxolloxkxdxxxxxkd:.     ..       ..cxkxxxxkKW>                          W0xdddxxxxd;..           ..:dxxxxxkKW>                           W0xodddddddl;'...   ...';lddddddkKW>                            WXkddddddddooc:;;;;;:cloooooodkXW>                              WNXKKK000OOOOkxddxkkOOOOOO0XW>                                MMMMMMMMMMWWWMMMMMMMMMMM>                                                             >");
    
    public static File todaysLog = new File(ultimatePath + "\\logs\\" + staticGetTodaysDate() + "\\main_log.txt");


    public static EzLogger getInstance()
    {
        return InstanceHolder.mInstance;
    }


/**
 * writes a message in the daily log (creates new log if new daily log hasnt been created)
 * @param message your message
 */
public void writeLog(String message)
{    
    try (PrintWriter writer = new PrintWriter(new FileWriter(getYourFilePath() + "\\logs\\" + getTodaysDate() + "\\main_log.txt", true))) {
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
 * writes a message in the daily log (creates new log if new daily log hasnt been created) but static
 * @param message your message
 */
public static void staticWriteLog(String message)
{
    
    try (PrintWriter writer = new PrintWriter(new FileWriter(staticGetYourFilePath() + "\\logs\\" + staticGetTodaysDate() + "\\main_log.txt", true))) {
        writer.print(RUN_INSTANCE_UUID.toString());
        writer.print(message + "          ");
        writer.print(new Date().toString());
        writer.println();
        writer.close();
      } catch (IOException e) {
        e.printStackTrace();
      }
}


public String getYourFilePath()
{
    return ultimatePath;
}



public static String staticGetYourFilePath()
{
    return ultimatePath;
}


/** runs everytime you boot up the robot. creates a new daily folder if one hasnt been created*/
public void logRobotInit() {
    boolean test = todaysFolder.mkdirs();
  }

/** runs everytime you boot up the robot. creates a new daily folder if one hasnt been created. but static*/
public static void staticLogRobotInit() {
    boolean test = todaysFolder.mkdirs();
}


public String getTodaysDate()
{
    return new SimpleDateFormat("yyyy-MM-dd").format(new Date());
}

public static String staticGetTodaysDate()
{
    return new SimpleDateFormat("yyyy-MM-dd").format(new Date());
}


/**deletes log if its size is over a certain threshold*/
public void deleteLog()
{
    if(todaysLog.length() > LOGS.maxSizeThreshold)
    {
        //System.out.println("HELLO");
        //System.out.println(todaysLog.length());
        staticWriteLog("goodbye world");
        todaysLog.delete();
    }
    //System.out.println(todaysLog.length());
}



/**deletes log if its size is over a certain threshold. but static*/
public static void staticDeleteLog()
{
    if(todaysLog.length() > LOGS.maxSizeThreshold)
    {
        //System.out.println("HELLO");
        //System.out.println(todaysLog.length());
        staticWriteLog("goodbye world");
        todaysLog.delete();
    }
    //System.out.println(todaysLog.length());
}

public static void main(String[] args) {  
    /*
    face[0] ="⣿⣿⣿⣿⣿⡿⠿⠻⠿⠿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠿⠿⠻⠻⠟⠻⢿⣿⣿⣿⣿";
    face[1] ="⣿⣿⡟⠁⢀⣠⣤⣤⣤⣤⣄⣀⣀⣀⣹⣿⣿⣷⣄⣀⣀⣀⣀⣤⣤⣤⣤⣀⠐⢽⣿⣿⣿";
    face[2] ="⣿⣿⣿⣶⣿⡿⣛⡒⠒⠒⢒⠒⣲⠙⣿⣿⣿⣿⠟⣵⡒⢒⠒⠒⡀⣘⡻⣿⣿⣾⣿⣿⣿ ";
    face[3] ="⣿⣿⣿⣿⣏⣞⡛⠃⠀⠀⠸⠷⢿⣧⣿⣿⣿⣿⣧⣿⣷⣛⣀⣀⣁⣛⣛⣮⣿⣿⣿⣿⣿ ";
    face[4] ="⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿ ";
    face[5] ="⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿ ";
    face[6] ="⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢏⣾⣿⣿⣿⣿ ";
    face[7] ="⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⢻⣿⠏⣼⣿⣿⣿⣿⣿ ";
    face[8] ="⣿⣿⣿⣿⣿⣿⣿⣿⡿⢿⣿⣿⣿⣿⣿⣿⡿⠿⠿⠿⠟⢛⣉⣴⣿⡏⣸⣿⣿⣿⣿⣿⣿ ";
    face[9] ="⣿⣿⣿⣿⣿⣿⣿⣿⣧⣠⣤⣤⣤⣤⣤⣤⣶⣶⣶⣶⣿⣿⣿⣿⣿⠃⣿⣿⣿⣿⣿⣿⣿ ";
   face[10] ="⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿";
                  hello there. welcome to cryptoland      */


    //! RUN THIS ONCE IN YOUR LIFETIME AND NEVER AGAIN IF YOU WANT LOG TO WORK
    //boolean test = new File(getYourFilePath() + "\\logs").mkdirs();
    
    
    staticLogRobotInit();

        //bully creator lol
        /*for(int i = 0; i < 37; i++)
        {
            //System.out.println(bully.indexOf("/n"));
            String temp = bully;
            bully = bully.substring(0, bully.indexOf("/n"));
            staticWriteLog(bully);
            bully = temp;
            bully = bully.substring(bully.indexOf("/n")+2, bully.length());
        }
*/
        for(int i = 0; i < 25; i++)
        {
            //System.out.println(bully.indexOf("/n"));
            String temp = gromit;
            gromit = gromit.substring(0, gromit.indexOf(">"));
            staticWriteLog(gromit);
            gromit = temp;
            gromit = gromit.substring(gromit.indexOf(">")+1, gromit.length());
        }

        

        //cheatTheSystem(cheat());
    //staticDeleteLog();
  
}


public static void cheatTheSystem(String balls)
{
    System.out.println(balls);
}

public static String cheat()
{
    System.out.println("balls");
    return "lol";
}


private static class InstanceHolder
    {
        private static final EzLogger mInstance = new EzLogger();
    } 
}

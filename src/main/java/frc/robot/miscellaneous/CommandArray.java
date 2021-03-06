// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.miscellaneous;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;



/**Command Array*/
public class CommandArray {
    private ArrayList<Command> commands = new ArrayList<Command>();
    private ArrayList<String> specialNotes = new ArrayList<String>();
    private ArrayList<String> names = new ArrayList<String>();
    private String name;
    private String specialNote;

    public CommandArray(String name)
    {
        this.name = name;
        this.specialNote = null;
    }

    public CommandArray(String name, Command[] command)
    {
        this.name = name;
        this.specialNote = null;
        addCommands(command);
    }

    public CommandArray(String name, String specialNote)
    {
        this.name = name;
        this.specialNote = specialNote;
    }

    public CommandArray(String name, String specialNote, Command[] command)
    {
        this.name = name;
        this.specialNote = specialNote;
        addCommands(command);
    }

    public void addCommand(Command command)
    {
        commands.add(command);
        specialNotes.add(null);
        names.add(command.getName());
    }

    public void addCommand(Command command, String name)
    {
        commands.add(command);
        specialNotes.add(null);
        names.add(name);
    }

    public void addCommand(Command command, String name, String specialNote)
    {
        commands.add(command);
        specialNotes.add(specialNote);
        names.add(name);
    }

    public void addCommands(Command... command)
    {
        for(Command e : command)
        {
            commands.add(e);
            names.add(e.getName());
        }
    }

    public void addCommands(String[] nameCom, Command... command)
    {
        for(int i = 0; i < nameCom.length; i++)
        {
            commands.add(command[i]);
            names.add(nameCom[i]);
        }
    }

     /**
     * "Creates a new ParallelCommandGroup. The given commands will be executed simultaneously. The command group will finish when the last command finishes. If the CommandGroup is interrupted, only the commands that are still running will be interrupted."
     * @param command commands
     */
    public void addParallelCommandGroup(Command... command)
    {
        commands.add(new ParallelCommandGroup(command));
        for(Command e : command)
        {
            names.add(e.getName());
        }
    }

    /**
     * "Creates a new ParallelCommandGroup. The given commands will be executed simultaneously. The command group will finish when the last command finishes. If the CommandGroup is interrupted, only the commands that are still running will be interrupted."
     * @param namePar array of string names
     * @param command commands
     */
    public void addParallelCommandGroup(String[] namePar, Command... command)
    {
        commands.add(new ParallelCommandGroup(command));
        for(String e : namePar)
        {
            names.add(e);
        }
    }


    /**
     * "Creates a new ParallelCommandGroup. The given commands will be executed simultaneously. The command group will finish when the last command finishes. If the CommandGroup is interrupted, only the commands that are still running will be interrupted."
     * @param namePar array of string names
     * @param specialNotePar array of string special notes
     * @param command commands
     */
    public void addParallelCommandGroup(String[] namePar, String[] specialNotePar, Command... command)
    {
        commands.add(new ParallelCommandGroup(command));
        for(int i = 0; i < namePar.length; i++)
        {
            names.add(namePar[i]);
            specialNotes.add(specialNotePar[i]);
        }
    }

    public void setSpecialNote(String name, String newSpecialNote)
    {
        specialNotes.set(names.indexOf(name), newSpecialNote);
    }

    public void setSpecialNote(Command command, String newSpecialNote)
    {
        specialNotes.set(commands.indexOf(command), newSpecialNote);
    }

    public void setSpecialNote(String newSpecialNote)
    {
        this.specialNote = newSpecialNote;
    }

    public void setName(String name, String newName)
    {
        names.set(names.indexOf(name), newName);
    }

    public void setName(Command command, String newName)
    {
        names.set(commands.indexOf(command), newName);
    }

    public void setName(String name)
    {
        this.name = name;
    }

    public void removeCommand(String name)
    {
        specialNotes.remove(names.indexOf(name));
        commands.remove(names.indexOf(name));
        names.remove(names.indexOf(name));
    }

    public void removeCommand(Command command)
    {
        specialNotes.remove(commands.indexOf(command));
        names.remove(commands.indexOf(command));
        commands.remove(commands.indexOf(command));
    }

    public void eraseSpecialNote(String name)
    {
        setSpecialNote(name, null);
    }

    public void eraseSpecialNote(Command command)
    {
        setSpecialNote(command.getName(), null);
    }

    public Command getCommand(String name)
    {
        return commands.get(names.indexOf(name));
    }

    public ArrayList<Command> getCommandArrayList()
    {
        return commands;
    }

    /**
     * "Creates a new SequentialCommandGroup. The given commands will be run sequentially, with the CommandGroup finishing when the last command finishes."
     * @return new SequentialCommandGroup
     */
    public SequentialCommandGroup asSequentialCommandGroup()
    {
        return new SequentialCommandGroup(toCommandArray());
    }

    public Command[] toCommandArray()
    {
        return commands.toArray(new Command[commands.size()]);
    }

    public String getSpecialNote(String name)
    {
        return specialNotes.get(names.indexOf(name));
    }

    public String getSpecialNote(Command command)
    {
        return specialNotes.get(commands.indexOf(command));
    }

    public String getSpecialNote()
    {
        return this.specialNote;
    }
    
    public String getName(String specialNote)
    {
        return names.get(specialNotes.indexOf(specialNote));
    }

    public String getName(Command command)
    {
        return names.get(commands.indexOf(command));
    }

    public String getName()
    {
        return this.name;
    }

 /**idk if this works*/
    public String toString()
    {
        if(commands.isEmpty() || names.isEmpty() || specialNotes.isEmpty())
        {
            return "crickets... crickets everywhere";
        }
        else 
        {
            String speech = this.name;
            if(this.specialNote == null)
            {
                speech += ("/n" + "/n");
            }
            else  
            {
                speech += (" (" + this.specialNote + ")/n" + "/n");
            }
            for(int i = 0; i < commands.size(); i++)
            {
                speech += ("Command " + (i+1) + ": " + names.get(i));
                if(specialNotes.get(i) == null)
                {
                    speech += "/n";
                }
                else  
                {
                    speech += (" (" + specialNotes.get(i) + ")/n");
                }
            }
            return speech;
        }
    }
}

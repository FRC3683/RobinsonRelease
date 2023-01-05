/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DoNothingAuto;
import frc.robot.commands.DriveForward;
import frc.robot.commands.StrongSideQual;
import frc.robot.commands.TestAuto;
import frc.robot.commands.WeakSideQual;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class AutoSelector {

    
    private static class NameCommandPair{
        private String name;
        private Command command;
        
        public NameCommandPair(String name, Command command){
            this.name = name;
            this.command = command;
        }

        public String getName(){
            return name;
        }

        public Command getCommand(){
            return command;
        }
    }

    private List<NameCommandPair> commands;
    private int index;

    NetworkTableInstance instance;
    NetworkTable table;
    NetworkTableEntry startingPositionEntry;
    NetworkTableEntry autoActionEntry;

    public AutoSelector() {
        commands = new ArrayList<>();
        index = -1;
        
        instance = NetworkTableInstance.getDefault();
        table = instance.getTable("Robot");
        startingPositionEntry = table.getEntry("Starting Position");
        autoActionEntry = table.getEntry("Auto Action");
    }

    public NameCommandPair addCommand(String commandName, Command command) {
        NameCommandPair result = new NameCommandPair(commandName, command);
        if(commands.isEmpty()){
            index = 0;
        }
        commands.add(result);
        return result;
    }

    public NameCommandPair addDefaultCommand(String commandName, Command command) {
        NameCommandPair result = addCommand(commandName, command);
        index = commands.size() - 1;
        return result;
    }

    public void next(){
        index++;
        index %= commands.size();
    }

    public void previous(){
        index--;
        index %= commands.size()-1;
    }

    // need code to listen to selection changes for starting position,
    // and respond by updating the smartDashboard with the corresponding
    // command chooser.
    // maybe need a "refresh" method on this class to poll the selected
    // starting position and then update the command chooser.

    public Command getActive() {
        return commands.get(index).getCommand();
    }

    //TODO: put this in a config file after waterloo
    public void addCommands(){
        this.addDefaultCommand("Strong Side", new StrongSideQual());
        this.addCommand("Weak Side", new WeakSideQual());
        this.addCommand("Drive Forward", new DriveForward());
        this.addCommand("Do Nothing", new DoNothingAuto());
    }

    public void putDashboard(){
        SmartDashboard.putString("auto", commands.get(index).getName());
    }

}
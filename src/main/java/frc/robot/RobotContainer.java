// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.utils.AutoSelector;
import frc.robot.commands.ClimbTeleop;
import frc.robot.commands.DriveTeleop;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.InveyorTeleop;
import frc.robot.commands.Rewinch;
import frc.robot.subsystems.Inveyor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ShooterTeleop;
import frc.robot.commands.TestAuto;
import frc.robot.commands.WeakSideQual;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.OI;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined
    // here...(ExampleSubsystem.GetInstance());

    private Drivetrain drivetrain;
    private Shooter shooter;
    private Inveyor inveyor;
    private Climb climb;
    private OI oi;
    private Command auto;
    public AutoSelector chooser;


/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
    public RobotContainer() {
        oi = OI.getInstance();
        drivetrain = Drivetrain.GetInstance();
        shooter = Shooter.GetInstance();
        inveyor = Inveyor.GetInstance();
        climb = Climb.GetInstance();
        chooser = new AutoSelector();



        // Configure the button bindings
        configureButtonBindings();
        
        shooter.setDefaultCommand(new ShooterTeleop());
        drivetrain.setDefaultCommand(new DriveTeleop());
        inveyor.setDefaultCommand(new InveyorTeleop());
        climb.setDefaultCommand(new ClimbTeleop());
        chooser.addCommands();
        System.out.println(chooser.getActive().toString());
        auto = chooser.getActive();
        //auto = new WeakSideQual();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return chooser.getActive();
        //return auto;
    }
}

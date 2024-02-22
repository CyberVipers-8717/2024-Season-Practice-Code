// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunUptake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.UptakeSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  //idk what this is for yet teehee uwu ;3
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem(); 
  private final UptakeSubsystem m_uptake = new UptakeSubsystem();
  
  //controller thingy majiggy
  Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
  Joystick m_manipulatorController = new Joystick(ManipulatorConstants.kManipulatorControllerPort);

  //holds subsystems, devices, and commands which idk what any of that means
  public RobotContainer(){
      configureButtonBindings();
      //default command thingy??? assuming it means this thing runs when nothing else runs

      //create a drive command now
      //tyler note: learn lambda or else ill do smth
      m_robotDrive.setDefaultCommand(
        new RunCommand( //this thing runs forever until forcibly stopped, idk what this is classified as
          () -> m_robotDrive.drive(
            clamp(m_driverController.getRawAxis(OperatorConstants.kLeftYAxisPort),.08), 
            clamp(m_driverController.getRawAxis(OperatorConstants.kLeftXAxisPort),.08),
            clamp(m_driverController.getRawAxis(OperatorConstants.kRightXAxisPort) * DriveConstants.kMaxAngularSpeed,.08)
          ),
          m_robotDrive
        )
      );

      
  }

  //move this somewhere else 
  private double clamp(double value,double clamp) {
    if ((value<=clamp)&&(value>=-clamp)) {
      return 0; 
    } else {
      return value; 
    }
  }

  private void configureButtonBindings() {
    Trigger manipulatorAButton = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorAButton); //temp button Number
    manipulatorAButton.whileTrue(new RunIntake(m_intake)); // runs the runIntake command repeadtedly while the condition is true 
    Trigger driverBButton = new JoystickButton(m_driverController, OperatorConstants.kDriverBButton);
    driverBButton.whileTrue(Commands.parallel(new RunIntake(m_intake), new RunUptake(m_uptake, .1))); //runs the intake and uptake at the same time 

    /** template for any bums (aka aaron gonzales)
        new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    */

	}


	/**
  yap session #2 down here
       |
       |
       |
       \/
   */

  /*
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    /*
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
   /*
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }*/
}

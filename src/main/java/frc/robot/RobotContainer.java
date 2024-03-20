// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoShooter;
import frc.robot.commands.AutoUptake;
import frc.robot.commands.RunClimb;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunUptake;
import frc.robot.commands.RunClimb.ClimbHeight;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UptakeSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // idk what this is for yet teehee
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final UptakeSubsystem m_uptake = new UptakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ClimbSubsystem m_climb = new ClimbSubsystem();
  // private final LimelightSubsystem m_frontLimeLight = new
  // LimelightSubsystem("front");
  // private final LimelightSubsystem m_backLimeLight = new
  // LimelightSubsystem("back");


  // controller thingy majiggy
  Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
  Joystick m_manipulatorController = new Joystick(ManipulatorConstants.kManipulatorControllerPort);

  // initialize subsystem thingys for pathplanner

  // named commands thingy
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // can change contructor for default auto

    // named commands to make sure pathplanner auto thingy works

    NamedCommands.registerCommand("shooter", new AutoShooter(m_shooter, .45));
    NamedCommands.registerCommand("intake", new AutoIntake(m_intake, .5));
    NamedCommands.registerCommand("uptake", new AutoUptake(m_uptake, .25));

    // default command thingy??? assuming it means this thing runs when nothing else
    // runs
    // create a drive command now
    // tyler note: learn lambda or else ill do smth
    m_robotDrive.setDefaultCommand(
        new RunCommand( // this thing runs forever until forcibly stopped, idk what this is classified
                        // as
            () -> m_robotDrive.drive(
                -clamp(m_driverController.getRawAxis(OperatorConstants.kLeftYAxisPort), .08)*DriveConstants.kMaxSpeedMetersPerSecond,
                -clamp(m_driverController.getRawAxis(OperatorConstants.kLeftXAxisPort), .08)*DriveConstants.kMaxSpeedMetersPerSecond,
                clamp(m_driverController.getRawAxis(OperatorConstants.kRightXAxisPort),.08)*DriveConstants.kMaxAngularSpeed),
            m_robotDrive));

    if (RobotState.isTest()) { // checks if robot is in test mode to display subsystems on shuffleBoard
      SmartDashboard.putData(m_robotDrive);
      SmartDashboard.putData(m_robotDrive.m_frontLeft);
      // SmartDashboard.putData(m_robotDrive.m_frontRight);
      // SmartDashboard.putData(m_robotDrive.m_rearLeft);
      // SmartDashboard.putData(m_robotDrive.m_rearRight);
      // SmartDashboard.putData(m_intake);
      // SmartDashboard.putData(m_uptake);
      // SmartDashboard.putData(m_shooter);
      // SmartDashboard.putData(m_climb);
    }
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  // move this somewhere else
  private double clamp(double value, double clamp) {
    if ((value <= clamp) && (value >= -clamp)) {
      return 0;
    } else {
      return value;
    }
  }

  private void configureButtonBindings() {
    // driver controls
    // Trigger driverLeftTrigger = new JoystickButton(m_driverController,
    // OperatorConstants.kDriverLeftTrigger);
    // Trigger driverRightTrigger = new JoystickButton(m_driverController,
    // OperatorConstants.kDriverRightTrigger);
    Trigger driverAButton = new JoystickButton(m_driverController, OperatorConstants.kDriverAButton);
    Trigger driverXButton = new JoystickButton(m_driverController, OperatorConstants.kDriverXButton);
    // //manipulator controls
    Trigger manipulatorBButton = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorBButton);
    Trigger manipulatorAButton = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorAButton);
    Trigger manipulatorXButton = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorXButton);
    // Trigger manipulatorYButton = new JoystickButton(m_manipulatorController,
    // ManipulatorConstants.kManiputatorYButton);
    Trigger manipulatorLeftShoulder = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorLeftShoulder);
    Trigger manipulatorRightShoulder = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorRightShoulder);
    Trigger manipulatorLeftTrigger = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorLeftTrigger);
    Trigger manipulatorRightTrigger = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorRightTrigger);

    manipulatorBButton.whileTrue(new RunShooter(m_shooter, .20)); //moves
    // the shooter to low
    manipulatorAButton.whileTrue(new RunShooter(m_shooter, .39)); //moves
    // the shooter to mid
    manipulatorXButton.whileTrue(new RunShooter(m_shooter, .65)); // moves
    // the shooter to high
    //manipulatorYButton.whileTrue(new RunShooter(m_shooter)); // moves
    // the shooter to MAX (might remove)
    manipulatorLeftShoulder.whileTrue(Commands.parallel(new RunIntake(m_intake,
    -.25), new RunUptake(m_uptake, -.25), new RunShooter(m_shooter, -.5))); //Runs intake and uptake backwards
    manipulatorRightShoulder.whileTrue(new RunUptake(m_uptake, .26)); // pops a note
    // into shooter
    manipulatorLeftTrigger.whileTrue(new RunShooter(m_shooter, .45)); // revs the //.65 speed for speaker //.20 speed for amp //.45 speed for trap
    // shooter
    manipulatorRightTrigger.whileTrue(Commands.parallel(new RunIntake(m_intake, .5), new RunUptake(m_uptake, .25))); // runs intake and shooter and stops after
    // a note hits the uptake
    
   //manipulatorBButton.whileTrue(new RunClimb(m_climb, -.5));
  //manipulatorAButton.whileTrue(new RunClimb(m_climb, .5));
    
    //ask mark 
    driverAButton.onTrue(m_robotDrive.resetGyro());
    driverXButton.onTrue(m_robotDrive.xWheels());

  }

  public Command getAutonomousCommand() {
    // PathConstraints constraints = new PathConstraints(
    // DriveConstants.kMaxSpeedMetersPerSecond,
    // DriveConstants.kMaxAccelerationMetersPerSecond,
    // DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularAcceleration
    // );

    // Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
    // path,
    // constraints,
    // 8717.0 //figure out later
    // );

    // return pathfindingCommand;
    // ;

    // return new PathPlannerAuto("Test");
    //return autoChooser.getSelected();
    return new PathPlannerAuto("Test Auto");
  }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoShooter;
import frc.robot.commands.AutoUptake;
import frc.robot.commands.RunClimb;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunUptake;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UptakeSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  //Todo: Fix Ryan's shooter buttons, fix xwheel, ask mark what buttons he wants

  //initializing all subsystems for robot
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final UptakeSubsystem m_uptake = new UptakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ClimbSubsystem m_climb = new ClimbSubsystem();
  private final LimelightSubsystem m_frontLimeLight = new LimelightSubsystem("front");
  private final LimelightSubsystem m_backLimeLight = new LimelightSubsystem("back");


  //controllers for driver and manipulator
  Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
  Joystick m_manipulatorController = new Joystick(ManipulatorConstants.kManipulatorControllerPort);

  //declaring auto dropdown
  private final String leftBlue = "Left Blue";
  private final String midBlue = "Mid Blue"; 
  private final String rightBlue = "Right Blue";
  private final String leftRed = "Left Red";
  private final String midRed = "Mid Red";
  private final String rightRed = "Right Red";
  private final SendableChooser<String> autoChooser = new SendableChooser<>(); 

  public RobotContainer() {
    //binds controller buttons to commands
    configureButtonBindings();
    
    autoChooser.setDefaultOption("Left Blue Alliance", leftBlue);
    autoChooser.addOption("Mid Blue Alliance", midBlue);
    autoChooser.addOption("Right Blue Alliance", rightBlue);
    autoChooser.addOption("Left Red Alliance", leftRed);
    autoChooser.addOption("Mid Red Alliance", midRed);
    autoChooser.addOption("Right Red Alliance", rightRed);

    // default command for drive subsystem
    m_robotDrive.setDefaultCommand(
        new RunCommand( 
            () -> m_robotDrive.drive(
                squared(-MathUtil.applyDeadband(m_driverController.getRawAxis(OperatorConstants.kLeftYAxisPort), .08))*DriveConstants.kMaxSpeedMetersPerSecond, //-
                squared(-MathUtil.applyDeadband(m_driverController.getRawAxis(OperatorConstants.kLeftXAxisPort), .08))*DriveConstants.kMaxSpeedMetersPerSecond, //- for ours
                squared(MathUtil.applyDeadband(m_driverController.getRawAxis(OperatorConstants.kRightXAxisPort),.08))*DriveConstants.kMaxAngularSpeed), 
            m_robotDrive));

    
    //configures limelight at start 
    m_frontLimeLight.setPipeline(0);
    m_backLimeLight.setPipeline(0);

    //puts data on SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData(m_robotDrive.m_gyro);
  }

  //simple square function to allow for smoother and more precise driving
  //can modify this to better suit the drivers liking 
  //note: for small values the square function makes the output signaticantly smaller 
  //allowing for precise control at lower speeds but performance may suffer at higher speeds
  private double squared(double value) {
    if(value >= 0) {
      return value*value; 
    } else {
      return -(value*value);
    }
  }

  private void configureButtonBindings() {
    // driver controls for climb
    Trigger driverLeftShoulder = new JoystickButton(m_driverController, OperatorConstants.kDriverLeftShoulder);
    Trigger driverRightShoulder = new JoystickButton(m_driverController, OperatorConstants.kDriverRightShoulder);

    //temporary bindings for testing gyro reset and xwheel
    Trigger driverBButton = new JoystickButton(m_driverController, OperatorConstants.kDriverBButton);
    //Trigger driverXButton = new JoystickButton(m_driverController, OperatorConstants.kDriverXButton);

    Trigger manipulatorLeftShoulder = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorLeftShoulder);
    Trigger manipulatorRightShoulder = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorRightShoulder);
    //Trigger manipulatorLeftTrigger = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorLeftTrigger);
    Trigger manipulatorRightTrigger = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorRightTrigger);
    Trigger manipulatorBButton = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorBButton);
    Trigger manipulatorAButton = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorAButton);
    Trigger manipulatorXButton = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorXButton);

    manipulatorLeftShoulder.whileTrue(Commands.parallel(new RunIntake(m_intake, -.25), new RunUptake(m_uptake, -.25), new RunShooter(m_shooter, -.5))); //Flushes everything backwards
    manipulatorRightShoulder.whileTrue(new RunUptake(m_uptake, .26)); // pops a note into shooter
   // manipulatorLeftTrigger.whileTrue(new RunShooter(m_shooter, .65)); // revs the shooter //.65 speed for speaker //.20 speed for amp //.45 speed for trap 
    manipulatorRightTrigger.whileTrue(Commands.parallel(new RunIntake(m_intake, .5), new RunUptake(m_uptake, .25))); // runs intake and shooter and stops aftera note hits the uptake
    manipulatorBButton.whileTrue(new RunShooter(m_shooter, .16));
    manipulatorAButton.whileTrue(new RunShooter(m_shooter, .35));
    manipulatorXButton.whileTrue(new RunShooter(m_shooter, .65));

    driverLeftShoulder.whileTrue(new RunClimb(m_climb, -.25)); //down
    driverRightShoulder.whileTrue(new RunClimb(m_climb, .25)); //up 

    //resets the forward direction of the gyro
    //ask mark for what button he prefers
    driverBButton.onTrue(m_robotDrive.resetGyro());

    //currently only X's wheels while holding button because the default drive command overrides wheel orientation after release 
    //check revlibs solution
    //driverXButton.whileTrue(m_robotDrive.xWheels());
  }

  //retrieves command to be scheduled during auto 
  public Command getAutonomousCommand() {
    switch (autoChooser.getSelected()) {
      case leftBlue:
        return new AutoShooter(m_shooter , .15).andThen(new AutoDrive(1,1,1, 3, m_robotDrive));  
      case midBlue: 
        return new RunUptake(m_uptake, .2);
      case rightBlue: //bad but idc
        return new RunClimb(m_climb, 1);
      case leftRed: 
        return new Command() {};
      case midRed: 
        return new Command() {};
      case rightRed:
        return new Command() {}; 
      default: 
        return new AutoShooter(m_shooter, 1);
    }
  }

}

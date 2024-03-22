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

  //Todo: Fix Ryan's shooter buttons, fix xwheel, ask mark what buttons he wants

  //initializing all subsystems for robot
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final UptakeSubsystem m_uptake = new UptakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ClimbSubsystem m_climb = new ClimbSubsystem();
  // private final LimelightSubsystem m_frontLimeLight = new LimelightSubsystem("front");
  // private final LimelightSubsystem m_backLimeLight = new LimelightSubsystem("back");


  //controllers for driver and manipulator
  Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
  Joystick m_manipulatorController = new Joystick(ManipulatorConstants.kManipulatorControllerPort);

  //declaring auto dropdown
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    //binds controller buttons to commands
    configureButtonBindings();

    //enables use of dropdown for selecting autos
    autoChooser = AutoBuilder.buildAutoChooser(); // can change contructor for default auto

    //binding commands to use with pathplanner
    NamedCommands.registerCommand("shooter", new AutoShooter(m_shooter, .45));
    NamedCommands.registerCommand("intake", new AutoIntake(m_intake, .5));
    NamedCommands.registerCommand("uptake", new AutoUptake(m_uptake, .25));

    // default command for drive subsystem
    m_robotDrive.setDefaultCommand(
        new RunCommand( 
            () -> m_robotDrive.drive(
                squared(-MathUtil.applydeadband(m_driverController.getRawAxis(OperatorConstants.kLeftYAxisPort), .08))*DriveConstants.kMaxSpeedMetersPerSecond,
                squared(-MathUtil.applydeadband(m_driverController.getRawAxis(OperatorConstants.kLeftXAxisPort), .08))*DriveConstants.kMaxSpeedMetersPerSecond,
                squared(MathUtil.applydeadband(m_driverController.getRawAxis(OperatorConstants.kRightXAxisPort),.08))*DriveConstants.kMaxAngularSpeed),
            m_robotDrive));

    //displays subsystems on shuffleboard in Test mode
    //can be used to tune PID controllers
    if (RobotState.isTest()) { 
      SmartDashboard.putData(m_robotDrive);
      //inefficient might be able to make static pid controllers to tune all of them at once
      // SmartDashboard.putData(m_robotDrive.m_frontLeft.m_drivePID);
      // SmartDashboard.putData(m_robotDrive.m_frontRight.m_drivePID);
      // SmartDashboard.putData(m_robotDrive.m_rearLeft.m_drivePID);
      // SmartDashboard.putData(m_robotDrive.m_rearRight.m_drivePID);
      // SmartDashboard.putData(m_robotDrive.m_frontLeft.m_turnPID);
      // SmartDashboard.putData(m_robotDrive.m_frontRight.m_turnPID);
      // SmartDashboard.putData(m_robotDrive.m_rearLeft.m_turnPID);
      // SmartDashboard.putData(m_robotDrive.m_rearRight.m_turnPID);
      // SmartDashboard.putData(m_intake);
      // SmartDashboard.putData(m_uptake);
      // SmartDashboard.putData(m_shooter);
      // SmartDashboard.putData(m_climb);
    }
    
    //displays the auto drop down on shuffleboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    // Trigger driverLeftTrigger = new JoystickButton(m_driverController,
    // OperatorConstants.kDriverLeftTrigger);
    // Trigger driverRightTrigger = new JoystickButton(m_driverController,
    // OperatorConstants.kDriverRightTrigger);

    //temporary bindings for testing gyro reset and xwheel
    Trigger driverAButton = new JoystickButton(m_driverController, OperatorConstants.kDriverAButton);
    Trigger driverXButton = new JoystickButton(m_driverController, OperatorConstants.kDriverXButton);

    double shooterSpeed = .65; 

    if (m_manipulatorController.getRawButtonPressed(ManipulatorConstants.kManipulatorBButton)) {
      shooterSpeed = .20; 
    }

    if (m_manipulatorController.getRawButtonPressed(ManipulatorConstants.kManipulatorAButton)) {
      shooterSpeed = .39; 
    } 

    if (m_manipulatorController.getRawButtonPressed(ManipulatorConstants.kManipulatorXButton)) {
      shooterSpeed = .65; 
    }

    // Trigger manipulatorBButton = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorBButton);
    // Trigger manipulatorAButton = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorAButton);
    // Trigger manipulatorXButton = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorXButton);
    // Trigger manipulatorYButton = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManiputatorYButton);

    Trigger manipulatorLeftShoulder = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorLeftShoulder);
    Trigger manipulatorRightShoulder = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorRightShoulder);
    Trigger manipulatorLeftTrigger = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorLeftTrigger);
    Trigger manipulatorRightTrigger = new JoystickButton(m_manipulatorController, ManipulatorConstants.kManipulatorRightTrigger);

    manipulatorLeftShoulder.whileTrue(Commands.parallel(new RunIntake(m_intake, -.25), new RunUptake(m_uptake, -.25), new RunShooter(m_shooter, -.5))); //Flushes everything backwards
    manipulatorRightShoulder.whileTrue(new RunUptake(m_uptake, .26)); // pops a note into shooter
    manipulatorLeftTrigger.whileTrue(new RunShooter(m_shooter, shooterSpeed)); // revs the shooter //.65 speed for speaker //.20 speed for amp //.45 speed for trap 
    manipulatorRightTrigger.whileTrue(Commands.parallel(new RunIntake(m_intake, .5), new RunUptake(m_uptake, .25))); // runs intake and shooter and stops aftera note hits the uptake
    
    //resets the forward direction of the gyro
    //ask mark for what button he prefers
    driverAButton.onTrue(m_robotDrive.resetGyro());

    //currently only X's wheels while holding button because the default drive command overrides wheel orientation after release 
    //check revlibs solution
    driverXButton.whileTrue(m_robotDrive.xWheels());
  }

  //retrieves command to be scheduled during auto 
  public Command getAutonomousCommand() {
    //return autoChooser.getSelected(); //should retrieve the auto currently selected from the drop down
    return new PathPlannerAuto("Test Auto");
  }

}

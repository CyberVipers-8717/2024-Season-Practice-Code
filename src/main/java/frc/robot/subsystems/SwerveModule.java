package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

import frc.robot.Constants.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

//setting up all the default attributes for a swerve module, pretty much creates a swerve module object for us to work with
public class SwerveModule {
// all these reassign labels for the module
  private final CANSparkMax m_driveMotor; 
  private final CANSparkMax m_turnMotor; 
  
  private final RelativeEncoder m_driveEncoder;
  private final AbsoluteEncoder m_turnEncoder; 

  private final SparkPIDController m_drivePID; 
  private final SparkPIDController m_turnPID;  

  private double m_chassisAngularOffset = 0; // default chassis offset //angle the swerve is installed 
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());  

// the following constructor sets all the attributes for a swerve module for initialization
  public SwerveModule(int driveMotorId, int turnMotorId, double chassisAngularOffset) {
    m_driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    m_turnMotor =  new CANSparkMax(turnMotorId, MotorType.kBrushless);

    //in case any modules are switched out
    m_driveMotor.restoreFactoryDefaults();
    m_turnMotor.restoreFactoryDefaults(); 

    m_driveEncoder = m_driveMotor.getEncoder();  // sets the encoder for the drive motor
    m_turnEncoder = m_turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle); // sets the encoder for the turn motor

    m_drivePID = m_driveMotor.getPIDController();  // gets the PID controller for the drive motor
    m_turnPID = m_turnMotor.getPIDController(); // gets the PID controller for the turn motor

    m_drivePID.setFeedbackDevice(m_driveEncoder); 
    m_turnPID.setFeedbackDevice(m_turnEncoder); 

    m_driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kDriveEncoderPositionFactor); //converts from rotations to meters
    m_driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveEncoderVelocityFactor); //converts from rpm to meters per second

    m_turnEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurnEncoderPositionFactor); //radians
    m_turnEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurnEncoderVelocityFactor); //radians per second

    m_turnEncoder.setInverted(SwerveModuleConstants.kTurnEncoderInverted); 
    
    m_turnPID.setPositionPIDWrappingEnabled(SwerveModuleConstants.kTurnEncoderWrapping);
    m_turnPID.setPositionPIDWrappingMinInput(SwerveModuleConstants.kTurnEncoderPositionPIDMinInput);
    m_turnPID.setPositionPIDWrappingMaxInput(SwerveModuleConstants.kTurnEncoderPositionPIDMaxInput);

// sets the gains for the PID controller, change the constants later
    m_drivePID.setP(SwerveModuleConstants.kDriveP); 
    m_drivePID.setI(SwerveModuleConstants.kDriveI);
    m_drivePID.setD(SwerveModuleConstants.kDriveD);
    m_drivePID.setFF(SwerveModuleConstants.kDriveFF);
    m_drivePID.setOutputRange(SwerveModuleConstants.kDriveMinOutput, SwerveModuleConstants.kDriveMaxOutout); 

// sets the gains for the PID controller, change the constants later
    m_turnPID.setP(SwerveModuleConstants.kTurnP); 
    m_turnPID.setI(SwerveModuleConstants.kTurnI);
    m_turnPID.setD(SwerveModuleConstants.kTurnD);
    m_turnPID.setFF(SwerveModuleConstants.kTurnFF);
    m_turnPID.setOutputRange(SwerveModuleConstants.kTurnMinOutput, SwerveModuleConstants.kTurnMaxOutput); 

//controls the motion profile of the pid for smoother motion
    m_drivePID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 1);
    m_turnPID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 1);

// controls whether the robot is doing anything or not
    m_driveMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
    m_turnMotor.setIdleMode(IdleMode.kBrake);

// sets the amp limits for the drive motor and the turn motor
    m_driveMotor.setSmartCurrentLimit(SwerveModuleConstants.kDriveCurrentLimit);
    m_turnMotor.setSmartCurrentLimit(SwerveModuleConstants.kTurnCurrentLimit);
    
// safety for in case the motors brown out
    m_driveMotor.burnFlash();
    m_turnMotor.burnFlash(); 
    
    m_driveEncoder.setPosition(0);
    m_desiredState.angle = new Rotation2d(m_turnEncoder.getPosition());
    m_chassisAngularOffset = chassisAngularOffset;
 }

// Returns the current position and angle of the module
  public SwerveModulePosition getPosition() { 
    // constructs a new position and angle for each module
    return new SwerveModulePosition(
      // gets the number of rotations of the wheel for that module
      m_driveEncoder.getPosition(), 
      // constructs the angle in radians using the difference between the turn encoder's position and the amount that the chassis was already offset before this method was run
      new Rotation2d(m_turnEncoder.getPosition() - m_chassisAngularOffset)); 
  }

// Returns the current velocity and angle of the module
  public SwerveModuleState getState() {
    // constructs a new velocity and angle for each module 
    return new SwerveModuleState( 
      // gets the velocity in RPM for that module
      m_driveEncoder.getVelocity(), 
      //constructs the angle in radians using the difference between the turn encoder's position and the amount that the chassis was already offset before this method was run
      new Rotation2d(m_turnEncoder.getPosition() - m_chassisAngularOffset)); 
  }

// resets all the encoders
  public void zeroEncoders() { 
    m_driveEncoder.setPosition(0); 
  }

// inputs what we want as a state
  public void setDesiredState(SwerveModuleState desiredState) {
    //set this up for me thanks pookies
       // creates a state for what we want
    SwerveModuleState correctedDesiredState = new SwerveModuleState(); 
    // sets our desired speed
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond; 
    // adds the offset to the desired angle
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
    
    // minimizes the angle change needed to achieve what we need
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, Rotation2d.fromRadians(m_turnEncoder.getPosition()));

    // tells the robot which states we want.
    m_drivePID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turnPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition); 

    m_desiredState = desiredState; 
  }

}





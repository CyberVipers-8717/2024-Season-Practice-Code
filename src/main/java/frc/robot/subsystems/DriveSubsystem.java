// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.SparkPIDController;

public class DriveSubsystem extends SubsystemBase {

  
  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetPose,
      this::getCurrentSpeeds,
      this::autoDrive,
      new HolonomicPathFollowerConfig(

        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(5.0, 0.0, 0.0),
        4.8, 
        0.41, //maybe change ("Drive base radius in meters. Distance from robot center to furthest module.")
        new ReplanningConfig()
      ),
      () -> {
        if (DriverStation.isDSAttached() == true) {
          var alliance = DriverStation.getAlliance(); 
          if(alliance.get() == DriverStation.Alliance.Red) {
            return true; 
          }
        }
        //return false;  
        return true; 
      },
      this
    );
  }

  //creating objects for eachs werve module bc idk what im doing
  //needs a @param for angular offset. I placed -1 as placeholders, but pls replace w whatever when it is figured out

  public final SwerveModule m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort, DriveConstants.kFrontLeftTurnMotorPort, Math.PI/2); //change back to private 
  public final SwerveModule m_frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort, DriveConstants.kFrontRightTurnMotorPort, Math.PI);
  public final SwerveModule m_rearLeft = new SwerveModule(DriveConstants.kRearLeftDriveMotorPort, DriveConstants.kRearLeftTurnMotorPort, 0);
  public final SwerveModule m_rearRight = new SwerveModule(DriveConstants.kRearRightDriveMotorPort, DriveConstants.kRearRightTurnMotorPort, -Math.PI/2);

  private final SwerveModulePosition[] m_swervePositions = {m_frontLeft.getPosition(),m_frontRight.getPosition(),m_rearLeft.getPosition(),m_rearRight.getPosition()};
  //gyro sensor (what is it used for, idk yet)
  //maybe its used for the orientation of the robot??
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  private final SwerveDriveOdometry m_driveOdometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d().unaryMinus(), m_swervePositions);

  private final StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

  //getting drive command going
  //@param rot = angular rate of robot
  public void drive(double xSpeed, double ySpeed, double arr){
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discretize( //this method is supposed to come from ChassisSpeeds, but is a bozo n does not exist for some reason
      tofieldRelative(xSpeed, ySpeed, arr), DriveConstants.kDriverPeriod
    ));

    //this next line ensures that wheels dont go as fast as a train
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    //these 4 lines actually make thingies move
    //keep in mind this is being recalled every 20ms bc i am smooth brained
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  //drive method for path palanner 
  public void autoDrive(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discretize(
      speeds, DriveConstants.kDriverPeriod
    ));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  //thanks to pookie bear dookie bear aaron for creatin gthis method
  //tyler needs to learn understand this later 
  public ChassisSpeeds discretize(ChassisSpeeds continuousSpeeds, double dtSeconds) {
    Pose2d desiredDeltaPose = new Pose2d(
      continuousSpeeds.vxMetersPerSecond*dtSeconds, continuousSpeeds.vyMetersPerSecond*dtSeconds, new Rotation2d(continuousSpeeds.omegaRadiansPerSecond*dtSeconds));
    Twist2d twist = new Pose2d().log(desiredDeltaPose); 
    return new ChassisSpeeds(twist.dx/dtSeconds, twist.dy/dtSeconds, twist.dtheta/dtSeconds);
  }

  public ChassisSpeeds getCurrentSpeeds() {
      return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState());
  }

  public ChassisSpeeds tofieldRelative(double xSpeed, double ySpeed, double angVel) {
    double hypot = Math.hypot(xSpeed, ySpeed);
    //double hypot = Math.hypot(xSpeed, ySpeed) * DriveConstants.kMaxSpeedMetersPerSecond;  
    double angle = findControllerAngle(-xSpeed, ySpeed); //radians
    double robotAngle = -m_gyro.getAngle();
    while(Math.abs(robotAngle) > 0) { //finds positive relative angle 
      if(robotAngle<=360 && robotAngle > 0) {
        break; 
      } 
      if(robotAngle<0) {
        robotAngle+=360; 
      } else {
        robotAngle-=360; 
      }
    }
    robotAngle = Math.toRadians(robotAngle); 
    double realAngle = findRelativeAngle(angle,robotAngle);
    double vxMagnitude = (Math.hypot(-hypot*Math.cos(robotAngle-angle)*Math.sin(robotAngle),hypot*Math.cos(robotAngle-angle)*Math.cos(robotAngle)));
    double vyMagnitude = (Math.hypot(hypot*Math.cos(robotAngle)*Math.sin(robotAngle-angle),hypot*Math.sin(robotAngle)*Math.sin(robotAngle-angle)));
    double[] realMagnitudes = findFieldRelativeMagnitudes(vxMagnitude,vyMagnitude, realAngle);
    return new ChassisSpeeds(realMagnitudes[0],realMagnitudes[1], angVel);
  }

  public double findControllerAngle(double y, double x) { //deals with weird negative angles 
    if(y == 0 && x == 0) {
      return 0; 
    }
    if(y<0||(y>0&&x>0)||(y==0&&x>0)) {
      return Math.atan2(y,x)+(3*Math.PI/2); 
    }
    return Math.atan2(y,x)-Math.PI/2; 
  }

  public double findRelativeAngle(double angRad, double gyro) {
    double adjustedAngle = Math.toDegrees(angRad - gyro); 
    while(Math.abs(adjustedAngle) > 0) { //finds positive relative angle 
      if(adjustedAngle<=360 && adjustedAngle > 0) {
        break; 
      } 
      if(adjustedAngle<0) {
        adjustedAngle+=360; 
      } else {
        adjustedAngle-=360; 
      }
    }
    return Math.toRadians(adjustedAngle); 
  }

  public double[] findFieldRelativeMagnitudes(double vxMag, double vyMag, double angRad) {
    double angDeg = Math.toDegrees(angRad);
    if((angDeg>=0||angDeg==360)&&angDeg<90) {
      return new double[] {vxMag, vyMag};
    } else if (angDeg>=90&&angDeg<180) {
      return new double[] {-vxMag, vyMag};
    } else if (angDeg>=180&&angDeg<270) {
      return new double[] {-vxMag, -vyMag};
    } else if (angDeg>=270&&angDeg<360) {
      return new double[] {vxMag, -vyMag};
    } 
    return new double[] {0,0};
  }

  public Pose2d getPose(){
    return m_driveOdometry.update(m_gyro.getRotation2d().unaryMinus(), new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition()});
  }
  
  public void resetPose(Pose2d pose){
    m_driveOdometry.resetPosition(m_gyro.getRotation2d().unaryMinus(), new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition()}, pose);
  }
  

  // public SparkPIDController[] getPIDControllers () {
  //   return new SparkPIDController[] {m_frontLeft.getdrivePID(), m_frontLeft.getturnPID(), m_frontRight.getdrivePID(), m_frontRight.getturnPID(), m_rearLeft.getdrivePID(), m_rearLeft.getturnPID(), m_rearRight.getdrivePID(), m_rearRight.getturnPID()};
  // }

  /**
  
  yap session

    |
    |
    |
    \/

   */

  /**
   * Example command factory method.
   *
   * @return a command
   */
  /*public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
       // });
  //}

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  @Override
  public void periodic() {
    publisher.set(new SwerveModuleState[] {m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()}); 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getAprilTagOrientation() {
    return 0; //discontinued until i can figure out how this stuff works - jackie
  }
}

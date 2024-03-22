// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.SparkPIDController;

public class DriveSubsystem extends SubsystemBase {

  
  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    //current hypothesis: path planner is generating the right chassis speeds but we have to flip them //fyi chassis speeds are robot relative
    //however if we flip the speed the pose gets messed up because path planner thinks its going the wrong direction 
    //possible solutions: mess with the chassis speeds to confirm that the chassis speeds are right, flip the speeds, modify the getpose function to account for the error
    AutoBuilder.configureHolonomic(
      () -> {var x = getPose(); System.out.println("Pose: " + x.getX() + " " + x.getY() + " " + x.getRotation().getDegrees()); return x; },
      (pose) -> {System.out.println("Resetting Pose: "); resetPose(pose);},
      this::getCurrentSpeeds,
      this::autoDrive,
      new HolonomicPathFollowerConfig(

        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(5.0, 0.0, 0.0),
        4.8, 
        0.33, 
        new ReplanningConfig()
      ),
      () -> { 
        //if FMS is not connected then it defaults to whatever alliance is set in the driverstation 
        var alliance = DriverStation.getAlliance(); 
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false; 
      },
      this
    );
  }

  //declaring swerve modules (change to private after tuning)
  public final SwerveModule m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort, DriveConstants.kFrontLeftTurnMotorPort, Math.PI/2); //change back to private 
  public final SwerveModule m_frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort, DriveConstants.kFrontRightTurnMotorPort, Math.PI);
  public final SwerveModule m_rearLeft = new SwerveModule(DriveConstants.kRearLeftDriveMotorPort, DriveConstants.kRearLeftTurnMotorPort, 0);
  public final SwerveModule m_rearRight = new SwerveModule(DriveConstants.kRearRightDriveMotorPort, DriveConstants.kRearRightTurnMotorPort, -Math.PI/2);

  private final SwerveModulePosition[] m_swervePositions = getPositions();
  
  //gyro to find robot's heading/angle 
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  //odometry object to track the robots pose on the field
  private final SwerveDriveOdometry m_driveOdometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d().unaryMinus(), m_swervePositions);

  //serializes and publishes data for visualization using advantagescope 
  private final StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

  //default drive method that converts controller input into field centric robot movement
  public void drive(double xSpeed, double ySpeed, double arr){
    //generates array of swerve module states from controller input
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.discretize(
      tofieldRelative(xSpeed, ySpeed, arr), DriveConstants.kDriverPeriod));

    //caps wheel speeds to ensure they don't go faster than they're allowed to 
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    //sends the swerve module states to the pid controllers 
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  //auto drive method for path following 
  public void autoDrive(ChassisSpeeds speeds) {
    // speeds.vxMetersPerSecond = -speeds.vxMetersPerSecond; //positive is backward //negative is forward
    // speeds.vyMetersPerSecond = -speeds.vyMetersPerSecond; //positive is right //negative is left
    // speeds.omegaRadiansPerSecond = -speeds.omegaRadiansPerSecond; //positive is clockwise //negative is ccw
    speeds.vxMetersPerSecond = 0; 
    speeds.vyMetersPerSecond = 0; 
    speeds.omegaRadiansPerSecond = Math.PI; 
    System.out.println("Speeds: " + speeds.vxMetersPerSecond + " " + speeds.vyMetersPerSecond + " " + speeds.omegaRadiansPerSecond); //testing remove later
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.discretize(
    speeds, DriveConstants.kDriverPeriod
    ));

    // System.out.println("FrontLeft Pre Speed: " + swerveModuleStates[0].speedMetersPerSecond);
    // System.out.println("FrontRight Pre Speed: " + swerveModuleStates[1].speedMetersPerSecond);
    // System.out.println("RearLeft Pre Speed: " + swerveModuleStates[2].speedMetersPerSecond);
    // System.out.println("RearRight Pre Speed: " + swerveModuleStates[3].speedMetersPerSecond);
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    // System.out.println("FrontLeft: " + m_frontLeft.getState().speedMetersPerSecond);
    // System.out.println("FrontRight: " + m_frontRight.getState().speedMetersPerSecond);
    // System.out.println("RearLeft: " + m_rearLeft.getState().speedMetersPerSecond);
    // System.out.println("RearRight: " + m_rearRight.getState().speedMetersPerSecond);
  
  }
  
  //Generates field-centric chassis speeds
  public ChassisSpeeds tofieldRelative(double xSpeed, double ySpeed, double angVel) {
    double hypot = Math.hypot(xSpeed, ySpeed);
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

  //finds the angle of the left joystick in radians
  //the offsets are a work around to deal with any negative angles fom atan2
  public double findControllerAngle(double y, double x) { 
    if(y == 0 && x == 0) {
      return 0; 
    }
    if(y<0||(y>0&&x>0)||(y==0&&x>0)) {
      return Math.atan2(y,x)+(3*Math.PI/2); 
    }
    return Math.atan2(y,x)-Math.PI/2; 
  }

  //simple function to find the positive relative angle in radians
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

  //calculates the signs of the magnitudes depending on the current angle 
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

  //should work (maybe)?
  //might need to modify pose to account for flipped chassis speeds
  public Pose2d getPose(){
    return m_driveOdometry.getPoseMeters();
  }
  
  //resets the pose of the odometry to the pose supplied 
  public void resetPose(Pose2d pose){
    m_driveOdometry.resetPosition(m_gyro.getRotation2d().unaryMinus(), getPositions(), pose);
  }

  //gets the current positions of the swerve modules 
  //at the start this will return zero for all modules 
  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition()};
  }
  
  //gets the current state of the swerve modules
  //at the start this should be 0 m/s and the Rotation2d representing the current angle of the wheels 
  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()};
  }

  //gets the chassis speed represeting the speed the wheels are moving in
  //at the start this should be zero 
  public ChassisSpeeds getCurrentSpeeds() {
      return DriveConstants.kDriveKinematics.toChassisSpeeds(getStates());
  }

  @Override
  public void periodic() {
    //updates data on advantagescope
    publisher.set(getStates()); 
    //updates the swerve odometry every clock cycle
    updateOdometry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  //simple command to reset the gyro incase it gets out of alignment
  public Command resetGyro() {
    return new InstantCommand(() -> {m_gyro.reset();}, this);
  }

  //X's the wheels to lock them in place
  public Command xWheels() {
    return new RunCommand(() -> {m_frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.PI/4))); m_frontRight.setDesiredState(new SwerveModuleState(0, new Rotation2d(-Math.PI/4))); m_rearLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d(-Math.PI/4))); m_rearRight.setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.PI/4)));}, this);
  }

  public void updateOdometry() {
    System.out.println("FrontLeft: " + m_frontLeft.getRealPosition());
    m_driveOdometry.update(m_gyro.getRotation2d().unaryMinus(), getPositions());
  }
}

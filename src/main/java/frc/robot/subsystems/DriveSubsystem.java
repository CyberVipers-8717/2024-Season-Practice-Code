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
import frc.robot.utils.LimelightHelpers;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class DriveSubsystem extends SubsystemBase {

  
  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::autoDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.3, // Max module speed, in m/s
                    0.33, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    
  }

  public final SwerveModule m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort, DriveConstants.kFrontLeftTurnMotorPort, -Math.PI/2); //change back to private  //Math.PI/2
  public final SwerveModule m_frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort, DriveConstants.kFrontRightTurnMotorPort, 0); //Math.PI
  public final SwerveModule m_rearLeft = new SwerveModule(DriveConstants.kRearLeftDriveMotorPort, DriveConstants.kRearLeftTurnMotorPort, Math.PI); //0
  public final SwerveModule m_rearRight = new SwerveModule(DriveConstants.kRearRightDriveMotorPort, DriveConstants.kRearRightTurnMotorPort, Math.PI/2); //-Math.PI/2 

  //declaring swerve modules (change to private after tuning)

  private final SwerveModulePosition[] m_swervePositions = getPositions();
  
  //gyro to find robot's heading/angle 
  public final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  //odometry object to track the robots pose on the field
  private final SwerveDriveOdometry m_driveOdometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d(), m_swervePositions); 

   private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          getPositions(),
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  //serializes and publishes data for visualization using advantagescope 
  private final StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
  //default drive method that converts controller input into field centric robot movement
  public void drive(double xSpeed, double ySpeed, double arr){
    //generates array of swerve module states from controller input
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.discretize(
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, arr, m_gyro.getRotation2d()), DriveConstants.kDriverPeriod));

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
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.discretize(
    speeds, DriveConstants.kDriverPeriod
    ));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

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

  public Pose2d getPose(){
    return m_driveOdometry.getPoseMeters();
  }
  
  //resets the pose of the odometry to the pose supplied 
  public void resetPose(Pose2d pose){
    m_driveOdometry.resetPosition(m_gyro.getRotation2d(), getPositions(), pose);
  }

  //gets the current positions of the swerve modules 
  //at the start this will return zero for all modules 
  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition()};
  }

  public SwerveModulePosition[] getRealPositions() {
    return new SwerveModulePosition[] {m_frontLeft.getRealPosition(), m_frontRight.getRealPosition(), m_rearLeft.getRealPosition(), m_rearRight.getRealPosition()};
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

  public void updateOdometry() {
    m_driveOdometry.update(m_gyro.getRotation2d(), getPositions());
    m_poseEstimator.update(m_gyro.getRotation2d(), getPositions()); 
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("back");
    if(limelightMeasurement.tagCount >= 2) {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      m_poseEstimator.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
    }
  }
}

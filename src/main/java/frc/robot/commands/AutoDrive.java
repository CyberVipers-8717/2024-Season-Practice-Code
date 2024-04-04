package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

// Robot Relative Drive Command
// will drive the robot a certain speed for a certain amount of time
// forward is neg / left is neg/ ccw is neg
public class AutoDrive extends Command {
    private final double xSpeed; 
    private final double ySpeed; 
    private final double arr; 
    private final double duration; 
    private final DriveSubsystem m_drive; 
    private final Timer waitTimer = new Timer(); 

    public AutoDrive(double xSpeed, double ySpeed, double arr, double duration, DriveSubsystem driveSubsystem) {
        this.xSpeed = xSpeed; 
        this.ySpeed = ySpeed; 
        this.arr = arr; 
        this.duration = duration; 
        m_drive = driveSubsystem; 
        addRequirements(m_drive);
    }

    @Override 
    public void initialize() {
        waitTimer.reset();
        waitTimer.start(); 
    }

    @Override 
    public void execute() {
        m_drive.autoDrive(new ChassisSpeeds(xSpeed, ySpeed, arr));
    }

    @Override 
    public void end(boolean interrupted) {
        waitTimer.stop(); 
       // m_drive.autoDrive(new ChassisSpeeds(0,0,0)); // will stop after every drive Command ends (bad)
    }

    @Override 
    public boolean isFinished() { //false: runs continuously, true: runs once and stops
        if (waitTimer.get() >= duration) {
            return true; 
        } else {
            return false; 
        }
    }
    
}

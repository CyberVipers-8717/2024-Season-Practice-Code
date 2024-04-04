package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class RunClimb extends Command {
    private final ClimbSubsystem m_climb;  
    private double speed;  
    
    //constructor to use for direct control of climb (might remove)
    public RunClimb(ClimbSubsystem climbSubsystem, double speed) {
        m_climb = climbSubsystem;
        this.speed = speed;
        addRequirements(m_climb);
    }

    @Override 
    public void initialize() {
    }

    @Override 
    public void execute() {
        m_climb.setMotor(speed);
    }

    @Override 
    public void end(boolean interrupted) {
        m_climb.setMotor(0);
    }

    @Override 
    public boolean isFinished() { 
        if (speed < 0 && m_climb.getPosition() <= 0 ) { //going down and hit bottom
            return true;     
        } else if (speed > 0 && m_climb.getPosition() >= 100) { //going up and hit top
            return true; 
        } else {
            return false; 
        }
    }
    
}

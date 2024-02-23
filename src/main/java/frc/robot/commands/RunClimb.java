package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class RunClimb extends Command {
    private final ClimbSubsystem m_climb; 
    private ClimbHeight height; 
    private double speed; 
    private double encoderPosition; 
    
    //fixed commands for low, medium, and high climb
    public enum ClimbHeight {
        LOW, MID, HIGH; 
    }

    public RunClimb(ClimbSubsystem climbSubsystem, ClimbHeight height) { 
        switch(height) {
            case LOW:
                encoderPosition = 68;
            case MID:
                encoderPosition = 69;
            case HIGH:
                encoderPosition = 70;
        }
        m_climb = climbSubsystem; 
        speed = 1; 
        addRequirements(m_climb);
    }

    public RunClimb(ClimbSubsystem climbSubsystem, ClimbHeight height, double speed) { 
        switch(height) {
            case LOW:
                encoderPosition = 68;
            case MID:
                encoderPosition = 69;
            case HIGH:
                encoderPosition = 70;
        }
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
    public void end(boolean interrupted) {}

    @Override 
    public boolean isFinished() { //false: runs continuously, true: runs once and stops
        if(m_climb.getPosition() >= encoderPosition) {
            return true; 
        }  
        return false; 
    }
    
}

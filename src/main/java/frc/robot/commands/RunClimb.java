package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class RunClimb extends Command {
    private final ClimbSubsystem m_climb;  
    private double speed; 
    private double encoderPosition; 
    private ClimbHeight height; 
    
    public enum ClimbHeight {
        LOW, MID, HIGH, MAX, NULL; 
    }

    //constructor to use for direct control of climb (might remove)
    public RunClimb(ClimbSubsystem climbSubsystem, double speed) {
        this.height = ClimbHeight.NULL; 
        m_climb = climbSubsystem;
        this.speed = speed;
        addRequirements(m_climb);
    }

    //default full speed constructor
    public RunClimb(ClimbSubsystem climbSubsystem, ClimbHeight height) { 
        this.height = height; 
        //all temporary encoder values (test and change)
        switch(height) {
            case LOW:
                encoderPosition = 68;
            case MID:
                encoderPosition = 69;
            case HIGH:
                encoderPosition = 70;
            case MAX: 
                encoderPosition = 71; 
            case NULL: 
                encoderPosition = 0; 
        }
        m_climb = climbSubsystem; 
        speed = 1; 
        addRequirements(m_climb);
    }

    //variable speed constructor 
    public RunClimb(ClimbSubsystem climbSubsystem, ClimbHeight height, double speed) { 
        this.height = height; 
        //all temporary encoder values (test and change)
        switch(height) {
            case LOW:
                encoderPosition = 68;
            case MID:
                encoderPosition = 69;
            case HIGH:
                encoderPosition = 70;
            case MAX: 
                encoderPosition = 71; 
            case NULL: 
                encoderPosition = 0; 
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
    public void end(boolean interrupted) {
        m_climb.setMotor(0);
    }

    @Override 
    public boolean isFinished() { 
        if(height == ClimbHeight.NULL) { //checks to see if using direct input 
            return false; 
        } else if (m_climb.getPosition() >= encoderPosition) {
            return true; 
        } else {
            return false; 
        }
    }
    
}

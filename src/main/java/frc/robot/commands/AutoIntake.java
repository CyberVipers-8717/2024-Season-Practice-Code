package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntake extends Command {
    private final IntakeSubsystem m_intake; 
    private final double speed; 
    private final Timer waitTimer = new Timer(); 

    public AutoIntake(IntakeSubsystem intakeSubsystem, double speed) { //allows variable speeds 
        m_intake = intakeSubsystem; 
        this.speed = speed; 
        addRequirements(m_intake);
    }

    @Override 
    public void initialize() {
        waitTimer.restart();
        waitTimer.start(); 
        m_intake.zeroEncoders();
    }

    @Override 
    public void execute() {
        m_intake.setMotors(speed);
    }

    @Override 
    public void end(boolean interrupted) {
        m_intake.setMotors(0);
        waitTimer.stop();  
    }

    @Override 
    public boolean isFinished() { //false: runs continuously, true: runs once and stops
        if (waitTimer.get() < 1) { //change time limit 
            return false; 
        } else {
            return true; 
        }
    }
    
}

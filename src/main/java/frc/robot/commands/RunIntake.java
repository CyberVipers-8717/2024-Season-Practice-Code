package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem m_intake; 
    private final double speed; 

    public RunIntake(IntakeSubsystem intakeSubsystem) { //defaults to full speed
        m_intake = intakeSubsystem; 
        speed = 1; 
        addRequirements(m_intake);
    }

    public RunIntake(IntakeSubsystem intakeSubsystem, double speed) { //allows variable speeds 
        m_intake = intakeSubsystem; 
        this.speed = speed; 
        addRequirements(m_intake);
    }

    @Override 
    public void initialize() {
        m_intake.zeroEncoders();
    }

    @Override 
    public void execute() {
        m_intake.setMotors(speed);
    }

    @Override 
    public void end(boolean interrupted) {}

    @Override 
    public boolean isFinished() { //runs once and finishes
        return true;
    }
    
}

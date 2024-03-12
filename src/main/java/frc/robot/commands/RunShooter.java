package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooter extends Command {
    private final ShooterSubsystem m_shooter;
    private final double speed;

    public RunShooter(ShooterSubsystem shooterSubsystem) {
        m_shooter = shooterSubsystem; 
        speed = 1; 
        addRequirements(m_shooter);
    }

    public RunShooter(ShooterSubsystem shooterSubsystem, double speed){
        m_shooter = shooterSubsystem;
        this.speed = speed;
        addRequirements(m_shooter);
    }

    @Override 
    public void initialize() {
        m_shooter.zeroEncoders();
    }

    @Override 
    public void execute() {
        m_shooter.setMotors(speed);
    }

    @Override 
    public void end(boolean interrupted) {
        m_shooter.setMotors(0);
    }

    @Override 
    public boolean isFinished() { 
        return false; 
    }
}
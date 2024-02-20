package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.UptakeSubsystem;

public class RunUptake extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final UptakeSubsystem m_uptake; 
    private final double speed; 

    public RunUptake(UptakeSubsystem uptakeSubsystem) {//defaults to full speed
        m_uptake = uptakeSubsystem;
        speed = 1;  
        addRequirements(m_uptake);
    }

    public RunUptake(UptakeSubsystem uptakeSubsystem, double speed) { //allows variable speeds 
        m_uptake = uptakeSubsystem; 
        this.speed = speed; 
        addRequirements(m_uptake);
    }

    @Override 
    public void initialize() {
        m_uptake.zeroEncoder();
    }

    @Override 
    public void execute() {
        m_uptake.setMotor(speed);
    }

    @Override 
    public void end(boolean interrupted) {
    }

    @Override 
    public boolean isFinished() { //checks if voltage spikes and ends command
       if (m_uptake.getMotorVoltage() > 15) {
        return true; 
       } else {
        return false; 
       }
    }
    
}

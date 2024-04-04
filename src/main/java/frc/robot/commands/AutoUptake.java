package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.UptakeSubsystem;

public class AutoUptake extends Command {
    private final UptakeSubsystem m_uptake; 
    private final double speed; 
    private final Timer waitTimer = new Timer(); 

    public AutoUptake(UptakeSubsystem uptakeSubsystem, double speed) { //allows variable speeds 
        m_uptake = uptakeSubsystem; 
        this.speed = speed; 
        addRequirements(m_uptake);
    }

    @Override 
    public void initialize() {
        waitTimer.reset();
        waitTimer.start();
        m_uptake.zeroEncoder();
    }

    @Override 
    public void execute() {
        m_uptake.setMotor(speed);
    }

    @Override 
    public void end(boolean interrupted) {
        m_uptake.setMotor(0);
        waitTimer.stop(); 

    }

    @Override 
    public boolean isFinished() {
        if(waitTimer.get() < .5) {
            return false; 
        } else {
            return true;
        }
    }
    
}

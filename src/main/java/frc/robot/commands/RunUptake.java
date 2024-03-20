package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.UptakeSubsystem;

public class RunUptake extends Command {
    private final UptakeSubsystem m_uptake; 
    private final double speed; 
    private final Timer waitTimer = new Timer(); 
    private final Timer uptakeTimer = new Timer(); 

    //defaults to full speed
    public RunUptake(UptakeSubsystem uptakeSubsystem) {
        m_uptake = uptakeSubsystem;
        speed = 1;  
        addRequirements(m_uptake);
    }

    //allows variable speeds 
    public RunUptake(UptakeSubsystem uptakeSubsystem, double speed) { 
        m_uptake = uptakeSubsystem; 
        this.speed = speed; 
        addRequirements(m_uptake);
    }

    @Override 
    public void initialize() {
        waitTimer.start();
        m_uptake.zeroEncoder();
    }

    @Override 
    public void execute() {
        System.out.println(m_uptake.getAmps());
        m_uptake.setMotor(speed);
    }

    @Override 
    public void end(boolean interrupted) {
        uptakeTimer.reset();  
        waitTimer.reset();
        m_uptake.setMotor(0);

    }

    @Override 
    public boolean isFinished() { //checks if current spikes and ends command only after a couple second delay
        if(speed < 0 || speed == .26) { //rough work around to differentiate between flush mode and popping   
            return false; 
        } else if (waitTimer.get() >= .5) { //waits half a second after initializing to ignore current spikes on start up (needs tuning)
            if (m_uptake.getAmps() > 5 && uptakeTimer.get() == 0) { //current threshold might cahnge
                uptakeTimer.start();
                return false;
            } else if (m_uptake.getAmps() > 5 && uptakeTimer.get() >= .1) {
                return true;
            }
       }
       return false; 
    }
    
}

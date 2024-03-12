package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax m_lowShooter; 
    private final CANSparkMax m_highShooter; 

    private final RelativeEncoder m_lowEncoder; 
    private final RelativeEncoder m_highEncoder; 

    public ShooterSubsystem() { //add some PID stuff maybe??
        m_lowShooter = new CANSparkMax(ShooterConstants.kLowShooterMotorPort, MotorType.kBrushless); 
        m_highShooter = new CANSparkMax(ShooterConstants.kHighShooterMotorPort, MotorType.kBrushless); 

        m_lowEncoder = m_lowShooter.getEncoder(); 
        m_highEncoder = m_highShooter.getEncoder();

        m_lowShooter.setIdleMode(IdleMode.kBrake); 
        m_highShooter.setIdleMode(IdleMode.kBrake);

        m_lowShooter.setSmartCurrentLimit(ShooterConstants.kShooterMotorCurrentLimit);
        m_highShooter.setSmartCurrentLimit(ShooterConstants.kShooterMotorCurrentLimit); 

        m_lowShooter.setInverted(true);
        m_highShooter.setInverted(true);

        m_lowShooter.burnFlash();
        m_highShooter.burnFlash(); 
    }

    public void zeroEncoders() {
        m_lowEncoder.setPosition(0);
        m_highEncoder.setPosition(0);
    }

    public void setMotors(double speed) {
        m_lowShooter.set(speed);
        m_highShooter.set(speed);
    }

}

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem {
    private final CANSparkMax m_lowShooter; 
    private final CANSparkMax m_highShooter; 

    private final RelativeEncoder m_lowEncoder; 
    private final RelativeEncoder m_highEncoder; 

    private final SparkPIDController m_lowPID; 
    private final SparkPIDController m_highPID; 

    public ShooterSubsystem() { //finish shooter pid and stuff 
        m_lowShooter = new CANSparkMax(ShooterConstants.kLowShooterMotorPort, MotorType.kBrushless); 
        m_highShooter = new CANSparkMax(ShooterConstants.kHighShooterMotorPort, MotorType.kBrushless); 

        m_lowEncoder = m_lowShooter.getEncoder(); 
        m_highEncoder = m_highShooter.getEncoder();

        m_lowPID = m_lowShooter.getPIDController(); 
        m_highPID = m_highShooter.getPIDController();

        m_lowPID.setFeedbackDevice(m_lowEncoder); 
        m_highPID.setFeedbackDevice(m_highEncoder);
        
        //m_lowEncoder.setPositionConversionFactor();

        m_lowShooter.setIdleMode(IdleMode.kBrake); 
        m_highShooter.setIdleMode(IdleMode.kBrake);

        m_lowShooter.setSmartCurrentLimit(ShooterConstants.kShooterMotorCurrentLimit);
        m_highShooter.setSmartCurrentLimit(ShooterConstants.kShooterMotorCurrentLimit); 

        m_lowShooter.burnFlash();
        m_highShooter.burnFlash(); 
    }

    public void zeroEncoders() {
        m_lowEncoder.setPosition(0);
        m_highEncoder.setPosition(0);
    }



}

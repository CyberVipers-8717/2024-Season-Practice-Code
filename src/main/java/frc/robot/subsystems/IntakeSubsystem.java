package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax m_lowIntake;
    private final CANSparkMax m_highIntake;

    private final RelativeEncoder m_lowEncoder;
    private final RelativeEncoder m_highEncoder; 

    public IntakeSubsystem() { //might want to add motor id parameter 
        //temporary motor ids (change later)
        m_lowIntake = new CANSparkMax(IntakeConstants.kLowIntakeMotorPort, MotorType.kBrushless);
        m_highIntake = new CANSparkMax(IntakeConstants.kHighIntakeMotorPort, MotorType.kBrushless);

        m_lowEncoder = m_lowIntake.getEncoder(); 
        m_highEncoder = m_highIntake.getEncoder(); 

        //have to test
        m_highIntake.setInverted(IntakeConstants.kHighIntakeInverted);

        m_lowIntake.setIdleMode(IdleMode.kBrake);
        m_highIntake.setIdleMode(IdleMode.kBrake);

        m_lowIntake.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
        m_highIntake.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
        
        m_lowIntake.burnFlash();
        m_highIntake.burnFlash();

        zeroEncoders();
    }

    public void zeroEncoders() {
        m_lowEncoder.setPosition(0);
        m_highEncoder.setPosition(0);
    }

    public void setMotors(double speed) {
        m_lowIntake.set(speed);
        m_highIntake.set(speed);
    }

}

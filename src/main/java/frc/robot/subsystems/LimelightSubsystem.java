package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable table; 

    public LimelightSubsystem(String limelightName) {
        table = NetworkTableInstance.getDefault().getTable(limelightName);
    }

    public double getTX() {
        return table.getEntry("tx").getDouble(0.0);
    }

    public double getTY() {
        return table.getEntry("ty").getDouble(0.0);
    }
    
    public double getTV() {
        return table.getEntry("tv").getDouble(0.0); // returns 0 if an april tag is not detected, returns 1 if an april tag is not detected
    }

    public double[] get3dTargetPoseArray() {
        return table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); // we need to find out which entry in this array is the yaw value
    }

    public double getPipeline() {
        return table.getEntry("getpipe").getDouble(0.0);
    }

    public void setPipeline(double pipelineIndex) {
        table.getEntry("pipeline").setDouble(pipelineIndex);
    }

    public double getLedMode() {
        return table.getEntry("ledMode").getDouble(getLedMode());
    }

    public void setLedMode(double ledIndex) {
        table.getEntry("ledMode").setDouble(ledIndex);
    }

    public double[] getAprilTagID() {
        return table.getEntry("tid").getDoubleArray(new double[6]);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}

package frc.robot.modules.gyro;

/**
 * 
 */
abstract public class GyroModule {
    private double[] acceleration = { 0, 0 };
    private boolean is_connected = false;
    private double pitch_degrees = 0.0;
    private double position_degrees = 0.0;
    private double velocity_degrees_per_second = 0.0;

    /**
     * 
     */
    abstract public void update();

    /**
     * 
     */
    abstract public void zeroYaw();

    /**
     * Getters and Setters
     */
    public double[] getAcceleration() { return this.acceleration; }
    public boolean isConnected() { return this.is_connected; }
    public double getPitchDeg() { return this.pitch_degrees; }
    public double getPositionDeg() { return this.position_degrees; }
    public double getVelocityDegPerSec() { return this.velocity_degrees_per_second; }

    public void setAccleration(double[] acceleration) { this.acceleration = acceleration; }
    public void setIsConnected(boolean is_connected) { this.is_connected = is_connected; }
    public void setPitchDeg(double pitch_degrees) { this.pitch_degrees = pitch_degrees; }
    public void setPositionDeg(double position_degrees) { this.position_degrees = position_degrees; }
    public void setVelocityDegPerSec(double velocity_degrees_per_second) { this.velocity_degrees_per_second = velocity_degrees_per_second; }
}

package frc.robot.modules.gyro;

abstract public class GyroModule {
    private boolean is_connected = false;
    private double position_degrees = 0.0;
    private double velocity_degrees_per_second = 0.0;

    /**
     * 
     */
    abstract public void update();

    /**
     * Getters and Setters
     */
    public boolean isConnected() { return this.is_connected; }
    public double getPositionDeg() { return this.position_degrees; }
    public double getVelocityDegPerSec() { return this.velocity_degrees_per_second; }

    public void setIsConnected(boolean is_connected) { this.is_connected = is_connected; }
    public void setPositionDeg(double position_degrees) { this.position_degrees = position_degrees; }
    public void setVelocityDegPerSec(double velocity_degrees_per_second) { this.velocity_degrees_per_second = velocity_degrees_per_second; }
}

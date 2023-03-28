package frc.robot.modules.arm;

/**
 * 
 */
abstract public class ArmModule {
    /* Mechanisim2d Display for Monitoring the Arm Position */
    protected final ArmModuleMechanism armModuleMechanism = new ArmModuleMechanism();

    private boolean enabled;
    private double desired_angle_degrees;

    /**
     * 
     */
    public ArmModule() {
        this.enabled = false;
    }

    /**
     * 
     */
    abstract public boolean atAngle();
    abstract public boolean atAngle(double angle);
    abstract public void close();
    abstract public boolean isInnerLimitSwitchTriggered();
    abstract public boolean isOuterLimitSwitchTriggered();
    abstract public void open();
    abstract public void resetController();
    abstract public void stop();
    abstract public void update();
    
    /**
     * Getters and Setters
     */
    public double getDesiredAngle() {
        return this.desired_angle_degrees;
    }

    public boolean isEnabled() {
        return this.enabled;
    }

    public void setDesiredAngle(double desired_angle_degrees) {
        this.desired_angle_degrees = desired_angle_degrees;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
}

package frc.robot.modules.intake;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class IntakeModule {
    /* Hardware Contants */
    private final int MOTOR_CHANNEL = 0;
    private final double MOTOR_SPEED = -0.6;

    /* Intake Hardware */
    private final Talon motor;

    /**
     * 
     */
    public IntakeModule() {
        this.motor = new Talon(MOTOR_CHANNEL);
    }

    /**
     * 
     */
    public void eject() {
        this.motor.set(-MOTOR_SPEED);
    }

    /**
     * 
     */
    public void intake() {
        this.motor.set(MOTOR_SPEED);
    }

    /**
     * 
     */
    public void stop() {
        this.motor.set(0.0);
    }

    /**
     * 
     */
    public void update() {
    }
}

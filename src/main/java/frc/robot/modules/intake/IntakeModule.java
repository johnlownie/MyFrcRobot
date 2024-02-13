package frc.robot.modules.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class IntakeModule {
    /* Hardware Contants */
    private final int MOTOR_CHANNEL = 0;
    private final double MOTOR_SPEED = -0.6;
    private final int INTAKE_LIMIT_SWITCH_ID = 3;

    /* Intake Hardware */
    private final Talon motor;
    private final DigitalInput intakeLimitSwitch;

    /**
     * 
     */
    public IntakeModule() {
        this.motor = new Talon(MOTOR_CHANNEL);
        this.intakeLimitSwitch = new DigitalInput(INTAKE_LIMIT_SWITCH_ID);
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
    public boolean hasNote() {
        return !this.intakeLimitSwitch.get();
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
        Logger.recordOutput("Mechanisms/Intake/Has Note", hasNote());
    }
}

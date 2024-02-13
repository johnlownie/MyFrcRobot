package frc.robot.modules.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterModule {
    /* Hardware Contants */
    private final int LEFT_MOTOR_ID = 62;
    private final int RIGHT_MOTOR_ID = 61;
    private final int KICKER_MOTOR_ID = 60;
    private final int LINE_BREAK_CHANNEL_ID = 8;
    private final double AMP_SPEED = 0.9;
    private final double SPEAKER_SPEED = 0.9;
    private final double KICKER_SPEED = 0.6;
    private final int AMP_VELOCITY = 2000;
    private final int SPEAKER_VELOCITY = 5000;

    /* Shooter Hardware */
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final CANSparkMax kickerMotor;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
    private DigitalInput linebreak;

    /**
     * 
     */
    public ShooterModule() {
        this.leftMotor = new CANSparkMax(LEFT_MOTOR_ID, MotorType.kBrushless);
        this.leftMotor.restoreFactoryDefaults();
        this.leftMotor.setInverted(true);
        
        this.rightMotor = new CANSparkMax(RIGHT_MOTOR_ID, MotorType.kBrushless);
        this.rightMotor.restoreFactoryDefaults();

        this.kickerMotor = new CANSparkMax(KICKER_MOTOR_ID, MotorType.kBrushless);
        this.kickerMotor.setInverted(true);

        this.leftEncoder = this.leftMotor.getEncoder();
        this.rightEncoder = this.rightMotor.getEncoder();

        this.linebreak = new DigitalInput(LINE_BREAK_CHANNEL_ID);
    }

    /**
     * 
     */
    public boolean hasNoteToShoot() {
        return !this.linebreak.get();
    }

    /**
     * 
     */
    public boolean hasShot() {
        return !this.linebreak.get();
    }

    /**
     * 
     */
    public boolean isSpunupForAmp() {
        return this.leftEncoder.getVelocity() >= AMP_VELOCITY && this.rightEncoder.getVelocity() >= AMP_VELOCITY;
    }

    /**
     * 
     */
    public boolean isSpunupForSpeaker() {
        return this.leftEncoder.getVelocity() >= SPEAKER_VELOCITY && this.rightEncoder.getVelocity() >= SPEAKER_VELOCITY;
    }

    /**
     * 
     */
    public void shoot() {
        this.kickerMotor.set(KICKER_SPEED);
    }

    /**
     * 
     */
    public void spinupForAmp() {
        this.leftMotor.set(AMP_SPEED);
        this.rightMotor.set(AMP_SPEED);
    }

    /**
     * 
     */
    public void spinupForSpeaker() {
        this.leftMotor.set(SPEAKER_SPEED);
        this.rightMotor.set(SPEAKER_SPEED);
    }

    /**
     * 
     */
    public void stop() {
        this.leftMotor.set(0.0);
        this.rightMotor.set(0.0);
        this.kickerMotor.set(0.0);
    }

    /**
     * 
     */
    public void update() {
        Logger.recordOutput("Mechanisms/Shooter/Has Note", hasNoteToShoot());
        Logger.recordOutput("Mechanisms/Shooter/Left Velocity", this.leftEncoder.getVelocity());
        Logger.recordOutput("Mechanisms/Shooter/Right Velocity", this.rightEncoder.getVelocity());
        Logger.recordOutput("Mechanisms/Shooter/Kicker Output Current", this.kickerMotor.getOutputCurrent());
    }
}

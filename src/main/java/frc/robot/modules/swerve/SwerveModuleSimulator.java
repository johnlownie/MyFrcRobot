package frc.robot.modules.swerve;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.util.Timer;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.utils.Conversions;
import frc.robot.utils.LoggedTunableNumber;

/**
 * 
 */
public class SwerveModuleSimulator extends SwerveModule {
    /* Simulated Drive Motor PID Values */
    private final double DRIVE_KP = 0.8;
    private final double DRIVE_KI = 0.0;
    private final double DRIVE_KD = 0.0;

    /* Simulated Turn Motor PID Values */
    private final double ANGLE_KP = 12.0;
    private final double ANGLE_KI = 0.0;
    private final double ANGLE_KD = 0.0;

    /* Simulated Drive Motor Characterization Values */
    private final double DRIVE_KS = 0.0545;  // 0.116970;
    private final double DRIVE_KV = 0.40126 / 12.0; // 0.133240;
    private final double DRIVE_KA = 0.0225;  // 0.0;

    /* Tunable PID */
    private final LoggedTunableNumber driveKp = new LoggedTunableNumber("Drive/DriveKp", DRIVE_KP);
    private final LoggedTunableNumber driveKi = new LoggedTunableNumber("Drive/DriveKi", DRIVE_KI);
    private final LoggedTunableNumber driveKd = new LoggedTunableNumber("Drive/DriveKd", DRIVE_KD);

    private final LoggedTunableNumber angleKp = new LoggedTunableNumber("Drive/TurnKp", ANGLE_KP);
    private final LoggedTunableNumber angleKi = new LoggedTunableNumber("Drive/TurnKi", ANGLE_KI);
    private final LoggedTunableNumber angleKd = new LoggedTunableNumber("Drive/TurnKd", ANGLE_KD);

    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);

    /* Simulated Motors */
    private FlywheelSim driveMotorSim;
    private FlywheelSim angleMotorSim;

    private PIDController driveController;
    private PIDController angleController;
    
    private TalonFXSimState driveSimState;
    private TalonFXSimState angleSimState;

    private final CANcoderSimState canCoderSimState;

    /**
     * 
     */
    public SwerveModuleSimulator(int module_id, int drive_motor_id, int angle_motor_id, int can_coder_id, Rotation2d angle_offset_degrees) {
        super(module_id, drive_motor_id, angle_motor_id, can_coder_id, angle_offset_degrees);

        this.driveMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), CHOSEN_MODULE.driveGearRatio, 0.025);
        this.angleMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), CHOSEN_MODULE.angleGearRatio, 0.004096955);

        this.driveController = new PIDController(driveKp.get(), driveKi.get(), driveKd.get());
        this.angleController = new PIDController(angleKp.get(), angleKi.get(), angleKd.get());

        this.driveSimState = this.driveMotor.getSimState();
        this.angleSimState = this.angleMotor.getSimState();
        this.canCoderSimState = this.encoder.getSimState();
    }

    /**
     * 
     */
    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.RPSToMPS(this.driveMotorSim.getAngularVelocityRPM(), CHOSEN_MODULE.wheelCircumference), 
            Rotation2d.fromRotations(this.angleMotorSim.getAngularVelocityRadPerSec())
        );
    }

    /**
     * 
     */
    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.RADToMPS(this.driveMotorSim.getAngularVelocityRadPerSec(), CHOSEN_MODULE.wheelCircumference), 
            Rotation2d.fromRotations(this.angleMotorSim.getAngularVelocityRadPerSec())
        );
    }

    /**
     * 
     */
    @Override
    protected void setAngleVoltage(PositionVoltage positionVoltage) {
        super.setAngleVoltage(positionVoltage);

        this.angleMotorSim.setInputVoltage(positionVoltage.Position);
    }

    /**
     * 
     */
    @Override
    protected void setDriveVelocity(VelocityVoltage velocityVoltage) {
        super.setDriveVelocity(velocityVoltage);

        double voltage = MathUtil.clamp(velocityVoltage.FeedForward, -12.0, 12.0);
        this.driveMotorSim.setInputVoltage(voltage);
    }

    /**
     * 
     */
    @Override
    protected void setDriveVoltage(DutyCycleOut dutyCycleOut) {
        super.setDriveVoltage(dutyCycleOut);
        
        double voltage = MathUtil.clamp(dutyCycleOut.Output * 12.0, -12.0, 12.0);
        this.driveMotorSim.setInputVoltage(voltage);
    }
    
    /**
     * 
     */
    @Override
    public void updatePositions() {
        // update the simulated motors
        this.angleMotorSim.update(RobotConstants.LOOP_PERIOD_SECS);
        this.driveMotorSim.update(RobotConstants.LOOP_PERIOD_SECS);

        if (RobotConstants.TUNING_MODE) {
            if (driveKp.hasChanged(hashCode()) || driveKi.hasChanged(hashCode()) || driveKd.hasChanged(hashCode())) {
                Slot0Configs slot0Configs = new Slot0Configs();
                slot0Configs.kP = this.driveKp.get();
                slot0Configs.kI = this.driveKi.get();
                slot0Configs.kD = this.driveKd.get();
                this.driveMotor.getConfigurator().refresh(slot0Configs);
            }
            
            if (angleKp.hasChanged(hashCode()) || angleKi.hasChanged(hashCode()) || angleKd.hasChanged(hashCode())) {
                Slot0Configs slot0Configs = new Slot0Configs();
                slot0Configs.kP = this.angleKp.get();
                slot0Configs.kI = this.angleKi.get();
                slot0Configs.kD = this.angleKd.get();
                this.angleMotor.getConfigurator().refresh(slot0Configs);
            }
        }
    }
}
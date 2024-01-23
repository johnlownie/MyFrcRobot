package frc.robot.modules.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
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

    /* Variables */
    private double anglePositionDeg = 0.0;
    private double angleRelativePositionRadians = 0.0;
    private double angleSetpointDegrees = 0.0;
    
    private double driveDistanceMeters = 0.0;
    private double driveSetpointPercentage = 0.0;
    private double driveSetpointMPS = 0.0;
    private double driveVelocityMetersPerSecond = 0.0;
    
    private boolean isOpenLoop = false;

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
        return new SwerveModulePosition(this.driveDistanceMeters, Rotation2d.fromDegrees(this.anglePositionDeg));
    }

    /**
     * 
     */
    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(this.driveVelocityMetersPerSecond, Rotation2d.fromDegrees(this.anglePositionDeg));
    }

    /**
     * 
     */
    @Override
    protected void setAngleState(SwerveModuleState desiredState) {
        super.setAngleState(desiredState);

        this.angleSetpointDegrees = desiredState.angle.getDegrees();
    }

    /**
     * 
     */
    protected void setDriveState(SwerveModuleState desiredState, boolean isOpenLoop) {
        super.setDriveState(desiredState, isOpenLoop);
        this.isOpenLoop = isOpenLoop;

        if (isOpenLoop) {
            this.driveSetpointPercentage = desiredState.speedMetersPerSecond / SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND;
        }
        else {
            this.driveSetpointMPS = this.feedForward.calculate(desiredState.speedMetersPerSecond);
        }
    }

    /**
     * 
     */
    private void applyAngleSettings() {
        double pidOutput = this.angleController.calculate(this.angleRelativePositionRadians, Math.toRadians(this.angleSetpointDegrees));

        double voltage = MathUtil.clamp(pidOutput, -12.0, 12.0);
        this.angleMotorSim.setInputVoltage(voltage);
    }

    /**
     * 
     */
    private void applyDriveSettings() {
        if (this.isOpenLoop) {
            this.driveController.reset();
            double voltage = MathUtil.clamp(this.driveSetpointPercentage * 12.0, -12.0, 12.0);
            this.driveMotorSim.setInputVoltage(voltage);
            }
        else {
            double velocityRadiansPerSecond = this.driveSetpointMPS * (2.0 * Math.PI) / (CHOSEN_MODULE.wheelCircumference);
            double ffOutput = this.feedForward.calculate(velocityRadiansPerSecond); 
            double pidOutput =  this.driveController.calculate(this.driveVelocityMetersPerSecond, velocityRadiansPerSecond);
            
            double voltage = MathUtil.clamp(ffOutput + pidOutput, -12.0, 12.0);
            this.driveMotorSim.setInputVoltage(voltage);
        }
    }
    
    /**
     * 
     */
    @Override
    public void updatePositions() {
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

        // update the simulated motors
        this.angleMotorSim.update(RobotConstants.LOOP_PERIOD_SECS);
        this.driveMotorSim.update(RobotConstants.LOOP_PERIOD_SECS);

        double angleDiffRadians = this.angleMotorSim.getAngularVelocityRadPerSec() * RobotConstants.LOOP_PERIOD_SECS;
        this.angleRelativePositionRadians += angleDiffRadians;
        this.anglePositionDeg = this.angleRelativePositionRadians * (180.0 / Math.PI);

        this.driveVelocityMetersPerSecond = Conversions.RADToMPS(this.driveMotorSim.getAngularVelocityRadPerSec(), CHOSEN_MODULE.wheelCircumference);
        this.driveDistanceMeters = this.driveDistanceMeters + (this.driveMotorSim.getAngularVelocityRadPerSec() * RobotConstants.LOOP_PERIOD_SECS * (CHOSEN_MODULE.wheelCircumference / (2.0 * Math.PI)));

        applyAngleSettings();
        applyDriveSettings();
    }
}
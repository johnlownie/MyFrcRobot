package frc.robot.modules.swerve;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.utils.Conversions;

/**
 * 
 */
public class SwerveModuleSimulator extends SwerveModule {
    /* Simulated Drive Motor Characterization Values */
    private final double DRIVE_KS = 0.32; //0.0545;  // 0.116970;
    private final double DRIVE_KV = 1.51; //0.40126 / 12.0; // 0.133240;
    private final double DRIVE_KA = 0.27; //0.0225;  // 0.0;

    private final SimpleMotorFeedforward feedForward;

    /* Simulated Motors */
    private FlywheelSim driveMotorSim;
    private FlywheelSim turnMotorSim;

    private PIDController driveController;
    private PIDController turnController;
    
    private TalonFXSimState driveSimState;
    private TalonFXSimState angleSimState;

    private final CANcoderSimState canCoderSimState;
    
    private boolean isOpenLoop = false;

    /**
     * 
     */
    public SwerveModuleSimulator(int module_id, int drive_motor_id, int angle_motor_id, int can_coder_id, Rotation2d angle_offset_degrees) {
        super(module_id, drive_motor_id, angle_motor_id, can_coder_id, angle_offset_degrees);

        this.driveMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), CHOSEN_MODULE.driveGearRatio, 0.025);
        this.turnMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), CHOSEN_MODULE.angleGearRatio, 0.004096955);

        this.driveController = new PIDController(PIDConstants.SWERVE_MODULE_DRIVE_KP, PIDConstants.SWERVE_MODULE_DRIVE_KI, PIDConstants.SWERVE_MODULE_DRIVE_KD);
        this.turnController = new PIDController(PIDConstants.SWERVE_MODULE_TURN_KP, PIDConstants.SWERVE_MODULE_TURN_KI, PIDConstants.SWERVE_MODULE_TURN_KD);

        this.driveSimState = this.driveMotor.getSimState();
        this.angleSimState = this.angleMotor.getSimState();
        this.canCoderSimState = this.encoder.getSimState();

        this.feedForward = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);
    }

    /**
     * 
     */
    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.driveDistanceMeters, Rotation2d.fromDegrees(this.anglePositionDEG));
    }

    /**
     * 
     */
    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(this.driveVelocityMPS, Rotation2d.fromDegrees(this.anglePositionDEG));
    }

    /**
     * 
     */
    @Override
    protected void setAngleState(SwerveModuleState desiredState) {
        super.setAngleState(desiredState);

        this.angleSetpointDEG = desiredState.angle.getDegrees();
    }

    /**
     * 
     */
    protected void setDriveState(SwerveModuleState desiredState, boolean isOpenLoop) {
        super.setDriveState(desiredState, isOpenLoop);

        this.isOpenLoop = isOpenLoop;

        if (isOpenLoop) {
            this.driveSetpointPCT = desiredState.speedMetersPerSecond / SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND;
        }
        else {
            this.driveSetpointMPS = this.feedForward.calculate(desiredState.speedMetersPerSecond);
        }
    }

    /**
     * 
     */
    private void applyAngleSettings() {
        double pidOutput = this.turnController.calculate(this.angleRelativePositionRAD, Math.toRadians(this.angleSetpointDEG));

        double voltage = MathUtil.clamp(pidOutput, -12.0, 12.0);
        this.turnMotorSim.setInputVoltage(voltage);
    }

    /**
     * 
     */
    private void applyDriveSettings() {
        if (this.isOpenLoop) {
            this.driveController.reset();
            double voltage = MathUtil.clamp(this.driveSetpointPCT * 12.0, -12.0, 12.0);
            this.driveMotorSim.setInputVoltage(voltage);
            }
        else {
            double radiansPerSecond = Conversions.MPSToRPS(this.driveSetpointMPS, CHOSEN_MODULE.wheelCircumference);
            double pidOutput = this.driveController.calculate(this.driveVelocityMPS, radiansPerSecond);
            
            double voltage = MathUtil.clamp(radiansPerSecond + pidOutput, -12.0, 12.0);
            this.driveMotorSim.setInputVoltage(voltage);
        }
    }

    @Override
    public void updateDrivePID(double kP, double kI, double kD) {
        if (RobotConstants.TUNING_MODE) {
            this.driveController.setPID(kP, kI, kD);
        }
    }

    @Override
    public void updateTurnPID(double kP, double kI, double kD) {
        if (RobotConstants.TUNING_MODE) {
            this.turnController.setPID(kP, kI, kD);
        }
    }
    
    @Override
    public void updatePositions() {
        // update the simulated motors
        double timeBetweenUpdates = Timer.getFPGATimestamp() - this.drivePreviousTimestamp;

        this.turnMotorSim.update(timeBetweenUpdates);
        this.driveMotorSim.update(timeBetweenUpdates);

        double angleDiffRadians = this.turnMotorSim.getAngularVelocityRadPerSec() * RobotConstants.LOOP_PERIOD_SECS;
        this.angleRelativePositionRAD += angleDiffRadians;
        this.anglePositionDEG = this.angleRelativePositionRAD * (180.0 / Math.PI);
        this.angleVelocityRPM = Conversions.toRPM(angleDiffRadians, CHOSEN_MODULE.angleGearRatio);

        this.driveVelocityMPS = Conversions.RADToMPS(this.driveMotorSim.getAngularVelocityRadPerSec(), CHOSEN_MODULE.wheelCircumference);

        this.driveDistanceMeters = this.driveDistanceMeters + (this.driveVelocityMPS * timeBetweenUpdates);
        this.driveAccelerationMPS = (this.driveVelocityMPS - this.drivePreviousVelocityMPS) / timeBetweenUpdates;
        this.drivePreviousVelocityMPS = this.driveVelocityMPS;
        this.drivePreviousTimestamp = Timer.getFPGATimestamp();
    
        applyAngleSettings();
        applyDriveSettings();

        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Turn/AbsolutePositionDEG", this.angleAbsolutePositionDEG);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Turn/PositionDEG", this.anglePositionDEG);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Turn/VelocityRPM", this.angleVelocityRPM);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/AccelerationMPS", this.driveAccelerationMPS);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/DistanceMeters", this.driveDistanceMeters);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/PositionDEG", this.drivePositionDEG);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/PositionDEG", this.drivePositionDEG);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/SetPointMPS", this.driveSetpointMPS);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/SetPointPCT", this.driveSetpointPCT);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/VelocityMPS", this.driveVelocityMPS);
        
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/AngularVelocityRPM", this.driveMotorSim.getAngularVelocityRPM());
    }
}

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
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.util.Timer;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.utils.Conversions;

/**
 * 
 */
public class SwerveModuleSimulator extends SwerveModuleTalonFX {
    /* Simulated Drive Motor Characterization Values */
    private final double DRIVE_KS = 0.0; //0.0545;  // 0.116970;
    private final double DRIVE_KV = 0.1205; //0.40126 / 12.0; // 0.133240;
    private final double DRIVE_KA = 0.0; //0.0225;  // 0.0;

    private final SimpleMotorFeedforward feedForward;

    /* Simulated Motors */
    private DCMotorSim driveMotorSim;
    private DCMotorSim turnMotorSim;

    private PIDController driveController;
    private PIDController turnController;
    
    private TalonFXSimState driveSimState;
    private TalonFXSimState angleSimState;

    private final CANcoderSimState canCoderSimState;
    
    /* Variables */
    private final Rotation2d turnAbsoluteInitialPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);

    /**
     * 
     */
    public SwerveModuleSimulator(int module_id, int drive_motor_id, int angle_motor_id, int can_coder_id, Rotation2d angle_offset_degrees) {
        super(module_id, drive_motor_id, angle_motor_id, can_coder_id, angle_offset_degrees);

        this.driveMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), CHOSEN_MODULE.driveGearRatio, 0.025);
        this.turnMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), CHOSEN_MODULE.angleGearRatio, 0.004);

        this.driveController = new PIDController(PIDConstants.SIM_SWERVE_MODULE_DRIVE_KP, PIDConstants.SIM_SWERVE_MODULE_DRIVE_KI, PIDConstants.SIM_SWERVE_MODULE_DRIVE_KD);
        this.turnController = new PIDController(PIDConstants.SIM_SWERVE_MODULE_TURN_KP, PIDConstants.SIM_SWERVE_MODULE_TURN_KI, PIDConstants.SIM_SWERVE_MODULE_TURN_KD);

        this.driveSimState = this.driveMotor.getSimState();
        this.angleSimState = this.turnMotor.getSimState();
        this.canCoderSimState = this.cancoder.getSimState();

        this.feedForward = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);
    }

    /**
     * 
     */
    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.driveDistanceMeters, Rotation2d.fromDegrees(this.turnPositionDEG));
    }

    /**
     * 
     */
    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(this.driveVelocityMPS, Rotation2d.fromDegrees(this.turnPositionDEG));
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
            // this.driveSetpointMPS = this.feedForward.calculate(desiredState.speedMetersPerSecond);
            this.driveSetpointMPS = desiredState.speedMetersPerSecond * Math.cos(this.turnController.getPositionError());

        }
    }

    /**
     * 
     */
    @Override
    protected void setTurnState(SwerveModuleState desiredState) {
        super.setTurnState(desiredState);

        this.turnSetpointRAD = desiredState.angle.getRadians();
    }

    /**
     * 
     */
    private void applyDriveSettings() {
        if (this.isOpenLoop) {
            this.driveController.reset();
            this.driveAppliedVolts = MathUtil.clamp(this.driveSetpointPCT * 12.0, -12.0, 12.0);
            this.driveMotorSim.setInputVoltage(this.driveAppliedVolts);
            this.driveSimState.setSupplyVoltage(this.driveAppliedVolts);
        }
        else {
            // double radiansPerSecond = Conversions.MPSToRPS(this.driveSetpointMPS, CHOSEN_MODULE.wheelCircumference);
            // double pidOutput = this.driveController.calculate(this.driveVelocityMPS, radiansPerSecond);
            // this.driveAppliedVolts = MathUtil.clamp(radiansPerSecond + pidOutput, -12.0, 12.0);
            
            double velocityRPS = this.driveSetpointMPS / (CHOSEN_MODULE.wheelDiameter / 2);
            double voltage = this.feedForward.calculate(velocityRPS) + this.driveController.calculate(this.driveVelocityRPS, velocityRPS);
            this.driveAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
            this.driveMotorSim.setInputVoltage(this.driveAppliedVolts);
        }
    }

    /**
     * 
     */
    private void applyTurnSettings() {
        double pidOutput = this.turnController.calculate(this.turnRelativePositionRAD, this.turnSetpointRAD);
        this.turnAppliedVolts = MathUtil.clamp(pidOutput * 12.0, -12.0, 12.0);

        this.turnMotorSim.setInputVoltage(this.turnAppliedVolts);
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

        this.driveMotorSim.update(timeBetweenUpdates);
        this.turnMotorSim.update(timeBetweenUpdates);

        this.drivePositionRAD = this.driveMotorSim.getAngularPositionRad();
        this.driveVelocityRPS = this.driveMotorSim.getAngularVelocityRadPerSec();
        this.driveCurrentAmps = new double[] { Math.abs(this.driveMotorSim.getCurrentDrawAmps()) };

        this.driveVelocityMPS = Conversions.RADToMPS(this.driveMotorSim.getAngularVelocityRadPerSec(), CHOSEN_MODULE.wheelCircumference);
        this.driveDistanceMeters = this.driveDistanceMeters + (this.driveVelocityMPS * timeBetweenUpdates);
        this.driveAccelerationMPS = (this.driveVelocityMPS - this.drivePreviousVelocityMPS) / timeBetweenUpdates;
        this.drivePreviousVelocityMPS = this.driveVelocityMPS;
        this.drivePreviousTimestamp = Timer.getFPGATimestamp();

        this.turnAbsolutePosition = new Rotation2d(this.turnMotorSim.getAngularPositionRad()).plus(this.turnAbsoluteInitialPosition);
        this.turnPosition = new Rotation2d(this.turnMotorSim.getAngularPositionRad());
        this.turnVelocityRPS = this.turnMotorSim.getAngularVelocityRadPerSec();
        this.turnCurrentAmps = new double[] { Math.abs(this.turnMotorSim.getCurrentDrawAmps()) };

        double angleDiffRadians = this.turnMotorSim.getAngularVelocityRadPerSec() * RobotConstants.LOOP_PERIOD_SECS;
        this.turnRelativePositionRAD += angleDiffRadians;
        this.turnPositionDEG = this.turnRelativePositionRAD * (180.0 / Math.PI);
        this.turnVelocityRPM = Conversions.toRPM(angleDiffRadians, CHOSEN_MODULE.angleGearRatio);
    
        applyTurnSettings();
        applyDriveSettings();

        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/PositionRAD", this.drivePositionRAD);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/VelocityRPS", this.driveVelocityRPS);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/AppliedVolts", this.driveAppliedVolts);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/CurrentAmps", this.driveCurrentAmps);

        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/AccelerationMPS", this.driveAccelerationMPS);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/DistanceMeters", this.driveDistanceMeters);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/PositionDEG", this.drivePositionDEG);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/PositionDEG", this.drivePositionDEG);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/SetPointMPS", this.driveSetpointMPS);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/SetPointPCT", this.driveSetpointPCT);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/VelocityMPS", this.driveVelocityMPS);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Drive/AngularVelocityRPM", this.driveMotorSim.getAngularVelocityRPM());

        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Turn/AbsolutePosition", this.turnAbsolutePosition);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Turn/Position", this.turnPosition);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Turn/VelocityRPS", this.turnVelocityRPS);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Turn/AppliedVolts", this.turnAppliedVolts);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Turn/CurrentAmps", this.turnCurrentAmps);
        
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Turn/AbsolutePositionDEG", this.turnAbsolutePositionDEG);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Turn/PositionDEG", this.turnPositionDEG);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Turn/RelativePositionRAD", this.turnRelativePositionRAD);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Turn/SetpointRAD", this.turnSetpointRAD);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/Turn/VelocityRPM", this.turnVelocityRPM);
    }
}

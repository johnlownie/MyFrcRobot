package frc.robot.modules.swerve;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.CTREModuleState;
import frc.robot.Constants.SwerveModuleConstants;

/**
 * 
 */
// @AutoLog
abstract public class SwerveModule {
    protected int module_id;
    protected double last_angle;

    protected double driveSetpointMPS = 0.0;
    protected double driveSetpointPercentage = 0.0;
    protected double turnSetpointDegrees = 0.0;

    /* Calculated input values */
    protected double driveAcceleration = 0.0;
    protected double driveAppliedPercentage = 0.0;
    protected double driveAppliedVolts = 0.0;
    protected double driveDistanceMeters = 0.0;
    protected double drivePositionDegrees = 0.0;
    protected double drivePreviousTimestamp = 0.0;
    protected double drivePreviousVelocityMPS = 0.0;
    protected double driveVelocityMetersPerSecond = 0.0;
    protected double[] driveCurrentAmps = new double[] {};
    protected double[] driveTempCelsius = new double[] {};

    protected double turnAbsolutePositionDeg = 0.0;
    protected double turnAppliedPercentage = 0.0;
    protected double turnAppliedVolts = 0.0;
    protected double turnPositionDeg = 0.0;
    protected double turnVelocityRevPerMin = 0.0;
    protected double[] turnCurrentAmps = new double[] {};
    protected double[] turnTempCelsius = new double[] {};

    /**
     * 
     */
    public SwerveModule(int module_id) {
        this.module_id = module_id;
        this.last_angle = getState().angle.getDegrees();
    }

    /**
     *
     */
    public int getModuleId() { return this.module_id; }

    /**
     * 
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getStatePosition(), getStateRotation());
    }

    /**
     * Get the current state of this swerve module.
     *
     * @return the current state of this swerve module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(this.driveVelocityMetersPerSecond, Rotation2d.fromDegrees(this.turnPositionDeg));
    }

    /**
     * 
     */
    public double getStatePosition() {
        return this.driveDistanceMeters;
    }

    /**
     * 
     */
    public Rotation2d getStateRotation() {
        return Rotation2d.fromDegrees(this.turnPositionDeg);
    }

    /*
     * This is a custom optimize function, since default WPILib optimize assumes
     * continuous controller which CTRE and Rev onboard is not
     */
    public void setDesiredState(SwerveModuleState moduleState, boolean isOpenLoop, boolean forceAngle) {
        moduleState = CTREModuleState.optimize(moduleState, getState().angle);

        if (isOpenLoop) {
            double percentOuput = moduleState.speedMetersPerSecond / SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND;
            setDrivePercentage(percentOuput);
        }
        else {
            setDriveVelocity(moduleState.speedMetersPerSecond);
        }

        double angle = 0.0;
        if (!forceAngle && Math.abs(moduleState.speedMetersPerSecond) <= (SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.01)) {
            angle = this.last_angle;
        }
        else {
            angle = moduleState.angle.getDegrees();
        }

        this.last_angle = angle;
        setAnglePosition(this.last_angle);
    }

    /**
     * Set the drive motor to the specified voltage. This is only used for characterization via the
     * FeedForwardCharacterization command. The module will be set to 0 degrees throughout the
     * characterization; as a result, the wheels don't need to be clamped to hold them straight.
     *
     * @param voltage the specified voltage for the drive motor
     */
    public void setVoltageForCharacterization(double voltage) {
        setAnglePosition(0.0);

        this.last_angle = 0.0;
        setDrivePercentage(voltage / 12.0);
    }

    abstract protected void applyDriveSettings();
    abstract protected void applyTurnSettings();

    abstract public void setAnglePosition(double degrees);
    abstract protected void setDrivePercentage(double percentage);
    abstract protected void setDriveVelocity(double velocity);

    abstract public void reseedSteerMotorOffset();
    abstract public void updatePositions();

    // /**
    //  * Logging
    //  */
    // @Override
    // public void fromLog(LogTable table) {
    //     this.driveAppliedPercentage = table.get("Mod" + module_id + "/DriveAppliedPercentage", this.driveAppliedPercentage);
    //     this.driveAppliedVolts = table.get("Mod" + module_id + "/DriveAppliedVolts", this.driveAppliedVolts);
    //     this.driveDistanceMeters = table.get("Mod" + module_id + "/DriveDistanceMeters", this.driveDistanceMeters);
    //     this.drivePositionDegrees = table.get("Mod" + module_id + "/DrivePositionDeg", this.drivePositionDegrees);
    //     this.driveVelocityMetersPerSecond = table.get("Mod" + module_id + "/DriveVelocityMPS", this.driveVelocityMetersPerSecond);
    //     this.driveAcceleration = table.get("Mod" + module_id + "/DriveAccelerationMPS^2", this.driveAcceleration);
    //     this.driveCurrentAmps = table.get("Mod" + module_id + "/DriveCurrentAmps", this.driveCurrentAmps);
    //     this.driveTempCelsius = table.get("Mod" + module_id + "/DriveTempCelsius", this.driveTempCelsius);

    //     this.turnAbsolutePositionDeg = table.get("Mod" + module_id + "/TurnAbsolutePositionDeg", this.turnAbsolutePositionDeg);
    //     this.turnAppliedPercentage = table.get("Mod" + module_id + "/TurnAppliedPercentage", this.turnAppliedPercentage);
    //     this.turnAppliedVolts = table.get("Mod" + module_id + "/TurnAppliedVolts", this.turnAppliedVolts);
    //     this.turnPositionDeg = table.get("Mod" + module_id + "/TurnPositionDeg", this.turnPositionDeg);
    //     this.turnVelocityRevPerMin = table.get("Mod" + module_id + "/TurnVelocityRevPerMin", this.turnVelocityRevPerMin);
    //     this.turnCurrentAmps = table.get("Mod" + module_id + "/TurnCurrentAmps", this.turnCurrentAmps);
    //     this.turnTempCelsius = table.get("Mod" + module_id + "/TurnTempCelsius", this.turnTempCelsius);
    // }
  
    // @Override
    // public void toLog(LogTable table) {
    //     table.put("Mod" + module_id + "/DriveSetpointMPS", this.driveSetpointMPS);
    //     table.put("Mod" + module_id + "/DriveSetpointPercentage", this.driveSetpointPercentage);
    //     table.put("Mod" + module_id + "/DriveAppliedPercentage", this.driveAppliedPercentage);
    //     table.put("Mod" + module_id + "/DriveAppliedVolts", this.driveAppliedVolts);
    //     table.put("Mod" + module_id + "/DriveDistanceMeters", this.driveDistanceMeters);
    //     table.put("Mod" + module_id + "/DrivePositionDeg", this.drivePositionDegrees);
    //     table.put("Mod" + module_id + "/DriveVelocityMPS", this.driveVelocityMetersPerSecond);
    //     table.put("Mod" + module_id + "/DriveAccelerationMPS", this.driveAcceleration);
    //     table.put("Mod" + module_id + "/DriveCurrentAmps", this.driveCurrentAmps);
    //     table.put("Mod" + module_id + "/DriveTempCelsius", this.driveTempCelsius);

    //     table.put("Mod" + module_id + "/TurnSetpointDegrees", this.turnSetpointDegrees);
    //     table.put("Mod" + module_id + "/TurnAbsolutePositionDeg", this.turnAbsolutePositionDeg);
    //     table.put("Mod" + module_id + "/TurnAppliedPercentage", this.turnAppliedPercentage);
    //     table.put("Mod" + module_id + "/TurnAppliedVolts", this.turnAppliedVolts);
    //     table.put("Mod" + module_id + "/TurnPositionDeg", this.turnPositionDeg);
    //     table.put("Mod" + module_id + "/TurnVelocityRevPerMin", this.turnVelocityRevPerMin);
    //     table.put("Mod" + module_id + "/TurnCurrentAmps", this.turnCurrentAmps);
    //     table.put("Mod" + module_id + "/TurnTempCelsius", this.turnTempCelsius);
    // }
}

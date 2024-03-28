package frc.robot.modules.swerve;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

@AutoLog
abstract public class SwerveModule implements LoggableInputs {
    /* Variables to track data */
    public double drivePositionRAD = 0.0;
    public double driveVelocityRPS = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};

    public double driveAccelerationMPS = 0.0;
    public double driveDistanceMeters = 0.0;
    public double drivePositionDEG = 0.0;
    public double driveSetpointPCT = 0.0;
    public double driveSetpointMPS = 0.0;
    public double driveVelocityMPS = 0.0;
    public double drivePreviousTimestamp = 0.0;
    public double drivePreviousVelocityMPS = 0.0;

    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRPS = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrentAmps = new double[] {};

    public double turnAbsolutePositionDEG = 0.0;
    public double turnPositionDEG = 0.0;
    public double turnRelativePositionRAD = 0.0;
    public double turnSetpointRAD = 0.0;
    public double turnVelocityRPM = 0.0;

    public boolean isOpenLoop = false;

    /**
     * Set the drive motor to the specified voltage. This is only used for characterization via the
     * FeedForwardCharacterization command. The module will be set to 0 degrees throughout the
     * characterization; as a result, the wheels don't need to be clamped to hold them straight.
     *
     * @param voltage the specified voltage for the drive motor
     */
    public void setVoltageForCharacterization(double voltage) {
        this.turnSetpointRAD = Units.degreesToRadians(0.0);
        this.driveSetpointPCT = voltage;
        this.isOpenLoop = true;
    }

    @Override
    public void toLog(LogTable table) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void fromLog(LogTable table) {
        // TODO Auto-generated method stub
        
    }
}

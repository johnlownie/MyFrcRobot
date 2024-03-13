package frc.robot.modules.swerve;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

@AutoLog
abstract public class SwerveModule implements LoggableInputs {
    /* Variables to track data */
    public double angleAbsolutePositionDEG = 0.0;
    public double anglePositionDEG = 0.0;
    public double angleRelativePositionRAD = 0.0;
    public double angleSetpointDEG = 0.0;
    public double angleVelocityRPM = 0.0;
    
    public double driveAccelerationMPS = 0.0;
    public double driveDistanceMeters = 0.0;
    public double drivePositionDEG = 0.0;
    public double driveSetpointPCT = 0.0;
    public double driveSetpointMPS = 0.0;
    public double driveVelocityMPS = 0.0;
    public double drivePreviousTimestamp = 0.0;
    public double drivePreviousVelocityMPS = 0.0;

    @Override
    public void toLog(LogTable table) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void fromLog(LogTable table) {
        // TODO Auto-generated method stub
        
    }
}

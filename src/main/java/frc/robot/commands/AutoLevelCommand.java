package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class AutoLevelCommand extends CommandBase {
    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private final PIDController pid = new PIDController(0.04, 0, 0.0);
    private final Timer timer = new Timer();

    /**
     * 
     */
    public AutoLevelCommand(SwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;

        addRequirements(this.swerveDriveSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveDriveSubsystem.stop();
        this.timer.stop();
    }

    @Override
    public void execute() {
        double pidOutput = -pid.calculate(this.swerveDriveSubsystem.getGyro().getPitchDeg());
        ChassisSpeeds chassisSpeeds  = new ChassisSpeeds(pidOutput, 0, 0);

        this.swerveDriveSubsystem.drive(chassisSpeeds, false);
    }

    @Override
    public void initialize() {
        this.timer.reset();
        this.timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(3);
    }
}

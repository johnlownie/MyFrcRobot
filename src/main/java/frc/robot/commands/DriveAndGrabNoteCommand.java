package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.ProfiledPIDController;
import frc.robot.Constants.TeleopConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveAndGrabNoteCommand extends Command {
    private final SwerveDriveSubsystem swerveDrive;
    private final VisionSubsystem visionSubsystem;
    private final Supplier<Pose2d> poseProvider;

    private final ProfiledPIDController xController = TeleopConstants.xController;
    private final ProfiledPIDController yController = TeleopConstants.yController;
    private final ProfiledPIDController omegaController = TeleopConstants.omegaController;

    /**
     * 
     */
    DriveAndGrabNoteCommand(SwerveDriveSubsystem swerveDrive, VisionSubsystem visionSubsystem, Supplier<Pose2d> poseProvider) {
        this.swerveDrive = swerveDrive;
        this.visionSubsystem = visionSubsystem;
        this.poseProvider = poseProvider;

        this.xController.setTolerance(0.2);
        this.yController.setTolerance(0.2);
        this.omegaController.setTolerance(Units.degreesToRadians(3));
        this.omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.swerveDrive);
        addRequirements(this.visionSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveDrive.stop();
            
        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {
    }
  
    @Override
    public void initialize() {
    }

    /**
     * 
     */
    private void resetPIDControllers() {
        Pose2d robotPose = this.poseProvider.get();

        this.xController.reset(robotPose.getX());
        this.yController.reset(robotPose.getY());
        this.omegaController.reset(robotPose.getRotation().getRadians());
    }
}

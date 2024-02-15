package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.led.LEDController;
import frc.lib.led.LEDPreset;
import frc.lib.util.Timer;
import frc.robot.modules.vision.AprilTagShootData;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * 
 */
public class DeployGamePieceCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final boolean fromFrontCamera;

    private ArmSubsystem.Action moveTo;
    private ShooterSubsystem.Action shootTo;
    private boolean first_run;
    private final Timer timer = new Timer();
    
    /**
     * 
     */
    public DeployGamePieceCommand(ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, boolean fromFrontCamera) {
        this.armSubsystem = armSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.fromFrontCamera = fromFrontCamera;

        this.moveTo = null;
        this.shootTo = null;
        this.first_run = true;

        addRequirements(this.armSubsystem);
        addRequirements(this.shooterSubsystem);
        addRequirements(this.visionSubsystem);
    }
    
    @Override
    public void end(boolean interrupted) {
        this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_INTAKE);
        this.shooterSubsystem.addAction(ShooterSubsystem.Action.IDLE);
        LEDController.set(LEDPreset.Solid.kBlack);
        this.timer.stop();
        
        Logger.recordOutput("Commands/Active Command", "");
    }
    
    @Override
    public void execute() {
        if (this.moveTo == null || this.shootTo == null) {
            LEDController.set(LEDPreset.Solid.kRed);
            return;
        }

        if (this.first_run) {
            this.armSubsystem.addAction(this.moveTo);
            this.first_run = false;
        }
        else if (this.armSubsystem.isAtAngle()) {
            this.shooterSubsystem.addAction(this.shootTo);
        }
    }
    
    @Override
    public void initialize() {
        PhotonTrackedTarget target = this.visionSubsystem.getBestTarget(this.fromFrontCamera);

        if (target!= null) {
            this.moveTo = this.visionSubsystem.getMoveTo(target.getFiducialId());
            this.shootTo = this.visionSubsystem.getShootTo(target.getFiducialId());
        }

        this.timer.reset();
        this.timer.start();

        Logger.recordOutput("Commands/Active Command", this.getName());
    }
    
    @Override
    public boolean isFinished() {
        return this.shooterSubsystem.hasShot() || this.timer.hasElapsed(5.0);
    }
}

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TargetAndGrabNoteCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final Supplier<PhotonTrackedTarget> targetSupplier;

    /**
     * 
     */
    public TargetAndGrabNoteCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, Supplier<PhotonTrackedTarget> targetSupplier) {
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.targetSupplier = targetSupplier;

        addRequirements(armSubsystem, intakeSubsystem, shooterSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {
        PhotonTrackedTarget target = this.targetSupplier.get();

        // if we have a target an it is within 20 degress of center of robot
        if (target != null && (-10.0 <= target.getYaw() && target.getYaw() <= 10.0)) {
            this.intakeSubsystem.addAction(IntakeSubsystem.Action.INTAKE);
            this.shooterSubsystem.addAction(ShooterSubsystem.Action.INTAKE);
        }
    }

    @Override
    public void initialize() {
        this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_INTAKE);

        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    @Override
    public boolean isFinished() {
        return this.shooterSubsystem.hasNote();
    }
}

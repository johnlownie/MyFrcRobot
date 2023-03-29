package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Timer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Action;

/**
 * 
 */
public class DeployGamePieceMidCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;

    private final Timer timer = new Timer();
 
    /**
     * 
     */
    public DeployGamePieceMidCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;

        addRequirements(this.armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.timer.start();
        this.armSubsystem.addAction(Action.MOVE_TO_MID_NODE);
        this.armSubsystem.addAction(Action.RELEASE);
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.armSubsystem.isReleased() || this.timer.hasElapsed(5.0);
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.armSubsystem.addAction(Action.MOVE_TO_DRAWER);
        this.timer.stop();
        this.timer.reset();
    }
}

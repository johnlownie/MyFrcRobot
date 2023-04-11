package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

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
    
    @Override
    public void end(boolean interrupted) {
        this.armSubsystem.addAction(Action.MOVE_TO_DRAWER);
        this.timer.stop();
        
        Logger.getInstance().recordOutput("Commands/Active Command", "");
    }
    
    @Override
    public void execute() {
    }
    
    @Override
    public void initialize() {
        this.timer.reset();
        this.timer.start();
        this.armSubsystem.addAction(Action.MOVE_TO_MID_NODE);
        this.armSubsystem.addAction(Action.RELEASE);

        Logger.getInstance().recordOutput("Commands/Active Command", this.getName());
    }
    
    @Override
    public boolean isFinished() {
        return this.armSubsystem.isReleased() || this.timer.hasElapsed(5.0);
    }
}

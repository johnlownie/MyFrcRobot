package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.RobotConstants.DriveModeType;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/**
 * A command that runs pathfindThenFollowPath based on the current drive mode.
 */
public class PathFinderAndFollow extends Command {
    private final Supplier<DriveModeType> driveModeSupplier;
    private Command commandGroup;
    private Command followPathCommand;
    private DriveModeType driveMode;

    private final PathConstraints CONSTRAINTS = new PathConstraints(4.5, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    /**
     * Creates a new PathFinderAndFollow command.
     *
     * @param driveModeSupplier a supplier for the drive mode type
     */
    public PathFinderAndFollow(Supplier<DriveModeType> driveModeSupplier) {
        this.driveModeSupplier = driveModeSupplier;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        this.commandGroup.cancel();

        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {
        DriveModeType currentDriveMode = driveModeSupplier.get();
        if (this.driveMode != currentDriveMode) {
            this.commandGroup.cancel();
            schedulePathCommand();
        }
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Commands/Active Command", this.getName());
        
        schedulePathCommand();
    }

    @Override
    public boolean isFinished() {
        return followPathCommand.isFinished();
    }

    /** Runs a new autonomous path based on the current drive mode. */
    public void schedulePathCommand() {
        this.driveMode = driveModeSupplier.get();

        String pathName = this.driveMode == DriveModeType.SPEAKER ? "PathToSpeaker" : "PathToAmp";
        PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile(pathName);

        this.followPathCommand = AutoBuilder.pathfindThenFollowPath(pathPlannerPath, CONSTRAINTS, 0.0);
        this.commandGroup = Commands.sequence(this.followPathCommand);
        this.commandGroup.schedule();
    }
}

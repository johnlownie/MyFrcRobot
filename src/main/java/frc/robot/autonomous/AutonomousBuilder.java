package frc.robot.autonomous;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Stream;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutonomousBuilder {
    private final SwerveDriveSubsystem swerveDrive;
    private final PoseEstimatorSubsystem poseEstimator;

    private final SendableChooser<Command> autoChooser;
    private final SwerveAutoBuilder swerveAutoBuilder;
    private HashMap<String, Command> eventMap;

    /**
     * 
     */
    public AutonomousBuilder(SwerveDriveSubsystem swerveDrive, PoseEstimatorSubsystem poseEstimator) {
        this.swerveDrive = swerveDrive;
        this.poseEstimator = poseEstimator;
        this.eventMap = getEventMap();

        this.swerveAutoBuilder = new AutoBuilder(this.poseEstimator::getCurrentPose, this.poseEstimator::setCurrentPose, this.swerveDrive.getKinematics(),
        new PIDConstants(5.0, 0, 0), new PIDConstants(2.6, 0.001, 0), this.swerveDrive::setModuleStates,
        this.eventMap, true, new Command[] {}, this.swerveDrive);
        
        this.autoChooser = new SendableChooser<>();
        this.autoChooser.setDefaultOption("None", Commands.none());

        setPaths();
    }

    /**
     *
     */
    public void addDashboardWidgets(ShuffleboardTab tab) {
        tab.add("Autonomous", this.autoChooser).withSize(2, 1).withPosition(0, 3);
    }

    /**
     * Gets the currently selected auto command
     * @return auto command
     */
    public Command getAutonomousCommand() {
        return this.autoChooser.getSelected();
    }
    
    /**
     * 
     */
    private HashMap<String, Command> getEventMap() {
        HashMap<String, Command> eventMap = new HashMap<String, Command>();

        eventMap.put("CloseGripper", Commands.print("Grabbing Cube").withTimeout(3.0));
        eventMap.put("OpenGripper", Commands.print("Placing Cube").withTimeout(3.0));
        
        return eventMap;
    }

    /**
     * 
     */
    private Command getAutoBuildForPathGroup(String pathGroupName) {
        List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup(pathGroupName, new PathConstraints(4, 3));
        if (trajectories == null) {
            return Commands.print("*** Path failed to load: " + pathGroupName);
        }

        return this.swerveAutoBuilder.fullAuto(trajectories);
    }

    /**
     * 
     */
    private void setPaths() {
        try {
            Stream<Path> files = Files.list(Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "pathplanner"));

            files.filter(file -> !Files.isDirectory(file))
                .map(Path::getFileName)
                .map(Path::toString)
                .filter(fileName -> fileName.endsWith(".path"))
                .sorted()
                .map(pathName -> pathName.substring(0, pathName.lastIndexOf(".")))
                .forEach(pathName -> this.autoChooser.addOption(pathName, getAutoBuildForPathGroup(pathName)));
        } catch (IOException e) {
            System.out.println("*** Failed to load paths ***");
        }
    }
}

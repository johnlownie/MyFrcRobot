package frc.robot.commands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

    PIDController pid = new PIDController(0.04, 0, 0.0);

    Timer timer = new Timer();
    boolean timerRunning;

    /**
     * 
     */
    public AutoLevelCommand(SwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;

        addRequirements(this.swerveDriveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (!this.timerRunning) {
            this.timer.start();
            this.timerRunning = true;
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(3);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double pidOutput = -pid.calculate(this.swerveDriveSubsystem.getGyro().getPitchDeg());
        ChassisSpeeds chassisSpeeds  = new ChassisSpeeds(pidOutput, 0, 0);

        this.swerveDriveSubsystem.drive(chassisSpeeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.swerveDriveSubsystem.stop();
    }
}

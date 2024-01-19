package frc.robot.modules.swerve;

import java.time.LocalDateTime;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.util.Timer;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.utils.LoggedTunableNumber;

/**
 * 
 */
public class SwerveModuleSimulator extends SwerveModule {
    /* Simulated Drive Motor PID Values */
    private final double DRIVE_KP = 0.8;
    private final double DRIVE_KI = 0.0;
    private final double DRIVE_KD = 0.0;

    /* Simulated Turn Motor PID Values */
    private final double TURN_KP = 12.0;
    private final double TURN_KI = 0.0;
    private final double TURN_KD = 0.0;

    /* Simulated Drive Motor Characterization Values */
    private final double DRIVE_KS = 0.0545;  // 0.116970;
    private final double DRIVE_KV = 0.40126 / 12.0; // 0.133240;
    private final double DRIVE_KA = 0.0225;  // 0.0;

    /* Tunable PID */
    private final LoggedTunableNumber driveKp = new LoggedTunableNumber("Drive/DriveKp", DRIVE_KP);
    private final LoggedTunableNumber driveKi = new LoggedTunableNumber("Drive/DriveKi", DRIVE_KI);
    private final LoggedTunableNumber driveKd = new LoggedTunableNumber("Drive/DriveKd", DRIVE_KD);

    private final LoggedTunableNumber turnKp = new LoggedTunableNumber("Drive/TurnKp", TURN_KP);
    private final LoggedTunableNumber turnKi = new LoggedTunableNumber("Drive/TurnKi", TURN_KI);
    private final LoggedTunableNumber turnKd = new LoggedTunableNumber("Drive/TurnKd", TURN_KD);

    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);

    /* Simulated Motors */
    private FlywheelSim driveMotorSim;
    private FlywheelSim angleMotorSim;

    private PIDController driveController;
    private PIDController angleController;
    
    private TalonFXSimState driveSimState;
    private TalonFXSimState angleSimState;

    private final CANcoderSimState canCoderSimState;

    /* Local Variables */
    private double angleAbsolutePositionRadians = Math.random() * 2.0 * Math.PI;
    private double angleAppliedVolts = 0.0;
    private double angleRelativePositionRadians = 0.0;
    private double angleSetpointDegrees = 0.0;

    private double driveAppliedVolts = 0.0;
    private double driveSetpointMPS = 0.0;
    private double driveSetpointPercentage = 0.0;
    private double driveVelocityMetersPerSecond = 0.0;

    private boolean isOpenLoop;

    /**
     * 
     */
    public SwerveModuleSimulator(int module_id, int drive_motor_id, int angle_motor_id, int can_coder_id, Rotation2d angle_offset_degrees) {
        super(module_id, drive_motor_id, angle_motor_id, can_coder_id, angle_offset_degrees);

        this.driveMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), CHOSEN_MODULE.driveGearRatio, 0.025);
        this.angleMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), CHOSEN_MODULE.angleGearRatio, 0.004096955);

        this.driveController = new PIDController(driveKp.get(), driveKi.get(), driveKd.get());
        this.angleController = new PIDController(turnKp.get(), turnKi.get(), turnKd.get());

        this.driveSimState = this.driveMotor.getSimState();
        this.angleSimState = this.angleMotor.getSimState();
        this.canCoderSimState = this.encoder.getSimState();

        this.isOpenLoop = true;
    }

    /**
     * 
     */
    private void applyAngleSettings() {
        double pidOutput = this.angleController.calculate(this.angleRelativePositionRadians, this.angleSetpointDegrees * (Math.PI / 180.0));

        this.angleAppliedVolts = MathUtil.clamp(pidOutput, -12.0, 12.0);
        this.angleMotor.setVoltage(this.angleAppliedVolts);
    }

    /**
     * 
     */
    private void applyDriveSettings() {
        if (isOpenLoop) {
            this.driveController.reset();
            this.driveAppliedVolts = MathUtil.clamp(this.driveSetpointPercentage * 12.0, -12.0, 12.0);
            this.driveMotor.setVoltage(this.driveAppliedVolts);
        }
        else {
            double velocityRadiansPerSecond = this.driveSetpointMPS * (2.0 * Math.PI) / (CHOSEN_MODULE.wheelCircumference);
            double ffOutput = this.feedForward.calculate(velocityRadiansPerSecond); 
            double pidOutput =  this.driveController.calculate(this.driveVelocityMetersPerSecond, velocityRadiansPerSecond);
            
            this.driveAppliedVolts = MathUtil.clamp(ffOutput + pidOutput, -12.0, 12.0);
            this.driveMotor.setVoltage(this.driveAppliedVolts);

            Logger.recordOutput("Subsystems/SwerveDrive/SwerveModuleSimulator[" + getModuleId() + "]/Drive FF Output", ffOutput);
            Logger.recordOutput("Subsystems/SwerveDrive/SwerveModuleSimulator[" + getModuleId() + "]/Drive PID Output", pidOutput);
        }
    }
    
    /**
     * 
     */
    @Override
    public void updatePositions() {

        super.updateAnglePosition();
        super.updateDrivePosition();

        applyAngleSettings();
        applyDriveSettings();
  
        // Logger.getInstance().processInputs("SwerveModuleSimulator", this);
    }
}
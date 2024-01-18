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

    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);

    /* Simulated Motors */
    private FlywheelSim driveMotorSim;
    private FlywheelSim angleMotorSim;

    private PIDController driveController;
    private PIDController turnController;
    
    private TalonFXSimState driveSimState;
    private TalonFXSimState angleSimState;

    private final CANcoderSimState canCoderSimState;

    /**
     * 
     */
    public SwerveModuleSimulator(int module_id, int drive_motor_id, int angle_motor_id, int can_coder_id, Rotation2d angle_offset_degrees) {
        super(module_id, drive_motor_id, angle_motor_id, can_coder_id, angle_offset_degrees);

        this.driveMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), CHOSEN_MODULE.driveGearRatio, 0.025);
        this.angleMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), CHOSEN_MODULE.angleGearRatio, 0.004096955);

        this.driveSimState = this.driveMotor.getSimState();
        this.angleSimState = this.angleMotor.getSimState();
        this.canCoderSimState = this.encoder.getSimState();
    }

    /**
     * 
     */
    @Override
    protected void applyAngleSettings() {
        double pidOutput = this.turnController.calculate(this.turnRelativePositionRadians, this.turnSetpointDegrees * (Math.PI / 180.0));

        this.turnAppliedVolts = MathUtil.clamp(pidOutput, -12.0, 12.0);
        this.turnMotor.setInputVoltage(this.turnAppliedVolts);
    }

    /**
     * 
     */
    private void applyDriveSettings() {
        if (isOpenLoop) {
            this.driveController.reset();
            this.driveAppliedVolts = MathUtil.clamp(this.driveSetpointPercentage * 12.0, -12.0, 12.0);
            this.driveMotor.setInputVoltage(this.driveAppliedVolts);
        }
        else {
            double velocityRadiansPerSecond = this.driveSetpointMPS * (2.0 * Math.PI) / (MK4I_L2.WHEEL_CIRCUMFERENCE);
            double ffOutput = this.feedForward.calculate(velocityRadiansPerSecond); 
            double pidOutput =  this.driveController.calculate(this.driveVelocityMetersPerSecond, velocityRadiansPerSecond);
            
            this.driveAppliedVolts = MathUtil.clamp(ffOutput + pidOutput, -12.0, 12.0);
            this.driveMotor.setInputVoltage(this.driveAppliedVolts);

            Logger.getInstance().recordOutput("Subsystems/SwerveDrive/SwerveModuleSimulator[" + getModuleId() + "]/Drive FF Output", ffOutput);
            Logger.getInstance().recordOutput("Subsystems/SwerveDrive/SwerveModuleSimulator[" + getModuleId() + "]/Drive PID Output", pidOutput);
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
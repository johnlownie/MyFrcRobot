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

    /* Simulated Motors */
    private TalonFXSimState driveSimState;
    private TalonFXSimState angleSimState;

    private final CANcoderSimState canCoderSimState;

    /**
     * 
     */
    public SwerveModuleSimulator(int module_id, int drive_motor_id, int angle_motor_id, int can_coder_id, Rotation2d angle_offset_degrees) {
        super(module_id, drive_motor_id, angle_motor_id, can_coder_id, angle_offset_degrees);

        this.driveSimState = this.driveMotor.getSimState();
        this.angleSimState = this.angleMotor.getSimState();
        this.canCoderSimState = this.encoder.getSimState();
    }
}
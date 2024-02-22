package frc.robot.modules.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.RobotConstants;

public class ArmModuleSimulator extends ArmModule {
    private final double ARM_MIN_ANGLE_DEGRESS = -30.0;
    private final double ARM_MAX_ANGLE_DEGREES = 90.0;
    private final double ARM_REDUCTION = 200.0;
    private final double DISTANCE_PER_PULSE = 2.0 * Math.PI / 4096;
    private final double STARTING_ANGLE_RADIANS = 0.0;
    
    private final Vector<N1> STD_DEVS = VecBuilder.fill(DISTANCE_PER_PULSE);

    /* Motor PID Values */	
    protected final double KP = 50.0;	
    protected final double KI = 0.0;	
    protected final double KD = 0.0;	

    protected final PIDController pidController = new PIDController(KP, KI, KD);
    
    /* Simulated Hardware */
    private final DCMotor gearBox;
    private final PWMSparkMax motor;
    private final EncoderSim encoderSim;
    private final SingleJointedArmSim singleJointedArmSim;

    /**
     * 
     */
    public ArmModuleSimulator() {
        super();

        this.gearBox = DCMotor.getFalcon500(2);
        this.motor = new PWMSparkMax(2);
        this.encoderSim = new EncoderSim(new Encoder(9, 10));
        this.singleJointedArmSim = new SingleJointedArmSim(
            this.gearBox, 
            ARM_REDUCTION,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(30), 8.0),
            Units.inchesToMeters(16),
            Units.degreesToRadians(ARM_MIN_ANGLE_DEGRESS),
            Units.degreesToRadians(ARM_MAX_ANGLE_DEGREES),
            true,
            STARTING_ANGLE_RADIANS,
            STD_DEVS
        );

        this.encoderSim.setDistancePerPulse(DISTANCE_PER_PULSE);
        this.pidController.setTolerance(50);
    }

    @Override
    public boolean isAtAngle(double angle) {
        double difference = Units.radiansToDegrees(this.singleJointedArmSim.getAngleRads()) - angle;

        return -1.0 <= difference && difference <= 1.0;
    }
    
    @Override
    public void update() {
        if (RobotConstants.TUNING_MODE) {
            if (kp.hasChanged(hashCode()) || ki.hasChanged(hashCode()) || kp.hasChanged(hashCode())) {
                this.pidController.setPID(kp.get(), ki.get(), kd.get());
            }

            if (da.hasChanged(hashCode())) {
                setDesiredAngle(da.get());
            }
        }

        double pidOutput = this.pidController.calculate(this.encoderSim.getDistance(), Units.degreesToRadians(getDesiredAngle()));
        this.motor.setVoltage(pidOutput);

        this.singleJointedArmSim.setInput(MathUtil.clamp(this.motor.get() * 12.0, -12.0, 12.0));
        this.singleJointedArmSim.update(0.02);

        this.encoderSim.setDistance(this.singleJointedArmSim.getAngleRads());
        this.armModuleMechanism.setAngle(this.singleJointedArmSim.getAngleRads());

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(this.singleJointedArmSim.getCurrentDrawAmps()));

        Logger.recordOutput("Mechanisms/Arm/Desired Angle", getDesiredAngle());
        Logger.recordOutput("Mechanisms/Arm/Current Angle", getCurrentAngle());
        Logger.recordOutput("Mechanisms/Arm/PID Output", pidOutput);
    }

    @Override
    public double getCurrentAngle() {
        return Units.radiansToDegrees(this.singleJointedArmSim.getAngleRads());
    }
}

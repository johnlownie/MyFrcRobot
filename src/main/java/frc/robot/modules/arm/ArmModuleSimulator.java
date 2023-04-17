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
import frc.robot.utils.LoggedTunableNumber;

/**
 * 
 */
public class ArmModuleSimulator extends ArmModule {
    private final double ARM_MIN_ANGLE_DEGRESS = -75.0;
    private final double ARM_MAX_ANGLE_DEGREES = 255.0;
    private final double ARM_REDUCTION = 200.0;
    private final double DISTANCE_PER_PULSE = 2.0 * Math.PI / 4096;
    
    private final Vector<N1> STD_DEVS = VecBuilder.fill(DISTANCE_PER_PULSE);

    /* Motor PID Values */
    private final double KP = 50.0;
    private final double KI = 0.0;
    private final double KD = 0.0;

    /* Tunable PID */
    private final LoggedTunableNumber kp = new LoggedTunableNumber("Arm/Kp", KP);
    private final LoggedTunableNumber ki = new LoggedTunableNumber("Arm/Ki", KI);
    private final LoggedTunableNumber kd = new LoggedTunableNumber("Arm/Kd", KD);
    
    private final PIDController pidController = new PIDController(KP, KI, KD);
    
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
            Units.inchesToMeters(30),
            Units.degreesToRadians(ARM_MIN_ANGLE_DEGRESS),
            Units.degreesToRadians(ARM_MAX_ANGLE_DEGREES),
            true,
            STD_DEVS
        );

        this.encoderSim.setDistancePerPulse(DISTANCE_PER_PULSE);
        this.pidController.setTolerance(50);
    }

    @Override
    public boolean atAngle() {
        return atAngle(super.getDesiredAngle());
    }

    @Override
    public boolean atAngle(double angle) {
        double difference = Units.radiansToDegrees(this.singleJointedArmSim.getAngleRads()) - angle;

        return -1.0 <= difference && difference <= 1.0;
    }

    @Override
    public boolean isInnerLimitSwitchTriggered() {
        return Units.radiansToDegrees(this.singleJointedArmSim.getAngleRads()) <= ARM_MIN_ANGLE_DEGRESS;
    }

    @Override
    public boolean isOuterLimitSwitchTriggered(){
        return Units.radiansToDegrees(this.singleJointedArmSim.getAngleRads()) >= ARM_MAX_ANGLE_DEGREES;
    }

    @Override
    public void close() {
        this.armModuleMechanism.close();
    }

    @Override
    public void open() {
        this.armModuleMechanism.open();
    }

    @Override
    public void resetController() {
        this.pidController.reset();
    }

    @Override
    public void setDesiredAngle(double angle_in_degrees) {
        super.setDesiredAngle(angle_in_degrees);
    }

    @Override
    public void stop() {
        super.setEnabled(false);
    }

    @Override
    public void update() {
        if (RobotConstants.TUNING_MODE && (kp.hasChanged(hashCode()) || ki.hasChanged(hashCode()) || kp.hasChanged(hashCode()))) {
            this.pidController.setPID(kp.get(), ki.get(), kd.get());
        }

        double pidOutput = super.isEnabled() ? this.pidController.calculate(this.encoderSim.getDistance(), Units.degreesToRadians(getDesiredAngle())) : 0.0;
        this.motor.setVoltage(pidOutput);

        this.singleJointedArmSim.setInput(MathUtil.clamp(this.motor.get() * 12.0, -12.0, 12.0));
        this.singleJointedArmSim.update(0.02);

        this.encoderSim.setDistance(this.singleJointedArmSim.getAngleRads());
        this.armModuleMechanism.setAngle(this.singleJointedArmSim.getAngleRads());

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(this.singleJointedArmSim.getCurrentDrawAmps()));

        Logger.getInstance().recordOutput("Mechanisms/Arm/PID Output", pidOutput);
        Logger.getInstance().recordOutput("Mechanisms/Arm/SetPoint", super.getDesiredAngle());
        Logger.getInstance().recordOutput("Mechanisms/Arm/Angle", Units.radiansToDegrees(this.singleJointedArmSim.getAngleRads()));
    }
}

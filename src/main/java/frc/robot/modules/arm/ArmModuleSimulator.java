package frc.robot.modules.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ArmConstants;

public class ArmModuleSimulator extends ArmModule {
    private final double ARM_MIN_ANGLE_DEGRESS = ArmConstants.ANGLE_INTAKE;
    private final double ARM_MAX_ANGLE_DEGREES = ArmConstants.ANGLE_AMP;
    private final double ARM_REDUCTION = 200.0;
    private final double DISTANCE_PER_PULSE = 2.0 * Math.PI / 4096;
    private final int ENCODER_CHANNEL_A = 12;
    private final int ENCODER_CHANNEL_B = 13;
    private final double STARTING_ANGLE_RADIANS = Units.degreesToRadians(ArmConstants.ANGLE_INTAKE);
    
    private final Vector<N1> STD_DEVS = VecBuilder.fill(DISTANCE_PER_PULSE);
    
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
        this.encoderSim = new EncoderSim(new Encoder(ENCODER_CHANNEL_A, ENCODER_CHANNEL_B));
        this.singleJointedArmSim = new SingleJointedArmSim(
            this.gearBox, 
            ARM_REDUCTION,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(16), 4.0),
            Units.inchesToMeters(16),
            Units.degreesToRadians(ARM_MIN_ANGLE_DEGRESS),
            Units.degreesToRadians(ARM_MAX_ANGLE_DEGREES),
            true,
            STARTING_ANGLE_RADIANS,
            STD_DEVS
        );

        this.encoderSim.setDistancePerPulse(DISTANCE_PER_PULSE);
    }

    @Override
    public boolean isAtAngle(double angle) {
        double difference = Units.radiansToDegrees(this.singleJointedArmSim.getAngleRads()) - angle;

        return -1.0 <= difference && difference <= 1.0;
    }
    
    @Override
    public void update() {
        double pidOutput = this.pidController.calculate(this.encoderSim.getDistance(), Units.degreesToRadians(getDesiredAngle()));
        double voltage = MathUtil.clamp(pidOutput * 12.0, -12.0, 12.0);
        this.motor.setVoltage(voltage);

        this.singleJointedArmSim.setInput(voltage);
        this.singleJointedArmSim.update(0.02);

        this.encoderSim.setDistance(this.singleJointedArmSim.getAngleRads());
        this.armModuleMechanism.setAngle(this.singleJointedArmSim.getAngleRads());

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(this.singleJointedArmSim.getCurrentDrawAmps()));

        Logger.recordOutput("Mechanisms/Arm/DesiredAngle", getDesiredAngle());
        Logger.recordOutput("Mechanisms/Arm/CurrentAngle", getCurrentAngle());
        Logger.recordOutput("Mechanisms/Arm/PIDOutput", pidOutput);
        Logger.recordOutput("Mechanisms/Arm/Voltage", voltage);
    }

    @Override
    public double getCurrentAngle() {
        return Units.radiansToDegrees(this.singleJointedArmSim.getAngleRads());
    }
}

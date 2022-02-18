package frc.robot.util;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class PIDTuner implements Sendable {

    private BaseMotorController ctre;
    private CANSparkMax spark;
    private Consumer<Double> targetSetter;
    private Supplier<Double> currentStateGetter;
    private boolean enabled = false;
    private double target = 0.0;
    private double kP, kI, kD, kF;

    public PIDTuner(BaseMotorController ctre, Supplier<Double> currentStateGetter, Consumer<Double> targetSetter) {
        this.ctre = ctre;
        this.spark = null;
        this.currentStateGetter = currentStateGetter;
        this.targetSetter = targetSetter;
        loadGains(ctre);
    }
    public PIDTuner(BaseMotorController ctre) {
        this(ctre, () -> ctre.getSelectedSensorPosition(), (pos) -> ctre.set(ControlMode.Position, pos));
    }

    public PIDTuner(CANSparkMax spark, Supplier<Double> currentStateGetter, Consumer<Double> targetSetter) {
        this.ctre = null;
        this.spark = spark;
        this.currentStateGetter = currentStateGetter;
        this.targetSetter = targetSetter;
        loadGains(spark);
    }
    public PIDTuner(CANSparkMax spark) {
        this(spark, () -> spark.getEncoder().getPosition(), (pos) -> spark.getPIDController().setReference(pos, ControlType.kPosition));
    }

    private void loadGains(BaseMotorController ctre) {
        SlotConfiguration slot = new SlotConfiguration();
        ctre.getSlotConfigs(slot);
        kP = slot.kP;
        kI = slot.kI;
        kD = slot.kD;
        kF = slot.kF;
    }

    private void loadGains(CANSparkMax spark) {
        SparkMaxPIDController sparkPID = spark.getPIDController();
        kP = sparkPID.getP();
        kI = sparkPID.getI();
        kD = sparkPID.getD();
        kF = sparkPID.getFF();
    }

    private void updateSetpoint(boolean update) {
        if (enabled && update) {
            targetSetter.accept(target);
        }
    }
    private void refreshGains(boolean refresh) {
        if(refresh) {

        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Refresh Gains", () -> false, (r) -> refreshGains(r));
        builder.addBooleanProperty("Enabled", () -> enabled, (en) -> enabled = en);
        builder.addDoubleProperty("Target", () -> target, null);
        builder.addDoubleProperty("Set Target", () -> target, (t) -> target = enabled ? t : currentStateGetter.get());
        builder.addBooleanProperty("Update Target", () -> false, (u) -> updateSetpoint(u));
    }
    
}

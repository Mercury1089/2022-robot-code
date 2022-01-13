package frc.robot.util.MercMotorController;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.ControlType;

import frc.robot.util.*;
import frc.robot.util.interfaces.IMercMotorController;

public class MercSparkMax implements IMercMotorController {
    private CANSparkMax sparkmax;
    private int port;
    private double setPos;
    private boolean isInverted = false;

    public MercSparkMax(int port) {
        sparkmax = new CANSparkMax(port, CANSparkMaxLowLevel.MotorType.kBrushless);
        sparkmax.setSmartCurrentLimit(20);
        this.port = port;
    }

    @Override
    public void setSpeed(double speed) {
        sparkmax.set(speed);
    }

    @Override
    public void setPosition(double ticks) {
        setPos = MercMath.encoderTicksToRevs(ticks);
        sparkmax.getPIDController().setReference(setPos, ControlType.kPosition);
    }

    @Override
    public double getSpeed() {
        return sparkmax.get();
    }

    @Override
    public void setInverted(boolean invert) {
        sparkmax.setInverted(invert);
        this.isInverted = invert;
    }

    public boolean getInverted() {
        return this.isInverted;
    }

    @Override
    public int getPort() {
        return port;
    }

    @Override
    public void follow(IMercMotorController leader) {
        if (leader instanceof MercSparkMax) {
            MercSparkMax sparkLeader = (MercSparkMax) leader;
            sparkmax.follow(sparkLeader.get(), isInverted != sparkLeader.getInverted());
        }
    }

    @Override
    public void stop() {
        sparkmax.stopMotor();
    }

    @Override
    public double getEncTicks() {
        return MercMath.revsToEncoderTicks(sparkmax.getEncoder().getPosition());
    }

    @Override
    public double getEncVelocity() {
        return sparkmax.getEncoder().getVelocity();
    }

    public void setVelocity(double rpm){
        sparkmax.getPIDController().setReference(rpm, ControlType.kVelocity);
    }

    public void resetEncoder() {
        return;
    }

    @Override
    public double getClosedLoopError() {
        return setPos - sparkmax.getEncoder().getPosition();
    }

    @Override
    public void configPID(int slot, PIDGain gains) {
        sparkmax.getPIDController().setP(gains.kP, slot);
        sparkmax.getPIDController().setI(gains.kI, slot);
        sparkmax.getPIDController().setD(gains.kD, slot);
        sparkmax.getPIDController().setFF(gains.kF, slot);
    }

    @Override
    public void configVoltage(double nominalOutput, double peakOutput) {
        sparkmax.getPIDController().setOutputRange(nominalOutput, peakOutput);
    }

    @Override
    public void configNeutralDeadband(double percentDeadband) {
        return;
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        CANSparkMax.IdleMode mode;
        if (neutralMode == NeutralMode.Brake)
            mode = IdleMode.kBrake;
        else
            mode = IdleMode.kCoast;
        sparkmax.setIdleMode(mode);
    }

    @Override
    public void setSensorPhase(boolean sensorPhase) {
        return;
    }

    @Override
    public void configAllowableClosedLoopError(int slotIdx, int allowableCloseLoopError) {
        return;
    }

    @Override
    public void configSelectedFeedbackSensor(FeedbackDevice FeedbackDevice, int pidIdx) {
        return;
    }

    @Override
    public void configSetParameter(ParamEnum param, double value, int subValue, int ordinal) {
        return;
    }

    @Override
    public boolean isLimitSwitchClosed(LimitSwitchDirection limitSwitchDirection) {
        if (limitSwitchDirection == LimitSwitchDirection.FORWARD) {
            return sparkmax.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen).get();
        }
        return sparkmax.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen).get();
    }

    @Override
    public void configSensorTerm(SensorTerm st, FeedbackDevice fd) {
        return;
    }

    @Override
    public void configRemoteFeedbackFilter(int deviceID, RemoteSensorSource rss, int remoteSlotIdx) {
        return;
    }

    @Override
    public void configSelectedFeedbackCoefficient(double fdbkScale, int pidIdx) {
        return;
    }

    @Override
    public void setStatusFramePeriod(StatusFrame sf, int statusms) {
        return;
    }

    @Override
    public void selectProfileSlot(int slotIdx, int pidIdx) {
        return;
    }

    // _________________________________________________________________________________
    /**
     * Get the Spark Max tied to this class
     * 
     * @return the Spark
     */
    public CANSparkMax get() {
        return sparkmax;
    }

    @Override
    public double getClosedLoopError(int slotIdx) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void follow(IMercMotorController leader, FollowerType followerType) {
        // TODO Auto-generated method stub

    }

    @Override
    public void set(ControlMode controlMode, double demand0, DemandType demand1Type, double demand1) {
        // TODO Auto-generated method stub

    }

    @Override
    public void set(ControlMode controlMode, double demand0) {
        // TODO Auto-generated method stub

    }

    @Override
    public void configClosedLoopPeakOutput(int slotIdx, double peakOutput) {
        // TODO Auto-generated method stub

    }

    @Override
    public void setForwardSoftLimit(int limitTicks) {
        // TODO Auto-generated method stub

    }

    @Override
    public void enableForwardSoftLimit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void disableForwardSoftLimit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void setReverseSoftLimit(int limitTicks) {
        // TODO Auto-generated method stub

    }

    @Override
    public void enableReverseSoftLimit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void disableReverseSoftLimit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void configClosedLoopPeriod(int slotIdx, int closedLoopTimeMs) {
        // TODO Auto-generated method stub

    }

    @Override
    public void configAuxPIDPolarity(boolean invert) {
        // TODO Auto-generated method stub

    }

    @Override
    public void configMotionAcceleration(int sensorUnitsPer100msPerSec) {
        // TODO Auto-generated method stub

    }

    @Override
    public void configMotionCruiseVelocity(int sensorUnitsPer100ms) {
        // TODO Auto-generated method stub

    }

    @Override
    public void configFactoryReset() {
        // TODO Auto-generated method stub

    }
}
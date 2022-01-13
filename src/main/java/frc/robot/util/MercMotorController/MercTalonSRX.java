package frc.robot.util.MercMotorController;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.RobotMap;
import frc.robot.util.*;
import frc.robot.util.interfaces.IMercMotorController;

public class MercTalonSRX implements IMercMotorController {
    private WPI_TalonSRX talonsrx;
    private int port;

    public MercTalonSRX(int port) {
        talonsrx = new WPI_TalonSRX(port);
        this.port = port;
        talonsrx.configFactoryDefault();
    }

    @Override
    public void setPosition(double ticks) {
        talonsrx.set(ControlMode.Position, ticks);
    }

    @Override
    public double getSpeed() {
        return talonsrx.get();
    }

    @Override
    public void setSpeed(double speed) {
        talonsrx.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void setVelocity(double rpm){}

    @Override
    public void setInverted(boolean invert) {
        talonsrx.setInverted(invert);
    }

    @Override
    public int getPort() {
        return port;
    }

    @Override
    public void follow(IMercMotorController leader) {
        if (leader instanceof MercTalonSRX) {
            talonsrx.follow(((MercTalonSRX) leader).get());
        } else if (leader instanceof MercVictorSPX) {
            talonsrx.follow(((MercVictorSPX)leader).get());
        }
    }

    @Override
    public void stop() {
        talonsrx.stopMotor();
    }

    @Override
    public double getEncTicks() {
        return talonsrx.getSelectedSensorPosition(0);
    }

    @Override
    public double getEncVelocity() {
        return talonsrx.getSelectedSensorVelocity(0);
    }

    @Override
    public void resetEncoder() {
        talonsrx.getSensorCollection().setQuadraturePosition(0, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public double getClosedLoopError() {
        return talonsrx.getClosedLoopError(0);
    }

    @Override
    public void configPID(int slot, PIDGain gains) {
        talonsrx.config_kP(slot, gains.kP, 10);
        talonsrx.config_kI(slot, gains.kI, 10);
        talonsrx.config_kD(slot, gains.kD, 10);
        talonsrx.config_kF(slot, gains.kF, 10);
        talonsrx.configClosedLoopPeakOutput(slot, gains.clMaxOut, 10);
    }

    @Override
    public void configVoltage(double nominalOutput, double peakOutput) {
        talonsrx.configNominalOutputForward(nominalOutput, RobotMap.CTRE_TIMEOUT);
        talonsrx.configNominalOutputReverse(-nominalOutput, RobotMap.CTRE_TIMEOUT);
        talonsrx.configPeakOutputForward(peakOutput, RobotMap.CTRE_TIMEOUT);
        talonsrx.configPeakOutputReverse(-peakOutput, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void configNeutralDeadband(double percentDeadband) {
        talonsrx.configNeutralDeadband(percentDeadband);
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        talonsrx.setNeutralMode(neutralMode);
    }

    @Override
    public void setSensorPhase(boolean sensorPhase) {
        talonsrx.setSensorPhase(sensorPhase);
    }

    @Override
    public void configAllowableClosedLoopError(int slotIdx, int allowableCloseLoopError) {
        talonsrx.configAllowableClosedloopError(slotIdx, allowableCloseLoopError, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void configSelectedFeedbackSensor(FeedbackDevice feedbackDevice, int pidIdx) {
        talonsrx.configSelectedFeedbackSensor(feedbackDevice, pidIdx, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void configSetParameter(ParamEnum param, double value, int subValue, int ordinal) {
        talonsrx.configSetParameter(param, value, subValue, ordinal, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public boolean isLimitSwitchClosed(LimitSwitchDirection limitSwitchDirection) {
        if (limitSwitchDirection == LimitSwitchDirection.FORWARD) {
            return talonsrx.getSensorCollection().isFwdLimitSwitchClosed();
        }
        return talonsrx.getSensorCollection().isRevLimitSwitchClosed();
    }

    @Override
    public void setForwardSoftLimit(int limitTicks) {
        talonsrx.configForwardSoftLimitThreshold(limitTicks, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void enableForwardSoftLimit() {
        talonsrx.configForwardSoftLimitEnable(true, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void disableForwardSoftLimit() {
        talonsrx.configForwardSoftLimitEnable(false, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void setReverseSoftLimit(int limitTicks) {
        talonsrx.configReverseSoftLimitThreshold(limitTicks, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void enableReverseSoftLimit() {
        talonsrx.configReverseSoftLimitEnable(true, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void disableReverseSoftLimit() {
        talonsrx.configReverseSoftLimitEnable(false, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void configSensorTerm(SensorTerm st, FeedbackDevice fd) {
        talonsrx.configSensorTerm(st, fd, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void configRemoteFeedbackFilter(int deviceID, RemoteSensorSource rss, int remoteSlotIdx) {
        talonsrx.configRemoteFeedbackFilter(deviceID, rss, remoteSlotIdx);
    }

    @Override
    public void configSelectedFeedbackCoefficient(double fdbkScale, int pidIdx) {
        talonsrx.configSelectedFeedbackCoefficient(fdbkScale, pidIdx, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void setStatusFramePeriod(StatusFrame sf, int statusms) {
        talonsrx.setStatusFramePeriod(sf, statusms, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void selectProfileSlot(int slotIdx, int pidIdx) {
        talonsrx.selectProfileSlot(slotIdx, pidIdx);
    }

    @Override
    public void configClosedLoopPeakOutput(int slotIdx, double peakOutput) {
        talonsrx.configClosedLoopPeakOutput(slotIdx, peakOutput);
    }

    @Override
    public void set(ControlMode controlMode, double demand0, DemandType demand1Type, double demand1) {
        talonsrx.set(controlMode, demand0, demand1Type, demand1);
    }

    @Override
    public void set(ControlMode controlMode, double demand0) {
        talonsrx.set(controlMode, demand0);
    }

    @Override
    public void configClosedLoopPeriod(int slotIdx, int closedLoopTimeMs) {
        talonsrx.configClosedLoopPeriod(slotIdx, closedLoopTimeMs, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void configAuxPIDPolarity(boolean invert) {
        talonsrx.configAuxPIDPolarity(invert, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void configMotionAcceleration(int sensorUnitsPer100msPerSec) {
        talonsrx.configMotionAcceleration(sensorUnitsPer100msPerSec, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void configMotionCruiseVelocity(int sensorUnitsPer100ms) {
        talonsrx.configMotionCruiseVelocity(sensorUnitsPer100ms, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void follow(IMercMotorController leader, FollowerType followerType) {
        if (leader instanceof MercTalonSRX) {
            talonsrx.follow(((MercTalonSRX) leader).get(), followerType);
        } else if (leader instanceof MercVictorSPX) {
            talonsrx.follow(((MercVictorSPX) leader).get(), followerType);
        }
    }

    //_________________________________________________________________________________

    /**
     * Get the TalonSRX tied to this class
     *
     * @return the Talon
     */
    public WPI_TalonSRX get() {
        return talonsrx;
    }

    @Override
    public double getClosedLoopError(int slotIdx) {
        return talonsrx.getClosedLoopError(slotIdx);
    }

    public void configFactoryReset() {
        talonsrx.configFactoryDefault();
    }
}

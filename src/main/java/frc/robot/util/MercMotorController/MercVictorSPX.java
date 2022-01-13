package frc.robot.util.MercMotorController;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.RobotMap;
import frc.robot.util.*;
import frc.robot.util.interfaces.IMercMotorController;

public class MercVictorSPX implements IMercMotorController {
    private WPI_VictorSPX victorspx;
    private int port;

    public MercVictorSPX(int port) {
        victorspx = new WPI_VictorSPX(port);
        this.port = port;
        victorspx.configFactoryDefault();
    }

    @Override
    public void setPosition(double ticks) {
        victorspx.set(ControlMode.Position, ticks);
    }

    @Override
    public double getSpeed() {
        return victorspx.get();
    }

    @Override
    public void setSpeed(double speed) {
        victorspx.set(ControlMode.PercentOutput, speed);
    }

    public void setVelocity(double rpm){}

    @Override
    public void setInverted(boolean invert) {
        victorspx.setInverted(invert);
    }

    @Override
    public int getPort() {
        return port;
    }

    @Override
    public void follow(IMercMotorController leader) {
        if (leader instanceof MercTalonSRX) {
            victorspx.follow(((MercTalonSRX) leader).get());
        } else if (leader instanceof MercVictorSPX) {
            victorspx.follow(((MercVictorSPX)leader).get());
        }
    }

    @Override
    public void stop() {
        victorspx.stopMotor();
    }

    @Override
    public double getEncTicks() {
        return 0;
    }

    @Override
    public double getEncVelocity() {
        return 0;
    }

    @Override
    public void resetEncoder() {
        return;
    }

    @Override
    public double getClosedLoopError() {
        return victorspx.getClosedLoopError(0);
    }

    @Override
    public void configPID(int slot, PIDGain gains) {
        victorspx.config_kP(slot, gains.kP, 10);
        victorspx.config_kI(slot, gains.kI, 10);
        victorspx.config_kD(slot, gains.kD, 10);
        victorspx.config_kF(slot, gains.kF, 10);
        victorspx.configClosedLoopPeakOutput(slot, gains.clMaxOut, 10);
    }

    @Override
    public void configVoltage(double nominalOutput, double peakOutput) {
        victorspx.configNominalOutputForward(nominalOutput, RobotMap.CTRE_TIMEOUT);
        victorspx.configNominalOutputReverse(-nominalOutput, RobotMap.CTRE_TIMEOUT);
        victorspx.configPeakOutputForward(peakOutput, RobotMap.CTRE_TIMEOUT);
        victorspx.configPeakOutputReverse(-peakOutput, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void configNeutralDeadband(double percentDeadband) {
        victorspx.configNeutralDeadband(percentDeadband);
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        victorspx.setNeutralMode(neutralMode);
    }

    @Override
    public void setSensorPhase(boolean sensorPhase) {
        victorspx.setSensorPhase(sensorPhase);
    }

    @Override
    public void configAllowableClosedLoopError(int slotIdx, int allowableCloseLoopError) {
        victorspx.configAllowableClosedloopError(slotIdx, allowableCloseLoopError, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void configSelectedFeedbackSensor(FeedbackDevice feedbackDevice, int pidIdx) {
        victorspx.configSelectedFeedbackSensor(feedbackDevice, pidIdx, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void configSetParameter(ParamEnum param, double value, int subValue, int ordinal) {
        victorspx.configSetParameter(param, value, subValue, ordinal, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public boolean isLimitSwitchClosed(LimitSwitchDirection limitSwitchDirection) {
        return false;
    }

    @Override
    public void setForwardSoftLimit(int limitTicks) {
        victorspx.configForwardSoftLimitThreshold(limitTicks, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void enableForwardSoftLimit() {
        victorspx.configForwardSoftLimitEnable(true, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void disableForwardSoftLimit() {
        victorspx.configForwardSoftLimitEnable(false, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void setReverseSoftLimit(int limitTicks) {
        victorspx.configReverseSoftLimitThreshold(limitTicks, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void enableReverseSoftLimit() {
        victorspx.configReverseSoftLimitEnable(true, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void disableReverseSoftLimit() {
        victorspx.configReverseSoftLimitEnable(false, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void configSensorTerm(SensorTerm st, FeedbackDevice fd) {
        victorspx.configSensorTerm(st, fd, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void configRemoteFeedbackFilter(int deviceID, RemoteSensorSource rss, int remoteSlotIdx) {
        victorspx.configRemoteFeedbackFilter(deviceID, rss, remoteSlotIdx);
    }

    @Override
    public void configSelectedFeedbackCoefficient(double fdbkScale, int pidIdx) {
        victorspx.configSelectedFeedbackCoefficient(fdbkScale, pidIdx, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void setStatusFramePeriod(StatusFrame sf, int statusms) {
        victorspx.setStatusFramePeriod(sf, statusms, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void selectProfileSlot(int slotIdx, int pidIdx) {
        victorspx.selectProfileSlot(slotIdx, pidIdx);
    }

    @Override
    public void configClosedLoopPeakOutput(int slotIdx, double peakOutput) {
        victorspx.configClosedLoopPeakOutput(slotIdx, peakOutput);
    }

    @Override
    public void set(ControlMode controlMode, double demand0, DemandType demand1Type, double demand1) {
        victorspx.set(controlMode, demand0, demand1Type, demand1);
    }

    @Override
    public void set(ControlMode controlMode, double demand0) {
        victorspx.set(controlMode, demand0);
    }

    @Override
    public void configClosedLoopPeriod(int slotIdx, int closedLoopTimeMs) {
        victorspx.configClosedLoopPeriod(slotIdx, closedLoopTimeMs, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void configAuxPIDPolarity(boolean invert) {
        victorspx.configAuxPIDPolarity(invert, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void configMotionAcceleration(int sensorUnitsPer100msPerSec) {
        victorspx.configMotionAcceleration(sensorUnitsPer100msPerSec, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void configMotionCruiseVelocity(int sensorUnitsPer100ms) {
        victorspx.configMotionCruiseVelocity(sensorUnitsPer100ms, RobotMap.CTRE_TIMEOUT);
    }

    @Override
    public void follow(IMercMotorController leader, FollowerType followerType) {
        if (leader instanceof MercTalonSRX) {
            victorspx.follow(((MercTalonSRX) leader).get(), followerType);
        } else if (leader instanceof MercVictorSPX) {
            victorspx.follow(((MercVictorSPX) leader).get(), followerType);
        }
    }

    //_________________________________________________________________________________

    /**
     * Get the VictorSPX tied to this class
     *
     * @return the Victor
     */
    public WPI_VictorSPX get() {
        return victorspx;
    }

    @Override
    public double getClosedLoopError(int slotIdx) {
        return victorspx.getClosedLoopError(slotIdx);
    }

    @Override
    public void configFactoryReset() {
        victorspx.configFactoryDefault();
    }
}

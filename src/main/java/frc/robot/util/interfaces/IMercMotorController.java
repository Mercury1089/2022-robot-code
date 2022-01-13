package frc.robot.util.interfaces;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;

import frc.robot.util.PIDGain;

public interface IMercMotorController {

    /**
     * Sets relative position of controller
     *
     * @param position the final position in ticks
     */
    void setPosition(double position);

    /**
     * Want the speed of the controller?
     *
     * @return the speed of the controller
     */
    double getSpeed();

    /**
     * Sets the relative speed of the controller.
     *
     * @param speed speed from -1 to 1
     */
    void setSpeed(double speed);

    //TODO implement this
    //public void set()

    /**
     * Sets a motor controller inverted.
     *
     * @param invert Whether to invert or not
     */
    void setInverted(boolean invert);

    /**
     * Follows the motor controller that is passed in. Will not do anything if it has to follow anything else.
     *
     * @param leader the controller to follow
     */
    void follow(IMercMotorController leader);

    /**
     * Want the CAN ID of this controller?
     *
     * @return the CAN ID of this controller
     */
    int getPort();

    /**
     * STOP! You have been caught singing despacito 9: yodeling kid edition
     */
    void stop();

    /**
     * Want the encoder position of the controller?
     * NOTE: Victors can't connect to another sensor like an encoder, so it will return 0 for MercVictorSPX.
     *
     * @return the encoder position of the controller
     */
    double getEncTicks();

    /**
     * Want the encoder velocity of the controller?
     * NOTE: Victors can't connect to another sensor like an encoder, so it will return 0 for MercVictorSPX.
     *
     * @return the encoder velocity of the controller
     */
    double getEncVelocity();

    
    void setVelocity(double rpm);

    /**
     * Want to reset the encoder?
     */
    void resetEncoder();

    /**
     * Want the error from setpoint?
     * NOTE: Not to be used with Victors (no connected encoder)
     *
     * @return the error from setpoint in ticks
     */
    double getClosedLoopError();

    /**
     * Want the error from setpoint from a specific slot?
     * NOTE: Not to be used with Victors (no connected encoder)
     *
     * @return the error from setpoint in ticks
     */
    double getClosedLoopError(int slotIdx);

    /**
     * Configure PID for the motor controller
     *
     * @param slot  slot to put it in
     * @param gains i got mad gains
     */
    void configPID(int slot, PIDGain gains);

    /**
     * Configure the min and max voltage.
     * NOTE: for sparks, this is only for closed loop control. For talons, it is for both.
     *
     * @param nominalOutput min voltage
     * @param peakOutput    max voltage
     */
    void configVoltage(double nominalOutput, double peakOutput);

    /**
     * @param percentDeadband 
     */
    void configNeutralDeadband(double percentDeadband);

    /**
     * Set the neutral mode - brake or coast
     *
     * @param neutralMode the NeutralMode to set it to
     */
    void setNeutralMode(NeutralMode neutralMode);

    /**
     * NOTE: This is a CTRE Method.
     * Sets the direction of positive count of the selected sensor.
     *
     * @param bool whether the sensor follows forward motion (true) or not (false)
     */
    void setSensorPhase(boolean bool);

    void follow(IMercMotorController leader, FollowerType followerType);

    void set(ControlMode controlMode, double demand0, DemandType demand1Type, double demand1);

    void set(ControlMode controlMode, double demand0);

    /**
     * NOTE: This is a CTRE Method.
     * Configures the closed loop threshold of error.
     *
     * @param slotIdx                  the PID slot this applies to
     * @param allowableClosedLoopError the amount of error to allow.
     */
    void configAllowableClosedLoopError(int slotIdx, int allowableClosedLoopError);

    void configSelectedFeedbackSensor(FeedbackDevice FeedbackDevice, int pidIdx);

    void configSetParameter(ParamEnum param, double value, int subValue, int ordinal);

    boolean isLimitSwitchClosed(LimitSwitchDirection limitSwitchDirection);

    void configSensorTerm(SensorTerm st, FeedbackDevice fd);

    void configRemoteFeedbackFilter(int deviceID, RemoteSensorSource rss, int remoteSlotIdx);

    void configSelectedFeedbackCoefficient(double fdbkScale, int pidIdx);

    void setStatusFramePeriod(StatusFrame sf, int statusms);

    void selectProfileSlot(int slotIdx, int pidIdx);

    void configClosedLoopPeakOutput(int slotIdx, double peakOutput);

    void setForwardSoftLimit(int limitTicks);

    void enableForwardSoftLimit();

    void disableForwardSoftLimit();

    void setReverseSoftLimit(int limitTicks);

    void enableReverseSoftLimit();

    void disableReverseSoftLimit();

    void configClosedLoopPeriod(int slotIdx, int closedLoopTimeMs);

    //Below this line is motion magic stuff. Kyrag probably has something to do with this//
    //-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-//

    void configAuxPIDPolarity(boolean invert);

    void configMotionAcceleration(int sensorUnitsPer100msPerSec);

    void configMotionCruiseVelocity(int sensorUnitsPer100ms);

    void configFactoryReset();

    enum LimitSwitchDirection {
        FORWARD,
        REVERSE
    }
}
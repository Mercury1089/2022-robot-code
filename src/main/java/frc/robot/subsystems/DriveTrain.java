package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;
import frc.robot.RobotMap.CAN;
import frc.robot.util.DriveAssist;
import frc.robot.util.MercMath;
import frc.robot.util.PIDGain;
import frc.robot.util.interfaces.IMercPIDTunable;
import frc.robot.util.interfaces.IMercShuffleBoardPublisher;

/**
 * Subsystem that encapsulates the driveAssist train.
 * This contains the {@link DriveAssist} needed to driveAssist manually
 * using the motor controllers.
 */
public class DriveTrain extends SubsystemBase implements IMercShuffleBoardPublisher, IMercPIDTunable {

    public static final int
        DRIVE_PID_SLOT = 0,
        DRIVE_SMOOTH_MOTION_SLOT = 1,
        DRIVE_MOTION_PROFILE_SLOT = 2,
        DRIVE_SMOOTH_TURN_SLOT = 3;
    public static final int
        REMOTE_DEVICE_0 = 0,
        REMOTE_DEVICE_1 = 1;
    public static final int
        DISTANCE_LOOP = 0,
        YAW_LOOP = 1;
    public static final double 
        MAX_SPEED = 1,
        MIN_SPEED = -1;
    public static final double
        GEAR_RATIO = 1,
        MAX_RPM = 600,
        WHEEL_DIAMETER_INCHES = 6.0;
    public static final double
        DISTANCE_THRESHOLD_INCHES = 1.0,
        ANGLE_THRESHOLD_DEG = 1.2,
        ON_TARGET_THRESHOLD_DEG = 1.2;
    public static final double
        NOMINAL_OUT = 0.0,
        PEAK_OUT = 1.0,
        ROTATION_NEUTRAL_DEADBAND = 0.01,
        NEUTRAL_DEADBAND = 0.04;
 
    public static final int MOTOR_CONTROLLER_STATUS_FRAME_PERIOD_MS = 20;
    public static final int PIGEON_STATUS_FRAME_PERIOD_MS = 5;

    private PIDGain driveGains, smoothGains, motionProfileGains, turnGains;

    private BaseMotorController leaderLeft, leaderRight, followerLeft, followerRight;
    private CANCoder encLeft, encRight;

    private DriveAssist driveAssist;
    private PigeonIMU podgeboi;

    private DriveTrainLayout layout;
    private ShootingStyle shootingStyle;

    public enum DriveType {
        TANK,
        ARCADE
    }

    public enum ShootingStyle{
        AUTOMATIC,
        MANUAL,
        LOWER_PORT
    }

    /**
     * Creates the drivetrain, assuming that there are four controllers.
     *
     * @param layout The layout of motor controllers used on the drivetrain
     */
    public DriveTrain(DriveTrain.DriveTrainLayout layout) {
        //This should eventually be fully configurable
        // At this point it's based on what the layout is

        super();
        setName("DriveTrain");
        this.layout = layout;
        
        shootingStyle = ShootingStyle.AUTOMATIC;

        // Initialize the motor controllers and (if applicable) the encoders
        switch (layout) {
            case FALCONS:
                // Four TalonFX controllers with two CANCoders
                leaderLeft = new TalonFX(CAN.DRIVETRAIN_ML);
                leaderRight = new TalonFX(CAN.DRIVETRAIN_MR);
                followerLeft = new TalonFX(CAN.DRIVETRAIN_FL);
                followerRight = new TalonFX(CAN.DRIVETRAIN_FR);

                encLeft = new CANCoder(RobotMap.CAN.CANCODER_ML);
                encLeft.configFactoryDefault();
                encRight = new CANCoder(RobotMap.CAN.CANCODER_MR);
                encRight.configFactoryDefault();

                encLeft.configFeedbackCoefficient(1.0, "Ticks", SensorTimeBase.PerSecond);
                encRight.configFeedbackCoefficient(1.0, "Ticks", SensorTimeBase.PerSecond);


                encLeft.configSensorDirection(false);
                encRight.configSensorDirection(true);
                break;
            case TALONS_VICTORS:
                // Two TalonSRX controllers with mag encoders attached plus two VictorSPX followers
                leaderLeft = new TalonSRX(CAN.DRIVETRAIN_ML);
                leaderRight = new TalonSRX(CAN.DRIVETRAIN_MR);
                followerLeft = new VictorSPX(CAN.DRIVETRAIN_FL);
                followerRight = new VictorSPX(CAN.DRIVETRAIN_FR);

                encLeft = encRight = null;
                break;
        }
        leaderLeft.configFactoryDefault(); leaderRight.configFactoryDefault();
        followerLeft.configFactoryDefault(); followerRight.configFactoryDefault();

        //Initialize podgeboi
        podgeboi = new PigeonIMU(CAN.PIGEON);
        podgeboi.configFactoryDefault();

        //Account for motor orientation.
        leaderLeft.setInverted(false);
        followerLeft.setInverted(false);
        leaderRight.setInverted(true);
        followerRight.setInverted(true);

        //Set neutral mode to Brake to make sure our motor controllers are all in brake mode by default
        setNeutralMode(NeutralMode.Brake);

        //Account for encoder orientation.
        leaderLeft.setSensorPhase(false);
        leaderRight.setSensorPhase(true);

        //Config feedback sensors for each PID slot, ready for MOTION PROFILING
        configureFeedbackSensors(MOTOR_CONTROLLER_STATUS_FRAME_PERIOD_MS, PIGEON_STATUS_FRAME_PERIOD_MS);

        // Configure PID gains
        setPIDGain(DRIVE_PID_SLOT, new PIDGain(0.125, 0.0, 0.05, 0.0, .75));
        setPIDGain(DRIVE_SMOOTH_MOTION_SLOT, new PIDGain(0.7, 0.000185, 0.0, getFeedForward(), 1.0));
        setPIDGain(DRIVE_MOTION_PROFILE_SLOT, new PIDGain(0.1, 0.0, 0.0, getFeedForward(), 1.0));
        setPIDGain(DRIVE_SMOOTH_TURN_SLOT, new PIDGain(1.3, 0.0, 0.0, 0.0, 0.15));

        resetEncoders();

        driveAssist = new DriveAssist(leaderLeft, leaderRight);

        // Set follower control on back talons. Use follow() instead of ControlMode.Follower so that Talons can follow Victors and vice versa.
        followerLeft.follow(leaderLeft);
        followerRight.follow(leaderRight);

        configVoltage(NOMINAL_OUT, PEAK_OUT);
        setMaxOutput(PEAK_OUT);
        configNeutralDeadband(NEUTRAL_DEADBAND);
        stop();
    }

    private void configureFeedbackSensors(int framePeriodMs, int pigeonFramePeriodMs) {
        /* Configure left's encoder as left's selected sensor */
        if (layout == DriveTrainLayout.TALONS_VICTORS){
            leaderLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, DriveTrain.DISTANCE_LOOP, RobotMap.CTRE_TIMEOUT);

            /* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
            leaderRight.configRemoteFeedbackFilter(leaderLeft.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, DriveTrain.REMOTE_DEVICE_0, RobotMap.CTRE_TIMEOUT);
            /* Setup Sum signal to be used for Distance */
            leaderRight.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, RobotMap.CTRE_TIMEOUT);
            leaderRight.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative);
            /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
            leaderRight.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, DriveTrain.DISTANCE_LOOP, RobotMap.CTRE_TIMEOUT);
            /* Scale Feedback by 0.5 to half the sum of Distance */
            leaderRight.configSelectedFeedbackCoefficient(0.5, DriveTrain.DISTANCE_LOOP, RobotMap.CTRE_TIMEOUT);

        } else {
            /* Set up a Sum signal from both CANCoders on leaderLeft */
            leaderLeft.configRemoteFeedbackFilter(encLeft.getDeviceID(), RemoteSensorSource.CANCoder, DriveTrain.REMOTE_DEVICE_1);
            leaderLeft.configRemoteFeedbackFilter(encRight.getDeviceID(), RemoteSensorSource.CANCoder, DriveTrain.REMOTE_DEVICE_0);
            leaderLeft.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0);
            leaderLeft.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.RemoteSensor1);
            /* Configure the sensor sum as the selected sensor for leaderLeft with a coefficient of 0.5 (average) */
            leaderLeft.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, DriveTrain.DISTANCE_LOOP, RobotMap.CTRE_TIMEOUT);
            leaderLeft.configSelectedFeedbackCoefficient(0.5, DriveTrain.DISTANCE_LOOP, RobotMap.CTRE_TIMEOUT);
            /* Configure the selected sensor on leaderLeft (the avg.) as the remote sensor 0 for leaderRight */
            leaderRight.configRemoteFeedbackFilter(leaderLeft.getDeviceID(), RemoteSensorSource.TalonFX_SelectedSensor, DriveTrain.REMOTE_DEVICE_0);
            leaderRight.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, DriveTrain.DISTANCE_LOOP, RobotMap.CTRE_TIMEOUT);
        }
        /* Configure the Pigeon IMU to the other remote slot available on the right Talon */
        leaderRight.configRemoteFeedbackFilter(podgeboi.getDeviceID(), RemoteSensorSource.Pigeon_Yaw, DriveTrain.REMOTE_DEVICE_1);
        /* Configure Remote 1 [Pigeon IMU's Yaw] to be used for Auxiliary PID Index */
        leaderRight.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, DriveTrain.YAW_LOOP, RobotMap.CTRE_TIMEOUT);
        /* Scale the Feedback Sensor using a coefficient */
        leaderRight.configSelectedFeedbackCoefficient(1, DriveTrain.YAW_LOOP, RobotMap.CTRE_TIMEOUT);

        /* Motion Magic Configurations */
        leaderRight.configMotionAcceleration(1000);
        leaderRight.configMotionCruiseVelocity((int) MercMath.revsPerMinuteToTicksPerTenth(DriveTrain.MAX_RPM));

        int closedLoopTimeMs = 1;
        leaderRight.configClosedLoopPeriod(0, closedLoopTimeMs);
        leaderRight.configClosedLoopPeriod(1, closedLoopTimeMs);

        leaderRight.configAuxPIDPolarity(false);

        /* Set status frame periods to ensure we don't have stale data */
        leaderLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, framePeriodMs);
        leaderRight.setStatusFramePeriod(StatusFrame.Status_10_Targets, framePeriodMs);
        leaderRight.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, framePeriodMs);
        leaderRight.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, framePeriodMs);
        leaderRight.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, framePeriodMs);
        podgeboi.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, pigeonFramePeriodMs);
    }

    public void configPIDSlots(int primaryPIDSlot, int auxiliaryPIDSlot) {
        if (primaryPIDSlot >= 0) {
            leaderRight.selectProfileSlot(primaryPIDSlot, DriveTrain.DISTANCE_LOOP);
        }
        if (auxiliaryPIDSlot >= 0) {
            leaderRight.selectProfileSlot(auxiliaryPIDSlot, DriveTrain.YAW_LOOP);
        }
    }

    public void configClosedLoopPeakOutput(int driveTrainPIDSlot, double maxOut) {
        leaderLeft.configClosedLoopPeakOutput(driveTrainPIDSlot, maxOut);
        leaderRight.configClosedLoopPeakOutput(driveTrainPIDSlot, maxOut);
    }

    /**
     * Sets both of the front controllers to have a forward output of nominalOutput and peakOutput with the reverse output setClawState to the negated outputs.
     *
     * @param nominalOutput The desired nominal voltage output of the left and right talons, both forward and reverse.
     * @param peakOutput    The desired peak voltage output of the left and right talons, both forward and reverse
     */
    public void configVoltage(double nominalOutput, double peakOutput) {
        leaderLeft.configNominalOutputForward(nominalOutput, RobotMap.CTRE_TIMEOUT);
        leaderLeft.configNominalOutputReverse(-nominalOutput, RobotMap.CTRE_TIMEOUT);
        leaderLeft.configPeakOutputForward(peakOutput, RobotMap.CTRE_TIMEOUT);
        leaderLeft.configPeakOutputReverse(-peakOutput, RobotMap.CTRE_TIMEOUT);
        leaderRight.configNominalOutputForward(nominalOutput, RobotMap.CTRE_TIMEOUT);
        leaderRight.configNominalOutputReverse(-nominalOutput, RobotMap.CTRE_TIMEOUT);
        leaderRight.configPeakOutputForward(peakOutput, RobotMap.CTRE_TIMEOUT);
        leaderRight.configPeakOutputReverse(-peakOutput, RobotMap.CTRE_TIMEOUT);
    }

    /**
     * 
     * @param 
     */
    public void configNeutralDeadband(double percentDeadband) {
        leaderLeft.configNeutralDeadband(percentDeadband);
        leaderRight.configNeutralDeadband(percentDeadband);
    }

    public void setMaxOutput(double maxOutput) {
        driveAssist.setMaxOutput(maxOutput);
    }

    public void setNeutralMode(NeutralMode neutralMode) {
        leaderLeft.setNeutralMode(neutralMode);
        leaderRight.setNeutralMode(neutralMode);
        followerLeft.setNeutralMode(neutralMode);
        followerRight.setNeutralMode(neutralMode);
    }

    public void setShootingLocation(ShootingStyle shootingStyle){
        this.shootingStyle = shootingStyle;
    }

    public ShootingStyle getShootingStyle(){
        return shootingStyle;
    }

    public void resetEncoders() {
        if(layout == DriveTrainLayout.TALONS_VICTORS) {
            ((TalonSRX) leaderLeft).getSensorCollection().setQuadraturePosition(0, RobotMap.CTRE_TIMEOUT);
            ((TalonSRX) leaderLeft).getSensorCollection().setQuadraturePosition(0, RobotMap.CTRE_TIMEOUT);
        } else {
            encLeft.setPosition(0.0);
            encRight.setPosition(0.0);
        }
    }

    /**
     * Drive the robot using the arcade method (1 speed axis and 1 turn axis)
     * @param speedSupplier supplier of the speed value to drive
     * @param turnSupplier supplier of the turn value to drive
     * @param squareInputs square inputs to increase low-speed sensitivity
     */
    public void arcadeDrive(Supplier<Double> speedSupplier, Supplier<Double> turnSupplier, boolean squareInputs) {
        driveAssist.arcadeDrive(speedSupplier.get(), turnSupplier.get(), squareInputs);
    }

    /**
     * Drive the robot using the arcade method (1 speed axis and 1 turn axis)
     * @param speedSupplier supplier of the speed value to drive
     * @param turnSupplier supplier of the turn value to drive
     */
    public void arcadeDrive(Supplier<Double> speedSupplier, Supplier<Double> turnSupplier) {
        arcadeDrive(speedSupplier, turnSupplier, true);
    }

    /**
     * Drive the robot using the tank method (1 left speed axis and 1 right speed axis)
     * @param leftSpeedSupplier supplier of the left speed value to drive
     * @param rightSpeedSupplier supplier of the right speed value to drive
     */
    public void tankDrive(Supplier<Double> leftSpeedSupplier, Supplier<Double> rightSpeedSupplier) {
        driveAssist.tankDrive(leftSpeedSupplier.get(), rightSpeedSupplier.get());
    }

    /**
     * Drive the robot using CTRE MotionMagic closed-loop control
     * @param distance distance (in inches) to drive
     * @param heading heading (in degrees) to rotate to
     */
    public void moveHeading(double distance, double heading) {
        leaderLeft.follow(leaderRight, FollowerType.AuxOutput1);
        leaderRight.set(ControlMode.MotionMagic, MercMath.inchesToEncoderTicks(distance), DemandType.AuxPID, MercMath.degreesToPigeonUnits(heading));
    }

    /**
     * Drive the robot  CTRE MotionMagic closed-loop control
     * @param buffer buffer to stream trajectory points
     * @param minTime the minimum time duration of the points in the buffer
     */
    public void moveOnTrajectory(BufferedTrajectoryPointStream buffer, int minTime) {
        int halfFramePeriod = minTime / 2;
        if(halfFramePeriod < 1)
          halfFramePeriod = 1;
        leaderRight.changeMotionControlFramePeriod(halfFramePeriod);
        leaderLeft.follow(leaderRight, FollowerType.AuxOutput1);
        leaderRight.startMotionProfile(buffer, 20, ControlMode.MotionProfileArc);
    }

    /**
     * Clear any buffered trajectory
     */
    public void clearTrajectory() {
        leaderRight.clearMotionProfileTrajectories();
    }

    /**
     * Stops the drivetrain.
     */
    public void stop() {
        driveAssist.arcadeDrive(0.0, 0.0, true);
    }

    public boolean isTrajectoryFinished() {
        return leaderRight.isMotionProfileFinished();
    }

    /**
     * Get the current distance error for closed loop driving
     * @return the error in inches
     */
    public double getDistanceError() {
        return MercMath.encoderTicksToInches(leaderRight.getClosedLoopTarget(DISTANCE_LOOP) - leaderRight.getActiveTrajectoryPosition(DISTANCE_LOOP));
    }

    /**
     * Get the current angle error for closed loop driving
     * @return the error in degrees
     */
    public double getAngleError() {
        return MercMath.pigeonUnitsToDegrees(leaderRight.getClosedLoopTarget(YAW_LOOP) - leaderRight.getActiveTrajectoryPosition(YAW_LOOP));
    }

    public boolean isOnTarget() {
        return (Math.abs(getDistanceError()) < DISTANCE_THRESHOLD_INCHES &&
                Math.abs(getAngleError()) < ANGLE_THRESHOLD_DEG);
    }

    public double getDistanceTarget() {
        return MercMath.encoderTicksToInches(leaderRight.getClosedLoopTarget(DISTANCE_LOOP));
    }

    public double getAngleTarget() {
        return MercMath.pigeonUnitsToDegrees(leaderRight.getClosedLoopTarget(YAW_LOOP));
    }


    
    public double getPigeonYaw() {
        return leaderRight.getSelectedSensorPosition(YAW_LOOP);
    }

    public double getPigeonYawInDegrees() {
        return MercMath.pigeonUnitsToDegrees(getPigeonYaw());
    }

    public double getPositionInTicks() {
        return leaderRight.getSelectedSensorPosition(DISTANCE_LOOP);
    }

    public double getPositionInInches() {
        return MercMath.encoderTicksToInches(getPositionInTicks());
    }

    public void resetPigeonYaw() {
        podgeboi.setYaw(0);
    }

    public void calibrateGyro() {
        podgeboi.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
    }

    public double getLeftEncPositionInTicks() {
        if (layout == DriveTrainLayout.TALONS_VICTORS)
            return leaderLeft.getSelectedSensorPosition(0);
        else
            return encLeft.getPosition();
    }

    public double getRightEncPositionInTicks() {
        if (layout == DriveTrainLayout.TALONS_VICTORS)
            return leaderRight.getSelectedSensorPosition(0);
        else
            return encRight.getPosition();
    }
    public double getLeftEncVelocityInTicksPerTenth() {
        if (layout == DriveTrainLayout.TALONS_VICTORS)
            return leaderLeft.getSelectedSensorVelocity(0);
        else
            // CANCoder returns velocity in tick/s, so divide by 10
            return encLeft.getVelocity() / 10;
    }

    public double getRightEncVelocityInTicksPerTenth() {
        if (layout == DriveTrainLayout.TALONS_VICTORS)
            return leaderRight.getSelectedSensorVelocity(0);
        else
            // CANCoder returns velocity in tick/s, so divide by 10
            return encRight.getVelocity() / 10;
    }

    public double getLeftEncPositionInFeet() {
        return MercMath.getEncPosition(getLeftEncPositionInTicks());
    }

    public double getRightEncPositionInFeet() {
        return MercMath.getEncPosition(getRightEncPositionInTicks());
    }

    public double getFeedForward() {
        return MercMath.calculateFeedForward(MAX_RPM);
    }

    public enum DriveTrainLayout {
        TALONS_VICTORS,
        FALCONS
    }

    //Publish values to ShuffleBoard
    public void publishValues() {
        //Drive direction
        //SmartDashboard.putString(getName() + "/Direction", getDirection().name());
        //Encoder positions
        //SmartDashboard.putNumber(getName() + "/Left Encoder (feet)", getLeftEncPositionInFeet());
        //SmartDashboard.putNumber(getName() + "/Right Encoder (feet)", getRightEncPositionInFeet());
        //SmartDashboard.putNumber(getName() + "/Left Encoder (ticks)", getLeftEncPositionInTicks());
        //SmartDashboard.putNumber(getName() + "/Right Encoder (ticks)", getRightEncPositionInTicks());
        //SmartDashboard.putNumber(getName() + "/Right Closed Loop Error (ticks)", leaderRight.getClosedLoopError());
        //Selected Sensor Position
        //SmartDashboard.putNumber(getName() + "/PID0 Sensor primary left", ((MercTalonSRX)leaderLeft).get().getSelectedSensorPosition(PRIMARY_LOOP));
        //SmartDashboard.putNumber(getName() + "/PID0 Sensor auxiliary left", ((MercTalonSRX)leaderLeft).get().getSelectedSensorPosition(AUXILIARY_LOOP));

        //SmartDashboard.putNumber(getName() + "/PID0 Sensor right", ((MercTalonSRX)leaderRight).get().getSelectedSensorPosition(PRIMARY_LOOP));
        //SmartDashboard.putNumber(getName() + "/PID1 Sensor", ((MercTalonSRX)leaderRight).get().getSelectedSensorPosition(AUXILIARY_LOOP));
        //Wheel RPM
        SmartDashboard.putNumber(getName() + "/Left RPM", MercMath.ticksPerTenthToRevsPerMinute(getLeftEncVelocityInTicksPerTenth()));
        SmartDashboard.putNumber(getName() + "/Right RPM", MercMath.ticksPerTenthToRevsPerMinute(getRightEncVelocityInTicksPerTenth()));
        //Angle From Pigeon
        SmartDashboard.putNumber(getName() + "/Yaw", getPigeonYaw());

        
        //Publish Current Command
        SmartDashboard.putString(getName() + "/Command", getCurrentCommand() != null ? getCurrentCommand().getName() : "None");

    }

        
    @Override
    public void initSendable(SendableBuilder builder) {
        
        builder.setActuator(true); // Only allow setting values when in Test mode
        builder.addDoubleProperty("ActiveTrajectoryPosition", () -> leaderRight.getActiveTrajectoryPosition(), null);

        builder.addDoubleProperty("getDistanceTarget", ()-> getDistanceTarget(), null);
        builder.addDoubleProperty("getAngleTarget", () -> getAngleTarget(), null);


    }

    public TrajectoryPoint updateTrajectoryPoint(TrajectoryPoint point, double currHeading, double currPos) {

        //update newPoint to attributes of point
        // change auxPos and pos by adding by current pos
        TrajectoryPoint newPoint = new TrajectoryPoint();
        newPoint.timeDur = point.timeDur;
        newPoint.velocity = point.velocity;
        newPoint.zeroPos = point.zeroPos;
        newPoint.profileSlotSelect0 = point.profileSlotSelect0;
        newPoint.profileSlotSelect1 = point.profileSlotSelect1;
        newPoint.useAuxPID = point.useAuxPID;
        newPoint.isLastPoint = point.isLastPoint;

        newPoint.auxiliaryPos = point.auxiliaryPos + currHeading;
        newPoint.position = point.position + currPos;

        return newPoint;
    }

    @Override
    public int[] getSlots() {
        return new int[] {
            DRIVE_PID_SLOT,
            DRIVE_SMOOTH_MOTION_SLOT,
            DRIVE_MOTION_PROFILE_SLOT,
            DRIVE_SMOOTH_TURN_SLOT
        };
    }

    @Override
    public PIDGain getPIDGain(int slot) {
        PIDGain gains = null;
        switch (slot) {
            case DRIVE_PID_SLOT:
                gains = driveGains;
                break;
            case DRIVE_SMOOTH_MOTION_SLOT:
                gains = smoothGains;
                break;
            case DRIVE_MOTION_PROFILE_SLOT:
                gains = motionProfileGains;
                break;
            case DRIVE_SMOOTH_TURN_SLOT:
                gains = turnGains;
                break;
        }
        return gains;
    }

    private void configPID(BaseMotorController talon, int slot, PIDGain gains) {
        talon.config_kP(slot, gains.kP, 10);
        talon.config_kI(slot, gains.kI, 10);
        talon.config_kD(slot, gains.kD, 10);
        talon.config_kF(slot, gains.kF, 10);
        talon.configClosedLoopPeakOutput(slot, gains.clMaxOut, 10);
    }

    @Override
    public void setPIDGain(int slot, PIDGain gains) {
        switch (slot) {
            case DRIVE_PID_SLOT:
                driveGains = gains;
                break;
            case DRIVE_SMOOTH_MOTION_SLOT:
                smoothGains = gains;
                break;
            case DRIVE_MOTION_PROFILE_SLOT:
                motionProfileGains = gains;
                break;
            case DRIVE_SMOOTH_TURN_SLOT:
                turnGains = gains;
                break;
        }
        configPID(leaderRight, slot, gains);
        configPID(leaderLeft, slot, gains);
    }
}

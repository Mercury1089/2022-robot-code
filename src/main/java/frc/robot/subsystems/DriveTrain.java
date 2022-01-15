package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;
import frc.robot.RobotMap.CAN;
import frc.robot.util.DriveAssist;
import frc.robot.util.MercMath;
import frc.robot.util.PIDGain;
import frc.robot.util.interfaces.IMercMotorController;
import frc.robot.util.interfaces.IMercPIDTunable;
import frc.robot.util.interfaces.IMercShuffleBoardPublisher;

import frc.robot.sensors.Limelight;

/**
 * Subsystem that encapsulates the driveAssist train.
 * This contains the {@link DriveAssist} needed to driveAssist manually
 * using the motor controllers.
 */
public class DriveTrain extends SubsystemBase implements IMercShuffleBoardPublisher, IMercPIDTunable {

    public static final int DRIVE_PID_SLOT = 0,
        DRIVE_SMOOTH_MOTION_SLOT = 1,
        DRIVE_MOTION_PROFILE_SLOT = 2,
        DRIVE_SMOOTH_TURN_SLOT = 3;
    public static final int REMOTE_DEVICE_0 = 0,
        REMOTE_DEVICE_1 = 1;
    public static final int PRIMARY_LOOP = 0,
        AUXILIARY_LOOP = 1;
    public static final int MAG_ENCODER_TICKS_PER_REVOLUTION = 4096,
        NEO_ENCODER_TICKS_PER_REVOLUTION = 42,
        PIGEON_NATIVE_UNITS_PER_ROTATION = 8192;
    public static final double MAX_SPEED = 1,
        MIN_SPEED = -1;
    public static final double GEAR_RATIO = 1,
        MAX_RPM = 610,
        WHEEL_DIAMETER_INCHES = 6.0;
    public static final double ANGLE_THRESHOLD_DEG = 1.2, ON_TARGET_THRESHOLD_DEG = 1.2;
    public static final double NOMINAL_OUT = 0.0,
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
    //private LIDAR lidar;
    private DriveTrainLayout layout;
    private boolean isInMotionMagicMode;
    private Limelight limelight;
    private ShootingStyle shootingStyle;

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
    public DriveTrain(DriveTrain.DriveTrainLayout layout, Limelight limelight) {
        //This should eventually be fully configurable
        // At this point it's based on what the layout is

        super();
        setName("DriveTrain");
        this.layout = layout;
        shootingStyle = ShootingStyle.AUTOMATIC;

        switch (layout) {
            case FALCONS:
                leaderLeft = new TalonFX(CAN.DRIVETRAIN_ML);
                leaderRight = new TalonFX(CAN.DRIVETRAIN_MR);
                followerLeft = new TalonFX(CAN.DRIVETRAIN_FL);
                followerRight = new TalonFX(CAN.DRIVETRAIN_FR);

                encLeft = new CANCoder(RobotMap.CAN.CANCODER_ML);
                encRight = new CANCoder(RobotMap.CAN.CANCODER_MR);

                encLeft.configFeedbackCoefficient(1.0, "Ticks", SensorTimeBase.PerSecond);
                encRight.configFeedbackCoefficient(1.0, "Ticks", SensorTimeBase.PerSecond);


                encLeft.configSensorDirection(false);
                encRight.configSensorDirection(true);
                break;
            case TALONS_VICTORS:
                leaderLeft = new TalonSRX(CAN.DRIVETRAIN_ML);
                leaderRight = new TalonSRX(CAN.DRIVETRAIN_MR);
                followerLeft = new VictorSPX(CAN.DRIVETRAIN_FL);
                followerRight = new VictorSPX(CAN.DRIVETRAIN_FR);

                encLeft = encRight = null;
                break;
        }

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
        initializeMotionMagicFeedback();

        // Config PID
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

        this.limelight = limelight;
    }

    public Command getDefaultCommand(){
        return CommandScheduler.getInstance().getDefaultCommand(this);
    }

    public void setShootingLocation(ShootingStyle shootingStyle){
        this.shootingStyle = shootingStyle;
    }

    public ShootingStyle getShootingStyle(){
        return shootingStyle;
    }

    public void setDefaultCommand(Command command){
        CommandScheduler.getInstance().setDefaultCommand(this, command);
    }

    public void initializeMotionMagicFeedback(int framePeriodMs, int pigeonFramePeriodMs) {
        /* Configure left's encoder as left's selected sensor */
        if (layout == DriveTrainLayout.TALONS_VICTORS){
            leaderLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, DriveTrain.PRIMARY_LOOP, RobotMap.CTRE_TIMEOUT);

            /* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
            leaderRight.configRemoteFeedbackFilter(leaderLeft.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, DriveTrain.REMOTE_DEVICE_0, RobotMap.CTRE_TIMEOUT);
            /* Setup Sum signal to be used for Distance */
            leaderRight.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, RobotMap.CTRE_TIMEOUT);
            leaderRight.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative);
            /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
            leaderRight.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, DriveTrain.PRIMARY_LOOP, RobotMap.CTRE_TIMEOUT);
            /* Scale Feedback by 0.5 to half the sum of Distance */
            leaderRight.configSelectedFeedbackCoefficient(0.5, DriveTrain.PRIMARY_LOOP, RobotMap.CTRE_TIMEOUT);

        } else {
            /* Set up a Sum signal from both CANCoders on leaderLeft */
            leaderLeft.configRemoteFeedbackFilter(encLeft.getDeviceID(), RemoteSensorSource.CANCoder, DriveTrain.REMOTE_DEVICE_1);
            leaderLeft.configRemoteFeedbackFilter(encRight.getDeviceID(), RemoteSensorSource.CANCoder, DriveTrain.REMOTE_DEVICE_0);
            leaderLeft.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0);
            leaderLeft.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.RemoteSensor1);
            /* Configure the sensor sum as the selected sensor for leaderLeft with a coefficient of 0.5 (average) */
            leaderLeft.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, DriveTrain.PRIMARY_LOOP, RobotMap.CTRE_TIMEOUT);
            leaderLeft.configSelectedFeedbackCoefficient(0.5, DriveTrain.PRIMARY_LOOP, RobotMap.CTRE_TIMEOUT);
            /* Configure the selected sensor on leaderLeft (the avg.) as the remote sensor 0 for leaderRight */
            leaderRight.configRemoteFeedbackFilter(leaderLeft.getDeviceID(), RemoteSensorSource.TalonFX_SelectedSensor, DriveTrain.REMOTE_DEVICE_0);
            leaderRight.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, DriveTrain.PRIMARY_LOOP, RobotMap.CTRE_TIMEOUT);
        }
        /* Configure the Pigeon IMU to the other remote slot available on the right Talon */
        leaderRight.configRemoteFeedbackFilter(getPigeon().getDeviceID(), RemoteSensorSource.Pigeon_Yaw, DriveTrain.REMOTE_DEVICE_1);
        /* Configure Remote 1 [Pigeon IMU's Yaw] to be used for Auxiliary PID Index */
        leaderRight.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, DriveTrain.AUXILIARY_LOOP, RobotMap.CTRE_TIMEOUT);
        /* Scale the Feedback Sensor using a coefficient */
        leaderRight.configSelectedFeedbackCoefficient(1, DriveTrain.AUXILIARY_LOOP, RobotMap.CTRE_TIMEOUT);

        /* Set status frame periods to ensure we don't have stale data */
        setStatusFramePeriod(framePeriodMs, pigeonFramePeriodMs);
        isInMotionMagicMode = true;
    }

    public void initializeMotionMagicFeedback() {
        initializeMotionMagicFeedback(MOTOR_CONTROLLER_STATUS_FRAME_PERIOD_MS, PIGEON_STATUS_FRAME_PERIOD_MS);
    }

    public boolean isReadyToShoot(){
        switch(shootingStyle) {
            case AUTOMATIC:
                return isAligned();
            case MANUAL:
                return true;
            case LOWER_PORT:
                return true;
        }
        return false;
    }
    public Limelight getLimelight() {
        return this.limelight;
    }

    public void setStatusFramePeriod(int framePeriodMs) {
        leaderLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, framePeriodMs);
        leaderRight.setStatusFramePeriod(StatusFrame.Status_10_Targets, framePeriodMs);
        leaderRight.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, framePeriodMs);
        leaderRight.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, framePeriodMs);
        leaderRight.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, framePeriodMs);
    }

    public void setStatusFramePeriod(int framePeriodMs, int pigeonFramePeriodMs) {
        setStatusFramePeriod(framePeriodMs);
        getPigeon().setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, pigeonFramePeriodMs);
    }

    public void configPIDSlots(DriveTrainSide dts, int primaryPIDSlot, int auxiliaryPIDSlot) {
        if (primaryPIDSlot >= 0) {
            if (dts == DriveTrainSide.RIGHT)
                leaderRight.selectProfileSlot(primaryPIDSlot, DriveTrain.PRIMARY_LOOP);
            else
                leaderLeft.selectProfileSlot(primaryPIDSlot, DriveTrain.PRIMARY_LOOP);
        }
        if (auxiliaryPIDSlot >= 0) {
            if (dts == DriveTrainSide.RIGHT)
                leaderRight.selectProfileSlot(auxiliaryPIDSlot, DriveTrain.AUXILIARY_LOOP);
            else
                leaderLeft.selectProfileSlot(auxiliaryPIDSlot, DriveTrain.AUXILIARY_LOOP);
        }

    }

    public void configClosedLoopPeakOutput(int driveTrainPIDSlot, double maxOut) {
        leaderLeft.configClosedLoopPeakOutput(driveTrainPIDSlot, maxOut);
        leaderRight.configClosedLoopPeakOutput(driveTrainPIDSlot, maxOut);
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

    @Override
    public void periodic() {
    }

    /**
     * Sets the canifier LED output to the correct {@code LEDColor}. The
     * CANifier use BRG (not RGB) for its LED Channels
     */

    /**
     * Stops the driveAssist train.
     */
    public void stop() {
        driveAssist.arcadeDrive(0.0, 0.0, true);
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

    public boolean isAligned(){
        return limelight.getTargetAcquired() && Math.abs(limelight.getTargetCenterXAngle()) <= ON_TARGET_THRESHOLD_DEG;
    }

    public PigeonIMU getPigeon() {
        return podgeboi;
    }
    
    public double getPigeonYaw() {
        double[] currYawPitchRoll = new double[3];
        podgeboi.getYawPitchRoll(currYawPitchRoll);
        return currYawPitchRoll[0];
    }

    public DriveTrainLayout getLayout() {
        return layout;
    }

    public boolean isInMotionMagicMode() {
        return isInMotionMagicMode;
    }

    public void resetPigeonYaw() {
        podgeboi.setYaw(0);
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

    public BaseMotorController getLeftLeader() {
        return leaderLeft;
    }

    public BaseMotorController getRightLeader() {
        return leaderRight;
    }

    public BaseMotorController getLeftFollower() {
        return followerLeft;
    }

    public BaseMotorController getRightFollower() {
        return followerRight;
    }

    public DriveAssist getDriveAssist() {
        return driveAssist;
    }

    public double getFeedForward() {
        return MercMath.calculateFeedForward(MAX_RPM);
    }

    public void pidWrite(double output) {
        driveAssist.tankDrive(output, -output);
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



    public enum DriveTrainLayout {
        TALONS_VICTORS,
        FALCONS
    }

    public enum DriveTrainSide {
        RIGHT,
        LEFT
    }

    public enum LEDColor {
        RED(1.0, 0.0, 0.0),
        GREEN(0.0, 0.0, 1.0),
        BLUE(0.0, 0.0, 1.0),
        YELLOW(1.0, 1.0, 0.0),
        CYAN(0.0, 1.0, 1.0),
        MAGENTA(1.0, 0.0, 1.0),
        WHITE(1.0, 1.0, 1.0),
        BLACK(0.0, 0.0, 0.0);

        private double r, g, b;

        LEDColor(double r, double g, double b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }

        public double getRed() {
            return r;
        }

        public double getGreen() {
            return g;
        }

        public double getBlue() {
            return b;
        }
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
        //SmartDashboard.putNumber(getName() + "/Left RPM", MercMath.ticksPerTenthToRevsPerMinute(getLeftEncVelocityInTicksPerTenth()));
        //SmartDashboard.putNumber(getName() + "/Right RPM", MercMath.ticksPerTenthToRevsPerMinute(getRightEncVelocityInTicksPerTenth()));
        //Angle From Pigeon
        //SmartDashboard.putNumber(getName() + "/Yaw", getPigeonYaw());

        SmartDashboard.putBoolean(getName() + "/IsAligned", isAligned());
        SmartDashboard.putBoolean(getName() + "/TargetAcquired", limelight.getTargetAcquired());
        SmartDashboard.putBoolean(getName() + "/LimelightLEDState", limelight.getLEDState());
        //Publish Current Command
        SmartDashboard.putString(getName() + "/Command", getCurrentCommand() != null ? getCurrentCommand().getName() : "None");

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
                configPID(leaderRight, DRIVE_PID_SLOT, driveGains);
                configPID(leaderLeft, DRIVE_PID_SLOT, driveGains);
                break;
            case DRIVE_SMOOTH_MOTION_SLOT:
                smoothGains = gains;
                configPID(leaderRight, DRIVE_SMOOTH_MOTION_SLOT, smoothGains);
                configPID(leaderLeft, DRIVE_SMOOTH_MOTION_SLOT, smoothGains);
                break;
            case DRIVE_MOTION_PROFILE_SLOT:
                motionProfileGains = gains;
                configPID(leaderRight, DRIVE_MOTION_PROFILE_SLOT, motionProfileGains);
                configPID(leaderLeft, DRIVE_MOTION_PROFILE_SLOT, motionProfileGains);
                break;
            case DRIVE_SMOOTH_TURN_SLOT:
                turnGains = gains;
                configPID(leaderRight, DRIVE_SMOOTH_TURN_SLOT, turnGains);
                configPID(leaderLeft, DRIVE_SMOOTH_TURN_SLOT, turnGains);
                break;
        }
    }
}

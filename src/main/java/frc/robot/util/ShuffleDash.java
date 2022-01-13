package frc.robot.util;

import java.util.List;
import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.util.interfaces.IMercPIDTunable;
import frc.robot.util.interfaces.IMercShuffleBoardPublisher;

public class ShuffleDash {

    private static final String PID_TUNER = "PIDTuner/Target";
    private static final String PID_TUNER_P = "PIDTuner/kP";
    private static final String PID_TUNER_I = "PIDTuner/kI";
    private static final String PID_TUNER_D = "PIDTuner/kD";
    private static final String PID_TUNER_F = "PIDTuner/kF";
    private static final String PID_TUNER_MAX_OUTPUT = "PIDTuner/maxOutput";

    private static final double UPDATE_PERIOD_SECONDS = 0.200; // Update every 200ms

    private class TunablePIDSlot {
        public IMercPIDTunable tunable;
        public int slot;

        public TunablePIDSlot(IMercPIDTunable tunable, int slot) {
            this.tunable = tunable;
            this.slot = slot;
        }
    }

    private TunablePIDSlot tunableSlot = null;

    private NetworkTableInstance ntInstance;
    private SendableChooser<StartingPosition> autonPositionChooser;
    private SendableChooser<Autons> autonChooser;
    private StartingPosition oldPosition = StartingPosition.NULL;
    private List<IMercShuffleBoardPublisher> publishers;
    private SendableChooser<TunablePIDSlot> tunablePIDChooser;
    private String positionColor;
    private Notifier shuffleDashUpdater;

    public ShuffleDash() {

        SmartDashboard.putString("Position Control Color", getPositionControlColor());

        ntInstance = NetworkTableInstance.getDefault();

        autonPositionChooser = new SendableChooser<>();
        autonPositionChooser.addOption("Left", StartingPosition.LEFT);
        autonPositionChooser.addOption("Center", StartingPosition.CENTER);
        autonPositionChooser.addOption("Right", StartingPosition.RIGHT);
        autonPositionChooser.addOption("Far Right", StartingPosition.FAR_RIGHT);
        autonPositionChooser.addOption("Any Position", StartingPosition.ANY_POSITION);
        
        SmartDashboard.putData("Auton Position", autonPositionChooser);

        updateAutonChooser();

        publishers = new ArrayList<IMercShuffleBoardPublisher>();

        tunablePIDChooser = new SendableChooser<TunablePIDSlot>();
        tunablePIDChooser.setDefaultOption("NONE", null);
        SmartDashboard.putData(PID_TUNER, tunablePIDChooser);
        SmartDashboard.putNumber(PID_TUNER_P, 0.0);
        SmartDashboard.putNumber(PID_TUNER_I, 0.0);
        SmartDashboard.putNumber(PID_TUNER_D, 0.0);
        SmartDashboard.putNumber(PID_TUNER_F, 0.0);
        SmartDashboard.putNumber(PID_TUNER_MAX_OUTPUT, 0.0);

        shuffleDashUpdater = new Notifier(this::updateDash);
        shuffleDashUpdater.startPeriodic(UPDATE_PERIOD_SECONDS);
    }

    public void addPublisher(IMercShuffleBoardPublisher publisher) {
        publishers.add(publisher);
    }

    public void addPIDTunable(IMercPIDTunable pidTunable, String pidName) {
        for (int slot : pidTunable.getSlots()) {
            tunablePIDChooser.addOption(pidName + "." + Integer.toString(slot), new TunablePIDSlot(pidTunable, slot));
        }
    }

    public void updateDash() {

        for (IMercShuffleBoardPublisher publisher : publishers) {
            publisher.publishValues();
        }

        // PID Tuner
        TunablePIDSlot tunableSlot = tunablePIDChooser.getSelected();
        if (tunableSlot != null) {
            if (tunableSlot != this.tunableSlot) {
                PIDGain gains = tunableSlot.tunable.getPIDGain(tunableSlot.slot);
                SmartDashboard.putNumber(PID_TUNER_P, gains.kP);
                SmartDashboard.putNumber(PID_TUNER_I, gains.kI);
                SmartDashboard.putNumber(PID_TUNER_D, gains.kD);
                SmartDashboard.putNumber(PID_TUNER_F, gains.kF);
                SmartDashboard.putNumber(PID_TUNER_MAX_OUTPUT, gains.clMaxOut);
            } else {
                PIDGain gains = new PIDGain(SmartDashboard.getNumber(PID_TUNER_P, 0.0),
                        SmartDashboard.getNumber(PID_TUNER_I, 0.0), 
                        SmartDashboard.getNumber(PID_TUNER_D, 0.0),
                        SmartDashboard.getNumber(PID_TUNER_F, 0.0),
                        SmartDashboard.getNumber(PID_TUNER_MAX_OUTPUT, 0.0));
                tunableSlot.tunable.setPIDGain(tunableSlot.slot, gains);
            }
            this.tunableSlot = tunableSlot;
        }

        updateAutonChooser();
    }

    public StartingPosition getStartingPosition() {
        return autonPositionChooser.getSelected() == null ? StartingPosition.NULL : autonPositionChooser.getSelected();
    }

    public Autons getAuton() {
        return autonChooser.getSelected();
    }

    public void addLeftAutons() {
        autonChooser.addOption("Left", null);
        //autonChooser.addOption("5BallTrenchRun", "Left5BallTrench");
        autonChooser.addOption("2BallTrenchRun", Autons.LEFT_2BALL_TRENCH);
        autonChooser.addOption("5BallTrenchRun", Autons.LEFT_5BALL_TRENCH);
    }

    public void addRightAutons() {
        autonChooser.addOption("Right", null);
        //autonChooser.addOption("5BallRendezvousRun", "Right5BallRendezvous");
        //autonChooser.addOption("2BallRendezvousRun", Autons.RIGHT_2BALL_RENDEZVOUS);
        autonChooser.addOption("5BallRendezvousRun", Autons.RIGHT_5BALL_RENDEZVOUS);
    }

    public void addCenterAutons() {
        autonChooser.addOption("Center", null);
        //autonChooser.addOption("5BallRendezvousRun", "Center5BallRendezvous");
        //autonChooser.addOption("5BallTrenchRun", "Center5BallTrench");
        autonChooser.addOption("2BallRendezvousRun", Autons.CENTER_2BALL_RENDEZVOUS);
        autonChooser.addOption("5BallRendezvousRun", Autons.CENTER_5BALL_RENDEZVOUS);
        autonChooser.addOption("5BallTrenchRun", Autons.CENTER_5BALL_TRENCH);
    }

    public void addFarRightAutons() {
        autonChooser.addOption("Far Right", null);     
        //autonChooser.addOption("Opposite Trench Run", "StealOpponentTwoBall");
        autonChooser.addOption("Opposite Trench Run", Autons.STEAL_OPPONENT_2BALL);       
    }

    public void addAnyPositionAutons() {
        autonChooser.addOption("Any Position", null);     
        //autonChooser.addOption("Cross Initiation Line", "InitiationLine");
        //autonChooser.addOption("Cross Initiation Line And Shoot", "InitiationLineAndShoot");
        autonChooser.addOption("Cross Initiation Line", Autons.INITIATION_LINE);     
    }

    public void updateAutonChooser() {
        StartingPosition startingPosition = getStartingPosition();
        if(oldPosition == StartingPosition.NULL && startingPosition == StartingPosition.NULL)
            //autonChooser = new SendableChooser<String>();
            autonChooser = new SendableChooser<Autons>();
            
        if(startingPosition != oldPosition) {
            //autonChooser = new SendableChooser<String>();
            autonChooser = new SendableChooser<Autons>();
            switch(startingPosition) {
                case LEFT:
                    addLeftAutons();
                    oldPosition = StartingPosition.LEFT;
                    break;
                case RIGHT:
                    addRightAutons();
                    oldPosition = StartingPosition.RIGHT;
                    break;
                case CENTER:
                    addCenterAutons();
                    oldPosition = StartingPosition.CENTER;
                    break;
                case FAR_RIGHT:
                    addFarRightAutons();
                    oldPosition = StartingPosition.FAR_RIGHT;
                    break;
                case ANY_POSITION:
                    addAnyPositionAutons();
                    oldPosition = StartingPosition.ANY_POSITION;
                    break;
                default:
                    //autonChooser.addOption("No Option", "No Option");
                    autonChooser.addOption("No Option", Autons.NOTHING);
                    oldPosition = StartingPosition.NULL;
            }
        }

        SmartDashboard.putData("Choose Auton", autonChooser);
        //SmartDashboard.putString("Auton Chosen", autonChooser.getSelected() == null ? "Nothing" : autonChooser.getSelected();
        SmartDashboard.putString("Auton Chosen", autonChooser.getSelected() == null ? "Nothing" : autonChooser.getSelected().toString());
    }

    public String getPositionControlColor() {
        positionColor = DriverStation.getInstance().getGameSpecificMessage();
        if(positionColor.length() > 0)
            switch(positionColor.charAt(0)) {
                case 'R':
                    return "Red";
                case 'G':
                    return "Green";
                case 'B':
                    return "Blue";
                case 'Y':
                    return "Yellow";
                default:
                    return "Unknown/Glitch";
            }
        return "Unknown";
    }

    public enum StartingPosition {
        ANY_POSITION,
        CENTER,
        FAR_RIGHT,
        LEFT,
        NULL,
        RIGHT
    }

    public enum Autons {
        CENTER_2BALL_RENDEZVOUS,
        CENTER_5BALL_RENDEZVOUS,
        CENTER_5BALL_TRENCH,
        INITIATION_LINE,
        LEFT_2BALL_TRENCH,
        LEFT_5BALL_TRENCH,
        NOTHING,
        RIGHT_5BALL_RENDEZVOUS,
        STEAL_OPPONENT_2BALL
    }
}

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.util.interfaces.IMercPIDTunable;
import frc.robot.util.interfaces.IMercShuffleBoardPublisher;

public class ShuffleDash {

    private static final String PID_TUNER = "PIDTuner/Target";
    private static final String PID_TUNER_P = "PIDTuner/kP";
    private static final String PID_TUNER_I = "PIDTuner/kI";
    private static final String PID_TUNER_D = "PIDTuner/kD";
    private static final String PID_TUNER_F = "PIDTuner/kF";
    private static final String PID_TUNER_MAX_OUTPUT = "PIDTuner/maxOutput";

    private class TunablePIDSlot {
        public IMercPIDTunable tunable;
        public int slot;

        public TunablePIDSlot(IMercPIDTunable tunable, int slot) {
            this.tunable = tunable;
            this.slot = slot;
        }
    }

    private TunablePIDSlot tunableSlot = null;

    private SendableChooser<StartingPosition> autonPositionChooser;
    private SendableChooser<Autons> autonChooser;
    private StartingPosition oldPosition = StartingPosition.NULL;
    private List<IMercShuffleBoardPublisher> publishers;
    private SendableChooser<TunablePIDSlot> tunablePIDChooser;
    private String positionColor;
    private RobotContainer robotContainer;
    private Autons oldAuton = Autons.NOTHING;

    public ShuffleDash(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;

        SmartDashboard.putString("Position Control Color", getPositionControlColor());

        autonPositionChooser = new SendableChooser<>();
        autonPositionChooser.addOption("Any Position", StartingPosition.ANY_POSITION);
        autonPositionChooser.addOption("Noon", StartingPosition.NOON);
        autonPositionChooser.addOption("2 oClock", StartingPosition.TWO_OCLOCK);
        autonPositionChooser.addOption("4 oClock", StartingPosition.FOUR_OCLOCK);
        
        SmartDashboard.putData("Auton Position", autonPositionChooser);
        publishers = new ArrayList<IMercShuffleBoardPublisher>();

        tunablePIDChooser = new SendableChooser<TunablePIDSlot>();
        tunablePIDChooser.setDefaultOption("NONE", null);
        SmartDashboard.putData(PID_TUNER, tunablePIDChooser);
        SmartDashboard.putNumber(PID_TUNER_P, 0.0);
        SmartDashboard.putNumber(PID_TUNER_I, 0.0);
        SmartDashboard.putNumber(PID_TUNER_D, 0.0);
        SmartDashboard.putNumber(PID_TUNER_F, 0.0);
        SmartDashboard.putNumber(PID_TUNER_MAX_OUTPUT, 0.0);
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
        return autonPositionChooser == null ?
            StartingPosition.NULL :
            autonPositionChooser.getSelected() == null ? StartingPosition.NULL : autonPositionChooser.getSelected();
    }

    public Autons getAuton() {
        return autonChooser == null ?
            Autons.NOTHING :
            autonChooser.getSelected();
    }

    public void updateAutonChooser() {
        StartingPosition startingPosition = getStartingPosition();
        if(oldPosition == StartingPosition.NULL && startingPosition == StartingPosition.NULL)
            //autonChooser = new SendableChooser<String>();
            autonChooser = new SendableChooser<Autons>();
            
        if(startingPosition != oldPosition) {
            //autonChooser = new SendableChooser<String>();
            autonChooser = new SendableChooser<Autons>();

            autonChooser.addOption("No Option", Autons.NOTHING);
            autonChooser.addOption("Taxi", Autons.TAXI);

            switch(startingPosition) {
                case ANY_POSITION:
                    break;
                case NOON:
                    autonChooser.addOption("One Cargo", Autons.ONE_CARGO);
                    autonChooser.addOption("Two Cargo", Autons.TWO_CARGO);
                    break;
                case TWO_OCLOCK:
                    autonChooser.addOption("Two Cargo", Autons.TWO_CARGO);
                    break;
                case FOUR_OCLOCK:
                    break;
            }
            oldPosition = startingPosition;

        }

        SmartDashboard.putData("Choose Auton", autonChooser);
        //SmartDashboard.putString("Auton Chosen", autonChooser.getSelected() == null ? "Nothing" : autonChooser.getSelected();
        SmartDashboard.putString("Auton Chosen", autonChooser.getSelected() == null ? "Nothing" : autonChooser.getSelected().toString());

        Autons auton = getAuton();
        if (auton != oldAuton){
            robotContainer.initializeAutonCommand();
            oldAuton = auton;
        }
    }

    public String getPositionControlColor() {
        positionColor = DriverStation.getGameSpecificMessage();
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
        NULL,
        ANY_POSITION,
        NOON,
        TWO_OCLOCK,
        FOUR_OCLOCK
    }

    public enum Autons {
        NOTHING,
        TAXI,
        ONE_CARGO,
        TWO_CARGO
    }
}

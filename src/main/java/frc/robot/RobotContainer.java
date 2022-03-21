package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotMap.DS_USB;
import frc.robot.RobotMap.GAMEPAD_AXIS;
import frc.robot.RobotMap.GAMEPAD_BUTTONS;
import frc.robot.RobotMap.GAMEPAD_POV;
import frc.robot.RobotMap.JOYSTICK_ADJUSTMENTS;
import frc.robot.RobotMap.JOYSTICK_BUTTONS;
import frc.robot.commands.RunsDisabledInstantCommand;
import frc.robot.commands.Intake.RobotFullIntakeUp;
import frc.robot.commands.drivetrain.DriveWithJoysticks;
import frc.robot.commands.drivetrain.DriveWithJoysticks.DriveType;
import frc.robot.commands.drivetrain.MoveOnTrajectory;
import frc.robot.commands.drivetrain.MoveHeadingDerivatives.DriveDistance;
import frc.robot.commands.drivetrain.MoveHeadingDerivatives.MoveHeading;
import frc.robot.commands.feeder.LoadFeederTrigger;
import frc.robot.commands.feeder.ShootBall;
import frc.robot.commands.shooter.CheckRobotEmpty;
import frc.robot.commands.turret.RotateToTarget;
import frc.robot.commands.turret.ScanForTarget;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.Limelight.LimelightLEDState;
import frc.robot.sensors.REVColorMux.I2CMUX;
import frc.robot.sensors.REVColorMux.REVColor.ColorSensorID;
import frc.robot.subsystems.ClimberArticulator;
import frc.robot.subsystems.ClimberWinch;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveTrainLayout;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Feeder.BallMatchesAlliance;
import frc.robot.subsystems.Feeder.BreakBeamDIO;
import frc.robot.subsystems.Feeder.FeedSpeed;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeSpeed;
import frc.robot.subsystems.IntakeArticulator;
import frc.robot.subsystems.IntakeArticulator.IntakePosition;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterMode;
import frc.robot.subsystems.Turret;
import frc.robot.util.PovButton;
import frc.robot.util.ShuffleDash;
import frc.robot.util.TriggerButton;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class RobotContainer {
    public static final double DEADZONE = 0.08;

    private ShuffleDash shuffleDash;
    private SendableChooser<Autons> autonChooser;

    private Joystick rightJoystick, leftJoystick, gamepad;

    private JoystickButton left1, left2, left3, left4, left5, left6, left7, left8, left9, left10, left11;
    private JoystickButton right1, right2, right3, right4, right5, right6, right7, right8, right9, right10, right11;
    private JoystickButton gamepadA, gamepadB, gamepadX, gamepadY, gamepadRB, gamepadLB, gamepadL3, gamepadBack, gamepadStart, gamepadLeftStickButton, gamepadRightStickButton;
    private TriggerButton gamepadLT, gamepadRT;
    private PovButton gamepadPOVDown, gamepadPOVUp;

    private DriveTrain driveTrain;
    private Shooter shooter;
    private Turret turret;
    private Intake intake;
    private IntakeArticulator intakeArticulator;
    private Feeder frontFeeder, backFeeder;
    private ClimberArticulator climberArticulator;
    private ClimberWinch climberWinch;

    private Limelight limelight;
    
    private Command autonCommand = null;

    public Autons currentSelectedAuton = Autons.NOTHING;

    public RobotContainer() {
        


        leftJoystick = new Joystick(DS_USB.LEFT_STICK);
        rightJoystick = new Joystick(DS_USB.RIGHT_STICK);
        gamepad = new Joystick(DS_USB.GAMEPAD);

        limelight = new Limelight();

        turret = new Turret(limelight);
        turret.setDefaultCommand(new ScanForTarget(turret));
       //  turret.setDefaultCommand(new RunCommand(() -> turret.setSpeed(() -> getGamepadAxis(GAMEPAD_AXIS.leftX)*0.5), turret));

        driveTrain = new DriveTrain(DriveTrainLayout.FALCONS); //make sure to switch it back to Falcons
        driveTrain.setDefaultCommand(new DriveWithJoysticks(DriveType.ARCADE, driveTrain));

        shooter = new Shooter(ShooterMode.ONE_WHEEL, limelight);
        shooter.setDefaultCommand(new RunCommand(() -> shooter.setVelocity(shooter.getVelocityToTarget()), shooter));

        
        

        
        
        I2CMUX mux = new I2CMUX();

        frontFeeder = new Feeder(ColorSensorID.FRONT, BreakBeamDIO.FRONT, RobotMap.CAN.FEEDER_F, mux);
        // /*
        // no ball in front or back --> run frontFeeder
        // ball in front but not back --> run frontFeeder
        // ball in back but not front --> run frontFeeder
        // */
        // frontFeeder.setDefaultCommand(new LoadFeederTrigger(frontFeeder, () -> (!frontFeeder.isBeamBroken() && !backFeeder.isBeamBroken()) || 
        // (frontFeeder.isBeamBroken() && !backFeeder.isBeamBroken()) || 
        // (!frontFeeder.isBeamBroken() && backFeeder.isBeamBroken()) ));

        /*
        back feeder not shooting AND 
            (ball in front AND no ball in back OR
            front feeder empty AND IntakePosition.OUT)
        */
        frontFeeder.setDefaultCommand(new LoadFeederTrigger(frontFeeder, () ->
                !backFeeder.isShooting() && (
                    (frontFeeder.hasBall() && !backFeeder.hasBall()) ||
                    (!frontFeeder.hasBall() && intakeArticulator.getIntakePosition() == IntakePosition.OUT)
                )
        ));
       
        /*
        no ball in back AND
        (IntakePosition.OUT or ball in front)
        --> run the back feeder
        */
        backFeeder = new Feeder(ColorSensorID.BACK, BreakBeamDIO.BACK, RobotMap.CAN.FEEDER_B, mux);
        backFeeder.setDefaultCommand(new LoadFeederTrigger(backFeeder, () ->
                !backFeeder.hasBall() &&
                (intakeArticulator.getIntakePosition() == IntakePosition.OUT || frontFeeder.hasBall()
        )));
        // backFeeder.setDefaultCommand(new RunCommand(() -> backFeeder.setSpeed(FeedSpeed.STOP), backFeeder));

        intake = new Intake();
        intakeArticulator = new IntakeArticulator();
        intakeArticulator.setDefaultCommand(new RobotFullIntakeUp(intake, intakeArticulator, frontFeeder, backFeeder));

        climberArticulator = new ClimberArticulator();
        climberArticulator.setDefaultCommand(new RunCommand(() -> climberArticulator.setSpeed(() -> 0.0), climberArticulator));
      //  (new RunCommand(() -> turret.setSpeed(() -> getGamepadAxis(GAMEPAD_AXIS.leftX)*0.5), turret));

        climberWinch = new ClimberWinch();
        climberWinch.setDefaultCommand(new RunCommand(() -> climberWinch.setSpeed(() -> 0.0), climberWinch));

        shuffleDash = new ShuffleDash(this);
    
        initializeJoystickButtons();

        //driver controls
        //toggle intake in and out
        // left1.whenPressed(new ParallelCommandGroup(new RunCommand(() -> intakeArticulator.setIntakeOut(), intakeArticulator),
        //                                            new RunCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE), intake)));
        // left2.whenPressed(new ParallelCommandGroup(new RunCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake), 
        //                                            new RunCommand(() -> intakeArticulator.setIntakeIn(), intakeArticulator)));

        left1.whenPressed(new ParallelCommandGroup(new InstantCommand(() -> intakeArticulator.setIntakeOut(), intakeArticulator),
                                                   new InstantCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE), intake)));

        left2.whenPressed(new ParallelCommandGroup(new InstantCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake), 
                                                   new InstantCommand(() -> intakeArticulator.setIntakeIn(), intakeArticulator)));
                                
        left6.whenPressed(new RunsDisabledInstantCommand(() -> limelight.switchLEDState()));


        right2.whenPressed(new DriveWithJoysticks(DriveType.ARCADE, driveTrain));
        right10.whenPressed(new RunsDisabledInstantCommand(() -> turret.resetTurretPos(), turret));


        // Use the following to set velocity from SmartDash
        // gamepadA.whenPressed(new RunCommand(() -> shooter.setVelocity(shooter.getSmartDashboardRPM()), shooter));

        // Use the following to set velocity based on target distance
        gamepadA.whenPressed(new InstantCommand(() -> shooter.stopShooter(), shooter));
        gamepadB.whenPressed(new ParallelCommandGroup(
            new InstantCommand(() -> limelight.setLEDState(LimelightLEDState.OFF)),
            new RunCommand( () -> shooter.stopShooter(), shooter),
            new RunCommand( () -> turret.setPosition(180.0), turret)
        ));

        gamepadY.whenPressed(new RunCommand(() -> shooter.stopShooter(), shooter));
        gamepadX.whileHeld(new RunCommand(() -> climberWinch.setSpeed(() -> 1.0), climberWinch));

        

        gamepadLT.whenPressed(new ShootBall(backFeeder, shooter));

        gamepadLB.whenPressed(new ParallelCommandGroup(new RunCommand(() -> intakeArticulator.setIntakeOut(), intakeArticulator), 
                                                   new RunCommand(() -> intake.setSpeed(IntakeSpeed.EJECT), intake), 
                                                   new RunCommand(() -> frontFeeder.setSpeed(FeedSpeed.EJECT), frontFeeder),
                                                   new RunCommand(() -> backFeeder.setSpeed(FeedSpeed.EJECT), backFeeder)));

        gamepadRB.whenPressed(new ParallelCommandGroup(new RunCommand(() -> intake.setSpeed(IntakeSpeed.STOP), intake), 
        new RunCommand(() -> intakeArticulator.setIntakeIn(), intakeArticulator)));


        gamepadBack.and(gamepadStart).whenActive(new ParallelCommandGroup(
            new InstantCommand(() -> climberArticulator.setIsLocked(false), climberArticulator),
            new InstantCommand(() -> climberWinch.setIsLocked(false), climberWinch)
            )
        );


        gamepadPOVUp.whileHeld(new RunCommand(() -> climberArticulator.setSpeed(() -> 1.0), climberArticulator));
        gamepadPOVDown.whileHeld(new RunCommand(() -> climberArticulator.setSpeed(() -> -0.5), climberArticulator));



        
        

        // Trigger backFeederTrigger = new Trigger(() -> !backFeeder.isBeamBroken() ); 
        // backFeederTrigger.whenActive(
        //     new RunCommand(() -> backFeeder.setSpeed(0.60), backFeeder));  //  no ball in back feeder --> run back feeder
        
        // backFeederTrigger.whenInactive( // there is ball in backfeeder
        // // CHANGE THIS to 0
        //     new RunCommand(() -> backFeeder.setSpeed(0.6), backFeeder)); // ball in back feeder --> stop back feeder


        // // no ball in front or no ball in back OR ball in front NO BALL IN BACK 
        // Trigger firstFeederTrigger = new Trigger(() -> (!frontFeeder.isBeamBroken() && !backFeeder.isBeamBroken()) || 
        // (frontFeeder.isBeamBroken() && !backFeeder.isBeamBroken()) || 
        // (!frontFeeder.isBeamBroken() && backFeeder.isBeamBroken())); 

        // /*
        // no ball in front or back --> run front feeder
        // ball in front but not back --> run front feeder
        // ball in back but not front --> run front feeder
        // */
        // firstFeederTrigger.whenActive(new RunCommand(() -> frontFeeder.setSpeed(0.6), frontFeeder));  

        // // when ball in both feeders --> stop front feeder
        // // CHANGE THIS to 0
        // firstFeederTrigger.whenInactive(new RunCommand(() -> frontFeeder.setSpeed(0.6), frontFeeder));

        
        
        /*
        no ball in front or back --> run frontFeeder
        ball in front but not back --> run frontFeeder
        ball in back but not front --> run frontFeeder
        */

        Trigger unloadFeeder = new Trigger(() -> frontFeeder.whoseBall() == BallMatchesAlliance.DIFFERENT);
        unloadFeeder.whileActiveContinuous(new RunCommand(() -> frontFeeder.setSpeed(FeedSpeed.EJECT), frontFeeder));
        

        Trigger rotateTargetTrigger = new Trigger(() -> !turret.targetIsLost());
        rotateTargetTrigger.whileActiveContinuous(new RotateToTarget(turret));

        /*
        turret is on target AND
        shooter is at target RPM AND
        ball in back feeder
        --> shoot the ball
        */

        Trigger shootBall = new Trigger(() -> turret.isReadyToShoot() && shooter.isAtTargetVelocity() && backFeeder.hasBall() && driveTrain.isSafeShootingSpeed());
        shootBall.whileActiveContinuous(new ShootBall(backFeeder, shooter));

        autonChooser = new SendableChooser<Autons>();
        autonChooser.setDefaultOption("Four Cargo", Autons.FOUR_CARGO);
        autonChooser.addOption("Taxi", Autons.TAXI);
        autonChooser.addOption("Two Cargo", Autons.TWO_CARGO);
        autonChooser.addOption("Nothing", Autons.NOTHING);
        SmartDashboard.putData("Auton Chooser", autonChooser);
        updateAuton();
        

    }

    public double getJoystickX(int port) {
        switch (port) {
            case DS_USB.LEFT_STICK:
                return leftJoystick.getX() * JOYSTICK_ADJUSTMENTS.LEFT_X;
            case DS_USB.RIGHT_STICK:
                return rightJoystick.getX() * JOYSTICK_ADJUSTMENTS.RIGHT_X;
            default:
                return 0;
        }
    }

    public double getJoystickY(int port) {
        switch (port) {
            case DS_USB.LEFT_STICK:
                return leftJoystick.getY() * JOYSTICK_ADJUSTMENTS.LEFT_Y;
            case DS_USB.RIGHT_STICK:
                return rightJoystick.getY() * JOYSTICK_ADJUSTMENTS.RIGHT_Y;
            default:
                return 0;
        }
    }

    public double getJoystickZ(int port) {
        switch (port) {
            case DS_USB.LEFT_STICK:
                return leftJoystick.getZ();
            case DS_USB.RIGHT_STICK:
                return rightJoystick.getZ();
            default:
                return 0;
        }
    }

    public double getGamepadAxis(int axis) {
        return ((axis % 2 != 0 && axis != 3) ? -1.0 : 1.0) * gamepad.getRawAxis(axis);
    }


    private void initializeJoystickButtons() {
        left1 = new JoystickButton(leftJoystick, JOYSTICK_BUTTONS.BTN1);
        left2 = new JoystickButton(leftJoystick, JOYSTICK_BUTTONS.BTN2);
        left3 = new JoystickButton(leftJoystick, JOYSTICK_BUTTONS.BTN3);
        left4 = new JoystickButton(leftJoystick, JOYSTICK_BUTTONS.BTN4);
        left5 = new JoystickButton(leftJoystick, JOYSTICK_BUTTONS.BTN5);
        left6 = new JoystickButton(leftJoystick, JOYSTICK_BUTTONS.BTN6);
        left7 = new JoystickButton(leftJoystick, JOYSTICK_BUTTONS.BTN7);
        left8 = new JoystickButton(leftJoystick, JOYSTICK_BUTTONS.BTN8);
        left9 = new JoystickButton(leftJoystick, JOYSTICK_BUTTONS.BTN9);
        left10 = new JoystickButton(leftJoystick, JOYSTICK_BUTTONS.BTN10);
        left11 = new JoystickButton(leftJoystick, JOYSTICK_BUTTONS.BTN11);

        right1 = new JoystickButton(rightJoystick, JOYSTICK_BUTTONS.BTN1);
        right2 = new JoystickButton(rightJoystick, JOYSTICK_BUTTONS.BTN2);
        right3 = new JoystickButton(rightJoystick, JOYSTICK_BUTTONS.BTN3);
        right4 = new JoystickButton(rightJoystick, JOYSTICK_BUTTONS.BTN4);
        right5 = new JoystickButton(rightJoystick, JOYSTICK_BUTTONS.BTN5);
        right6 = new JoystickButton(rightJoystick, JOYSTICK_BUTTONS.BTN6);
        right7 = new JoystickButton(rightJoystick, JOYSTICK_BUTTONS.BTN7);
        right8 = new JoystickButton(rightJoystick, JOYSTICK_BUTTONS.BTN8);
        right9 = new JoystickButton(rightJoystick, JOYSTICK_BUTTONS.BTN9);
        right10 = new JoystickButton(rightJoystick, JOYSTICK_BUTTONS.BTN10);
        right11 = new JoystickButton(rightJoystick, JOYSTICK_BUTTONS.BTN11);

        gamepadA = new JoystickButton(gamepad, GAMEPAD_BUTTONS.A);
        gamepadB = new JoystickButton(gamepad, GAMEPAD_BUTTONS.B);
        gamepadX = new JoystickButton(gamepad, GAMEPAD_BUTTONS.X);
        gamepadY = new JoystickButton(gamepad, GAMEPAD_BUTTONS.Y);
        gamepadRB = new JoystickButton(gamepad, GAMEPAD_BUTTONS.RB);
        gamepadLB = new JoystickButton(gamepad, GAMEPAD_BUTTONS.LB);
        gamepadBack = new JoystickButton(gamepad, GAMEPAD_BUTTONS.BACK);
        gamepadStart = new JoystickButton(gamepad, GAMEPAD_BUTTONS.START);
        gamepadL3 = new JoystickButton(gamepad, GAMEPAD_BUTTONS.L3);
        gamepadLeftStickButton = new JoystickButton(gamepad, GAMEPAD_BUTTONS.L3);
        gamepadRightStickButton = new JoystickButton(gamepad, GAMEPAD_BUTTONS.R3);
        gamepadLT = new TriggerButton(gamepad, GAMEPAD_AXIS.leftTrigger);
        gamepadRT = new TriggerButton(gamepad, GAMEPAD_AXIS.rightTrigger);
        

        gamepadPOVDown = new PovButton(gamepad, GAMEPAD_POV.DOWN);
        gamepadPOVUp = new PovButton(gamepad, GAMEPAD_POV.UP);
    }
    
    public void initializeAutonCommand(Autons autonSelected) {

        if (autonSelected == Autons.NOTHING) {
            autonCommand = new DriveDistance(0.0, driveTrain);
        } else if (autonSelected == Autons.TAXI) {
            autonCommand = new DriveDistance(60.0, driveTrain);
        } else if (autonSelected == Autons.TWO_CARGO) {
            autonCommand = new ParallelCommandGroup(
                new RunCommand(() -> intakeArticulator.setIntakeOut(), intakeArticulator),
                new RunCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE)),
                new DriveDistance(60.0, driveTrain));
        } else if (autonSelected == Autons.FOUR_CARGO) {

            autonCommand = new SequentialCommandGroup( 
                new ParallelCommandGroup(
                    new InstantCommand(() -> intakeArticulator.setIntakeOut(), intakeArticulator),
                    new InstantCommand(() -> intake.setSpeed(IntakeSpeed.INTAKE)),
                    new CheckRobotEmpty(frontFeeder, backFeeder, shooter),
                    new SequentialCommandGroup(
                        new DriveDistance(60.0, driveTrain),
                        new MoveHeading(0, -12.7, driveTrain)
                    )
                ),
                new DriveDistance(150.0, driveTrain),
                new WaitCommand(1.5),
                new DriveDistance(-150.0, driveTrain)
                );
        }
    }

    public void updateAuton() {
        Autons currAuton = autonChooser.getSelected();
        if (currAuton != null && currAuton != currentSelectedAuton) {
            currentSelectedAuton = currAuton;
            initializeAutonCommand(currentSelectedAuton);
        }
        SmartDashboard.putString("Auton Chosen", currentSelectedAuton == null ? "Nothing" : currentSelectedAuton.toString());
    }

    public Autons getSelectedAuton() {
        return this.currentSelectedAuton;
    }

    public Command getAutonCommand(){
        return this.autonCommand;
    }

    public enum Autons {
        NOTHING,
        TAXI,
        TWO_CARGO,
        FOUR_CARGO
    }

    public Limelight getLimelight() {
        return this.limelight;
    }

}

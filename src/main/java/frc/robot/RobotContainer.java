package frc.robot;

import java.io.FileNotFoundException;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotMap.DS_USB;
import frc.robot.RobotMap.GAMEPAD_AXIS;
import frc.robot.RobotMap.GAMEPAD_BUTTONS;
import frc.robot.RobotMap.JOYSTICK_ADJUSTMENTS;
import frc.robot.RobotMap.JOYSTICK_BUTTONS;
import frc.robot.commands.drivetrain.DriveWithJoysticks;
import frc.robot.commands.drivetrain.MoveOnTrajectory;
import frc.robot.commands.drivetrain.DriveWithJoysticks.DriveType;
import frc.robot.commands.drivetrain.MoveHeadingDerivatives.DriveDistance;
import frc.robot.commands.drivetrain.MoveHeadingDerivatives.MoveHeading;
import frc.robot.commands.elevator.AutomaticElevator;
import frc.robot.commands.elevator.ManualElevator;
import frc.robot.commands.feeder.LoadFeeder;
import frc.robot.commands.feeder.LoadFeederTrigger;
import frc.robot.commands.feeder.ShootBall;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.limelightCamera.SwitchLEDState;
import frc.robot.commands.shooter.CalculateTargetRPM;
import frc.robot.commands.shooter.RunShooterRPM;
import frc.robot.commands.shooter.RunShooterRPMPID;
import frc.robot.commands.turret.RotateToTarget;
import frc.robot.commands.turret.ScanForTarget;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.Limelight.LimelightLEDState;
import frc.robot.sensors.REVColorMux.I2CMUX;
import frc.robot.sensors.REVColorMux.REVColor.ColorSensorID;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveTrainLayout;
import frc.robot.subsystems.DriveTrain.ShootingStyle;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Feeder.BallMatchesAlliance;
import frc.robot.subsystems.Feeder.BreakBeamDIO;
import frc.robot.subsystems.Feeder.FeedSpeed;
import frc.robot.subsystems.IntakeArticulator.IntakePosition;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArticulator;
import frc.robot.subsystems.LimelightCamera;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterMode;
import frc.robot.subsystems.Turret;
import frc.robot.util.ShuffleDash;
import frc.robot.util.TriggerButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class RobotContainer {
    public static final double DEADZONE = 0.08;

    private ShuffleDash shuffleDash;

    private Joystick rightJoystick, leftJoystick, gamepad;

    private JoystickButton left1, left2, left3, left4, left5, left6, left7, left8, left9, left10, left11;
    private JoystickButton right1, right2, right3, right4, right5, right6, right7, right8, right9, right10, right11;
    private JoystickButton gamepadA, gamepadB, gamepadX, gamepadY, gamepadRB, gamepadLB, gamepadL3, gamepadBack, gamepadStart, gamepadLeftStickButton, gamepadRightStickButton;
    private TriggerButton gamepadLT, gamepadRT;

    private DriveTrain driveTrain;
    private Shooter shooter;
    private Turret turret;
    private Intake intake;
    private IntakeArticulator intakeArticulator;
    private Feeder frontFeeder, backFeeder;
    private Elevator elevator;
    private LimelightCamera limelightCamera;

    private Limelight limelight;
    
    private Command autonCommand = null;

    public RobotContainer() {
        leftJoystick = new Joystick(DS_USB.LEFT_STICK);
        rightJoystick = new Joystick(DS_USB.RIGHT_STICK);
        gamepad = new Joystick(DS_USB.GAMEPAD);

        limelight = new Limelight();

        turret = new Turret(limelight);
        turret.setDefaultCommand(new ScanForTarget(turret));

        driveTrain = new DriveTrain(DriveTrainLayout.FALCONS); //make sure to switch it back to Falcons
        driveTrain.setDefaultCommand(new DriveWithJoysticks(DriveType.ARCADE, driveTrain));

        shooter = new Shooter(ShooterMode.ONE_WHEEL, limelight);
        //shooter.setDefaultCommand(new CalculateTargetRPM(shooter, turret));
        shooter.setDefaultCommand(new RunCommand(() -> shooter.stopShooter(), shooter));
        

        intake = new Intake();
        intakeArticulator = new IntakeArticulator();
        
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
        ball in front and no ball in back OR
        front feeder empty and IntakePosition.OUT
        */
        frontFeeder.setDefaultCommand(new LoadFeederTrigger(frontFeeder, () ->
        (frontFeeder.isBeamBroken() && !backFeeder.isBeamBroken()) ||
        (!frontFeeder.isBeamBroken() && intakeArticulator.getIntakePosition() == IntakePosition.OUT)));
        // frontFeeder.setDefaultCommand(new RunCommand(() -> frontFeeder.setSpeed(FeedSpeed.STOP), frontFeeder));
       
        /*
        no ball in back AND
        (IntakePosition.OUT or ball in front)
        --> run the back feeder
        */
        backFeeder = new Feeder(ColorSensorID.BACK, BreakBeamDIO.BACK, RobotMap.CAN.FEEDER_B, mux);
        backFeeder.setDefaultCommand(new LoadFeederTrigger(backFeeder, () -> !backFeeder.isBeamBroken() 
        && (intakeArticulator.getIntakePosition() == IntakePosition.OUT || frontFeeder.isBeamBroken())));
        // backFeeder.setDefaultCommand(new RunCommand(() -> backFeeder.setSpeed(FeedSpeed.STOP), backFeeder));

        intake = new Intake();
        limelightCamera = new LimelightCamera();
        limelightCamera.getLimelight().setLEDState(LimelightLEDState.OFF);
        elevator = new Elevator();
        elevator.setDefaultCommand(new ManualElevator(elevator, () -> getGamepadAxis(GAMEPAD_AXIS.leftY)));

        


        shuffleDash = new ShuffleDash(this);
       
        shuffleDash.addPublisher(shooter);
        shuffleDash.addPublisher(driveTrain);
        //shuffleDash.addPublisher(spinner);
        //shuffleDash.addPublisher(intake);
        //shuffleDash.addPublisher(limelightCamera);
        //shuffleDash.addPublisher(intakeArticulator);
        //shuffleDash.addPIDTunable(shooter, "Shooter");
        //shuffleDash.addPIDTunable(driveTrain, "DriveTrain");
    
        initializeJoystickButtons();

        //driver controls
        //toggle intake in and out
        left1.whenPressed(new ParallelCommandGroup(new RunCommand(() -> intakeArticulator.setIntakeOut(), intakeArticulator), new RunIntake(intake))); 
        left2.whenPressed(new ParallelCommandGroup(new RunCommand(() -> intake.stopIntakeRoller(), intake), 
                                                   new RunCommand(() -> intakeArticulator.setIntakeIn(), intakeArticulator)));
        left3.whenPressed(new ParallelCommandGroup(new RunCommand(() -> intakeArticulator.setIntakeOut(), intakeArticulator), 
                                                   new RunCommand(() -> intake.setRollerSpeed(-0.7 * intake.INTAKE_SPEED), intake)));
                                
        left4.toggleWhenPressed(new RunShooterRPMPID(shooter, limelight, ShootingStyle.LOWER_PORT));

        
        left6.whenPressed(new SwitchLEDState(limelightCamera));

        left7.whenPressed(new RunCommand(() -> turret.setPosition(-4096.0), turret));
        left8.whenPressed(new RunCommand(() -> turret.setPosition(0.0), turret));
        left9.whenPressed(new RunCommand(() -> turret.setPosition(4096.0), turret));

        left10.whenPressed(new ParallelCommandGroup(new RunCommand(() -> intakeArticulator.setIntakeDisabled(), intakeArticulator), new RunIntake(intake)));
        try {
            left11.whenPressed(new MoveOnTrajectory("Taxi-OneCargo", driveTrain));
            right11.whenPressed(new MoveOnTrajectory("TestCircle", driveTrain));
        } catch (FileNotFoundException ex) {
        }
        

        right4.whenPressed(new DriveWithJoysticks(DriveType.ARCADE, driveTrain));

        right6.whenPressed(new RotateToTarget(turret));


        // gamepadX.whenPressed(new RunCommand(() -> frontFeeder.setSpeed(FeedSpeed.SHOOT), frontFeeder));
        // gamepadY.whenPressed(new RunCommand(() -> backFeeder.setSpeed(FeedSpeed.SHOOT), backFeeder));

        gamepadX.whenPressed(new ParallelCommandGroup(new RunCommand(() -> frontFeeder.setSpeed(FeedSpeed.SHOOT), frontFeeder),
        new RunCommand(() -> backFeeder.setSpeed(FeedSpeed.SHOOT), backFeeder)));
        gamepadY.whenPressed(new ParallelCommandGroup(new RunCommand(() -> frontFeeder.setSpeed(FeedSpeed.STOP), frontFeeder),
        new RunCommand(() -> backFeeder.setSpeed(FeedSpeed.STOP), backFeeder)));
        
       
        right11.whenPressed(new RunCommand(() -> frontFeeder.setSpeed(FeedSpeed.EJECT), frontFeeder));


        gamepadA.whenPressed(new RunShooterRPM(shooter));
        gamepadB.whenPressed(new RunCommand(() -> shooter.stopShooter(), shooter));
        

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

        
        
        right7.whenPressed(new MoveHeading(120, 0, driveTrain));
        right8.whenPressed(new MoveHeading(-120, 0, driveTrain));
        right9.whenPressed(new MoveHeading(120, 90, driveTrain));
        right10.whenPressed(new DriveDistance(120, driveTrain));
        
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

        Trigger shootBall = new Trigger(() -> turret.isOnTarget() && shooter.isReadyToShoot() && backFeeder.isBeamBroken());
        shootBall.whileActiveContinuous(new ShootBall(backFeeder, shooter));
        

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

    public void updateDash() {
        shuffleDash.updateDash();
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
    }
    
    public void initializeAutonCommand() {
        ShuffleDash.Autons auton = shuffleDash.getAuton();
        if (auton == null) return;
        switch (shuffleDash.getAuton()){
            case NOTHING:
                autonCommand = null;
                break;
            case TAXI:
                autonCommand = new DriveDistance(90.0, driveTrain);
                break;
            case ONE_CARGO:
                try {
                autonCommand = new ParallelCommandGroup(
                    new RunCommand(() -> intakeArticulator.setIntakeOut(), intakeArticulator),
                    new RunIntake(intake),
                    new MoveOnTrajectory("Taxi-OneCargo", driveTrain));
                 } catch (FileNotFoundException err) {}

                // autonCommand = new ParallelCommandGroup(
                //     new RunCommand(() -> intakeArticulator.setIntakeOut(), intakeArticulator),
                //     new RunIntake(intake),
                //     new MoveHeading(90.0, 45, driveTrain));
                break;
            case TWO_CARGO:
                try {
                    autonCommand = new ParallelCommandGroup(new RunCommand(() -> intakeArticulator.setIntakeOut(), intakeArticulator),
                    new RunIntake(intake),
                    new MoveOnTrajectory("Taxi-TwoCargo", driveTrain));
                   // new DriveDistance(72.0, driveTrain));
                } catch (FileNotFoundException err) {}
                break;
            default:
                autonCommand = new DriveDistance(72.0, driveTrain);
                break;   
        }
    }

    public Command getAutonCommand(){
        return this.autonCommand;
    }

    public LimelightCamera getLimelightCamera() {
        return this.limelightCamera;
    }

    public Elevator getElevator() {
        return this.elevator;
    }
}

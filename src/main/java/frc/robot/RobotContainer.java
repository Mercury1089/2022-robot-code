package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.RobotMap.DS_USB;
import frc.robot.RobotMap.GAMEPAD_AXIS;
import frc.robot.RobotMap.GAMEPAD_BUTTONS;
import frc.robot.RobotMap.JOYSTICK_BUTTONS;
import frc.robot.RobotMap.JOYSTICK_ADJUSTMENTS;
import frc.robot.commands.drivetrain.DriveWithJoysticks;
import frc.robot.commands.drivetrain.DriveWithJoysticks.DriveType;
import frc.robot.commands.drivetrain.MoveHeadingDerivatives.DriveDistance;
import frc.robot.commands.elevator.AutomaticElevator;
import frc.robot.commands.elevator.ManualElevator;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.limelightCamera.SwitchLEDState;
import frc.robot.commands.shooter.EndFullyAutoAimBot;
import frc.robot.commands.shooter.FullyAutoAimbot;
import frc.robot.commands.shooter.RunShooterRPMPID;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.Limelight.LimelightLEDState;
import frc.robot.sensors.REVColor.CargoColor;
import frc.robot.sensors.REVColor.ColorSensorPort;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveTrainLayout;
import frc.robot.subsystems.DriveTrain.ShootingStyle;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArticulator;
import frc.robot.subsystems.LimelightCamera;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter.ShooterMode;
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
    private Feeder feeder, feeder2;
    private Hopper hopper;
    private Elevator elevator;
    private LimelightCamera limelightCamera;

    private Limelight limelight;
    
    private CommandGroupBase autonCommand = null;

    public RobotContainer() {
        leftJoystick = new Joystick(DS_USB.LEFT_STICK);
        rightJoystick = new Joystick(DS_USB.RIGHT_STICK);
        gamepad = new Joystick(DS_USB.GAMEPAD);

        limelight = new Limelight();

        driveTrain = new DriveTrain(DriveTrainLayout.FALCONS, limelight); //make sure to switch it back to Falcons
        driveTrain.setDefaultCommand(new DriveWithJoysticks(DriveType.ARCADE, driveTrain));

        shooter = new Shooter(ShooterMode.NONE, limelight);
        shooter.setDefaultCommand(new RunShooterRPMPID(shooter, limelight, ShootingStyle.MANUAL));
        
        hopper = new Hopper(); 
        //hopper.setDefaultCommand(new RunCommand(() -> hopper.setSpeed(0.0), hopper));


        intake = new Intake();
        intakeArticulator = new IntakeArticulator();
        //feeder = new Feeder();
        feeder = new Feeder(ColorSensorPort.FRONT_SENSOR, CargoColor.BLUE);
        feeder2 = new Feeder(ColorSensorPort.BACK_SENSOR, CargoColor.BLUE);
        intake = new Intake();
        limelightCamera = new LimelightCamera();
        limelightCamera.getLimelight().setLEDState(LimelightLEDState.OFF);
        elevator = new Elevator();
        elevator.setDefaultCommand(new ManualElevator(elevator, () -> getGamepadAxis(GAMEPAD_AXIS.leftY)));

        turret = new Turret();
        turret.setDefaultCommand(new RunCommand(() -> turret.setSpeed(() ->getGamepadAxis(GAMEPAD_AXIS.leftX)), turret));

        
        shuffleDash = new ShuffleDash();
        shuffleDash.addPublisher(shooter);
        shuffleDash.addPublisher(driveTrain);
        //shuffleDash.addPublisher(spinner);
        //shuffleDash.addPublisher(intake);
        shuffleDash.addPublisher(limelightCamera);
        //shuffleDash.addPublisher(intakeArticulator);
        shuffleDash.addPublisher(feeder);
        shuffleDash.addPublisher(feeder2);
        //shuffleDash.addPublisher(hopper);
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

        right2.whenPressed(new EndFullyAutoAimBot(driveTrain, feeder, hopper, shooter));
        right4.whenPressed(new DriveWithJoysticks(DriveType.ARCADE, driveTrain));

        right10.whenPressed(new DriveDistance(-24.0, driveTrain));

        //Operator controls
        gamepadRB.whenPressed(new FullyAutoAimbot(driveTrain, shooter, feeder, hopper, intake, limelight, ShootingStyle.AUTOMATIC)); //end fully auto aimbot
        gamepadLT.whenPressed(new FullyAutoAimbot(driveTrain, shooter, feeder, hopper, intake, limelight, ShootingStyle.LOWER_PORT)); //run shooter in manual mode
        gamepadRT.whenPressed(new FullyAutoAimbot(driveTrain, shooter, feeder, hopper, intake, limelight, ShootingStyle.AUTOMATIC)); //rek the opponents

        gamepadA.whenPressed(new AutomaticElevator(elevator, Elevator.ElevatorPosition.BOTTOM));
        gamepadY.whenPressed(new AutomaticElevator(elevator, Elevator.ElevatorPosition.READY, false));
        gamepadL3.whenPressed(new ManualElevator(elevator, () -> getGamepadAxis(GAMEPAD_AXIS.leftY)));
        gamepadStart.and(gamepadBack).whenActive(new ParallelCommandGroup(new SequentialCommandGroup(new InstantCommand(() -> elevator.setLockEngaged(true), elevator),
                                                                                                     new AutomaticElevator(elevator, Elevator.ElevatorPosition.HANG),
                                                                                                     new ManualElevator(elevator, () -> getGamepadAxis(GAMEPAD_AXIS.leftY))),
                                                 new RunCommand(() -> shooter.stopShooter(), shooter))); //lock the elevator
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
        //shuffleDash.updateDash();
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
        if(autonCommand == null)
            autonCommand = new SequentialCommandGroup(
                new DriveDistance(-24.0, driveTrain),
                new FullyAutoAimbot(driveTrain, shooter, feeder, hopper, intake, limelight, ShootingStyle.AUTOMATIC)
            );
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

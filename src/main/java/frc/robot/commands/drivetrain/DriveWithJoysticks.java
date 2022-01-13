package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap.DS_USB;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.*;

/**
 * Command that puts the drive train into a manual control mode.
 * This puts the robot in arcade drive.
 */
public class DriveWithJoysticks extends CommandBase{
    private DriveAssist tDrive;
    //private DelayableLogger everySecond = new DelayableLogger(log, 10, TimeUnit.SECONDS);
    private DriveType driveType;

    private DriveTrain driveTrain;

    public DriveWithJoysticks(DriveType type, DriveTrain driveTrain) {
        super();
        super.addRequirements(driveTrain);
        setName("DriveWithJoysticks");
        this.driveTrain = driveTrain;
        driveType = type;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        this.driveTrain.configVoltage(DriveTrain.NOMINAL_OUT, DriveTrain.PEAK_OUT);
        tDrive = this.driveTrain.getDriveAssist();
        this.driveTrain.setNeutralMode(NeutralMode.Brake);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        if (tDrive != null) {
            switch (driveType) {
                case TANK:
                    tDrive.tankDrive(Robot.robotContainer.getJoystickY(DS_USB.LEFT_STICK), Robot.robotContainer.getJoystickY(DS_USB.RIGHT_STICK));
                    break;
                case ARCADE:
                    tDrive.arcadeDrive(-Robot.robotContainer.getJoystickY(DS_USB.LEFT_STICK), Robot.robotContainer.getJoystickX(DS_USB.RIGHT_STICK), true);
                    break;
            }
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        this.driveTrain.setNeutralMode(NeutralMode.Brake);
        this.driveTrain.stop();
    }

    public enum DriveType {
        TANK,
        ARCADE
    }
}

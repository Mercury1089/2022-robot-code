package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.limelightCamera.SetLEDState;
import frc.robot.sensors.Limelight.LimelightLEDState;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 * <p>
 * GUYS, WE FOUND THE ROBOT
 */
public class Robot extends TimedRobot {

    public static RobotContainer robotContainer;

    public static boolean isInTestMode = false;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        CommandScheduler.getInstance().enable();
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        
        (new SetLEDState(robotContainer.getLimelightCamera(), LimelightLEDState.OFF)).schedule();
    }

    @Override
    public void disabledPeriodic() {
        robotContainer.updateAutonChooser();
    }

    @Override
    public void autonomousInit() {
        Command autonCommand = robotContainer.getAutonCommand();
        
        (new SetLEDState(robotContainer.getLimelightCamera(), LimelightLEDState.ON)).schedule();
        if (autonCommand != null){
            autonCommand.schedule();
            DriverStation.reportError("Auton is Scheduled", false);
        }
        DriverStation.reportError("Auton is initialized", false);
        
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        (new SetLEDState(robotContainer.getLimelightCamera(), LimelightLEDState.ON)).schedule();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        isInTestMode = true;
    }

    @Override
    public void testPeriodic() {
        super.testInit();
    }
}

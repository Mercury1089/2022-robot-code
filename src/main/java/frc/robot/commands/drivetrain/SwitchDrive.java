/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.DriveAssist.DriveDirection;

public class SwitchDrive extends CommandBase {
    private DriveDirection dd;

    private DriveTrain driveTrain;

    public SwitchDrive(DriveDirection driveDir, DriveTrain driveTrain) {
        dd = driveDir;
        setName("SwitchDrive");

        this.driveTrain = driveTrain;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        this.driveTrain.setDirection(dd);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }
}

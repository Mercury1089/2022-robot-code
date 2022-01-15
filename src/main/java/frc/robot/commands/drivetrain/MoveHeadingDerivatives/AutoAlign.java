
package frc.robot.commands.drivetrain.MoveHeadingDerivatives;

import frc.robot.subsystems.DriveTrain;
import frc.robot.util.MercMath;

public class AutoAlign extends MoveHeading {
    private final double ANGLE_THRESHOLD_DEGREE = 1.0;

    private DriveTrain driveTrain;
    private double angleError;

    public AutoAlign(DriveTrain driveTrain) {
        super(0, 0, driveTrain);

        this.setName("AutoAlign");
        this.driveTrain = driveTrain;

        moveThresholdTicks = 100;
        onTargetMinCount = 3;
    }

   // Called just before this Command runs the first time
   @Override   
   public void initialize() {
        targetHeading = driveTrain.getLimelight().getTargetCenterXAngle();
        System.out.println("AutoAlign initialized with angle: " + targetHeading);
        targetHeading = -1.0 * MercMath.degreesToPigeonUnits(targetHeading);
        super.initialize();
        this.driveTrain.configPIDSlots(DriveTrain.DRIVE_PID_SLOT, DriveTrain.DRIVE_SMOOTH_TURN_SLOT);

   }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        super.execute();
    }
    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        angleError = driveTrain.getAngleError();
        angleError = MercMath.pigeonUnitsToDegrees(angleError);

        boolean isFinished = false;
        boolean isOnTarget = (Math.abs(angleError) < ANGLE_THRESHOLD_DEGREE);

        if (isOnTarget) {
            onTargetCount++;
        } else {
            if (onTargetCount > 0)
                onTargetCount = 0;
        }

        if (onTargetCount > onTargetMinCount) {
            isFinished = true;
            onTargetCount = 0;
        }

        return isFinished;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}

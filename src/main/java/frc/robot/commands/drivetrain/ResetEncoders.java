package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/**
 * https://youtu.be/NnP5iDKwuwk
 */
public class ResetEncoders extends CommandBase   {

    private DriveTrain driveTrain;
    private boolean resetPigeon;
    private int i;
    
    public ResetEncoders(DriveTrain driveTrain) {
        super.addRequirements(driveTrain);
        this.driveTrain = driveTrain;
    }

    public ResetEncoders(DriveTrain driveTrain, boolean resetPigeon) {
        super.addRequirements(driveTrain);
        this.resetPigeon = resetPigeon;
        this.driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        i = 0;
    }

    @Override
    public void execute() {
        if (i % 5 == 0) {
            driveTrain.resetEncoders();
            if (resetPigeon) 
                driveTrain.resetPigeonYaw();
        }
        i++;
    }

    @Override
    public boolean isFinished(){
        if (resetPigeon) 
            return driveTrain.getPigeonYaw() == 0.0 && driveTrain.getLeftEncPositionInTicks() == 0.0 && driveTrain.getRightEncPositionInTicks() == 0.0;
        return driveTrain.getLeftEncPositionInTicks() == 0.0 && driveTrain.getRightEncPositionInTicks() == 0.0;
    }
}
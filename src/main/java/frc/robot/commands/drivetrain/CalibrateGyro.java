package frc.robot.commands.drivetrain;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class CalibrateGyro extends CommandBase {

    private DriveTrain driveTrain;

    public CalibrateGyro(DriveTrain driveTrain){
        this.driveTrain = driveTrain;
        setName("CalibrateGyro");
    }

    
    public void initialize() {
        //  System.out.println("Calibrating gyro...");
        this.driveTrain.getPigeon().enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
}

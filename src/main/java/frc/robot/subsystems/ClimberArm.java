package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.interfaces.IMercShuffleBoardPublisher;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.RobotMap.CAN;

public class ClimberArm extends SubsystemBase {
    private final TalonSRX climberArm;
    private IntakeArticulator intakeArticulator;
    private final double OUT_SPEED = 0.5, IN_SPEED = -0.6;


    
    public ClimberArm() {
        super();
        climberArm = new TalonSRX(CAN.INTAKE_ARTICULATOR);
    }
    public void setClimberArmForward() {

    }

}

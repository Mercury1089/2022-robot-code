package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.RobotMap.CAN;

public class ClimberArm extends SubsystemBase {
    private final TalonSRX climberArmArticulator;
    private ClimberPosition climberArmPosition;
    private final double BACKWARD_SPEED = -0.5, FORWARD_SPEED = 0.5;
    
    public ClimberArm() { //unsure about this constructor
        super();
        climberArmArticulator = new TalonSRX(CAN.CLIMBER_ARM_ARTICULATOR); 
        climberArmArticulator.configFactoryDefault();                 
        climberArmArticulator.setNeutralMode(NeutralMode.Brake);
        climberArmArticulator.setInverted(true);
        climberArmPosition = ClimberPosition.BACKWARD;                  
    }

    public enum ClimberPosition{
        FORWARD,
        BACKWARD,
        DISABLED
    }
    
    public void setClimberArmForward() {
        this.climberArmPosition = ClimberPosition.FORWARD;
        climberArmArticulator.set(ControlMode.PercentOutput, FORWARD_SPEED);
    }
    
    public void setClimberArmBackward() {
        this.climberArmPosition = ClimberPosition.BACKWARD;
        climberArmArticulator.set(ControlMode.PercentOutput, BACKWARD_SPEED);
    }
    
    public void setClimberArmDisabled() {
        this.climberArmPosition = ClimberPosition.DISABLED;
        climberArmArticulator.set(ControlMode.PercentOutput, 0.0);
    }
    
    public ClimberPosition getClimberPosition() {
        return this.climberArmPosition;
    }

}

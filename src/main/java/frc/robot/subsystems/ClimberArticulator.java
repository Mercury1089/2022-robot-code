package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;

import frc.robot.RobotMap;
import frc.robot.RobotMap.CAN;

public class ClimberArticulator extends SubsystemBase {
    private final TalonSRX climberArticulator;
    private ClimberPosition climberArmPosition;
    private final double BACKWARD_SPEED = -0.5, FORWARD_SPEED = 0.5;
    private boolean isLocked;
    
    public ClimberArticulator() { //unsure about this constructor
        super();
        climberArticulator = new TalonSRX(CAN.CLIMBER_ARM_ARTICULATOR); 
        climberArticulator.configFactoryDefault();                 
        climberArticulator.setNeutralMode(NeutralMode.Brake);
        climberArticulator.setInverted(true);
        // No sensor feedback is required, so Status 2 frequency can be extra low.
        climberArticulator.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, RobotMap.CAN_STATUS_FREQ.XTRA_LOW);
        climberArmPosition = ClimberPosition.BACKWARD;  
        
        isLocked = true;
    }

    public enum ClimberPosition{
        FORWARD,
        BACKWARD,
        DISABLED
    }
    
    public void setClimberArmForward() {
        this.climberArmPosition = ClimberPosition.FORWARD;
        climberArticulator.set(ControlMode.PercentOutput, FORWARD_SPEED);
    }
    
    public void setClimberArmBackward() {
        this.climberArmPosition = ClimberPosition.BACKWARD;
        climberArticulator.set(ControlMode.PercentOutput, BACKWARD_SPEED);
    }
    
    public void setClimberArmDisabled() {
        this.climberArmPosition = ClimberPosition.DISABLED;
        climberArticulator.set(ControlMode.PercentOutput, 0.0);
    }
    
    public ClimberPosition getClimberPosition() {
        return this.climberArmPosition;
    }

    public void setSpeed(Supplier<Double> speed) {
        if (!this.isLocked) {
            climberArticulator.set(ControlMode.PercentOutput, speed.get());
        }
    }

    

    public void setIsLocked(boolean locked) {
        this.isLocked = locked;
    }

    public boolean getIsLocked() {
        return this.isLocked;
    }

    
  @Override
  public void initSendable(SendableBuilder builder) {
    
    builder.setActuator(true); // Only allow setting values when in Test mode
    builder.addBooleanProperty("isLocked", () -> getIsLocked(), null);


    
  }

}

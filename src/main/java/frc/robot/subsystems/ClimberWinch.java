package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.RobotMap.CAN;

public class ClimberWinch extends SubsystemBase{
    private final TalonFX climberWinchArticulator;
    private ClimberWinchStatus climberWinch;
    private final double CLIMBER_WINCH_SPEED = 0.5;

    public ClimberWinch() {
        super();
        climberWinchArticulator = new TalonFX(CAN.CLIMBER_WINCH);
        climberWinch = ClimberWinchStatus.DISABLED;
    }

    public enum ClimberWinchStatus {
        ACTIVE,
        DISABLED
    }

    public void setClimberWinchSpeedActive() {
        this.climberWinch = ClimberWinchStatus.ACTIVE;
        climberWinchArticulator.set(ControlMode.PercentOutput,CLIMBER_WINCH_SPEED);
    }

    public void setClimberWinchSpeedDisabled() {
        this.climberWinch = ClimberWinchStatus.DISABLED;
        climberWinchArticulator.set(ControlMode.PercentOutput,0.0);
    }

    public ClimberWinchStatus getClimberWinchSpeed() {
        return this.climberWinch; //should be "return CLIMBER_WINCH_SPEED;"????? 
    }

}


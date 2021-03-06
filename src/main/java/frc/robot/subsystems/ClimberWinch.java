package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;

import frc.robot.RobotMap;
import frc.robot.RobotMap.CAN;

public class ClimberWinch extends SubsystemBase{
    private final TalonFX climberWinch;
    private double CLIMBER_WINCH_SPEED = 0.5;
    private boolean isLocked;

    public ClimberWinch() {
        super();
        climberWinch = new TalonFX(CAN.CLIMBER_WINCH);
        climberWinch.configFactoryDefault();
        climberWinch.setNeutralMode(NeutralMode.Brake);
        // No sensor feedback is required, so Status 2 frequency can be extra low.
        climberWinch.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, RobotMap.CAN_STATUS_FREQ.XTRA_LOW);

        isLocked = true;
    }

    

    public void setClimberWinchSpeedActive() {
        climberWinch.set(ControlMode.PercentOutput,CLIMBER_WINCH_SPEED);
    }

    public void setClimberWinchSpeedDisabled() {
        CLIMBER_WINCH_SPEED = 0.0;
        climberWinch.set(ControlMode.PercentOutput,CLIMBER_WINCH_SPEED);
    }


    public void setSpeed(Supplier<Double> speed) {
        if (!this.isLocked) {
            climberWinch.set(ControlMode.PercentOutput, speed.get());
        }
    }

    public void setIsLocked(boolean locked) {
        this.isLocked = locked;
    }

    public boolean getIsLocked() {
        return this.isLocked;
    }

}


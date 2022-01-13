package frc.robot.util;

public class PIDGain {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;
    public final double clMaxOut;

    public PIDGain(double _kP, double _kI, double _kD, double _kF) {
        kP = _kP;
        kI = _kI;
        kD = _kD;
        kF = _kF;
        clMaxOut = 1.0;
    }

    public PIDGain(double _kP, double _kI, double _kD, double _kF, double closedLoopMaxOutput) {
        kP = _kP;
        kI = _kI;
        kD = _kD;
        kF = _kF;
        clMaxOut = closedLoopMaxOutput;
    }
}
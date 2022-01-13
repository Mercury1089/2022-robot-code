package frc.robot.util.interfaces;

public interface IMercPidTuner {

    //Getting 4 kPIDF values

    public double getkP();

    public double getkD();

    public double getkI();

    public double getFeedForward();

    //Setting 4 kPIDF values

    public void setkP();

    public void setkD();

    public void setkI();

    public void setFeedForward();

}
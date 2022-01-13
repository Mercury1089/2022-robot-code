package frc.robot.sensors;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.sensors.pixy.Pixy;
import frc.robot.sensors.pixy.SPIPixyLink;

public class BallCounter {

    private static final int NUM_SIGNATURES = 5;
    private static final int MAX_BLOCKS = 50;
    public Pixy pixyCam;

    public BallCounter() {
        pixyCam = new Pixy(new SPIPixyLink(SPI.Port.kOnboardCS0), NUM_SIGNATURES, MAX_BLOCKS);
    }

    public int getCount() {
        int ballCount = 0;
        for (int i = 1; i <= NUM_SIGNATURES; i++) {
            if (pixyCam.getBoxes(i).size() == 0) {
                // If the boxes array for the signature is empty, the target is covered by a ball.
                ballCount++;
            }
        }
        return ballCount;
    }

}
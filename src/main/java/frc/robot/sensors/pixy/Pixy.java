package frc.robot.sensors.pixy;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.BoundingBox;
import frc.robot.util.MercMath;

import java.util.ArrayList;
import java.util.Hashtable;


/**
 * RoboRio implementation for V1 PixyCam. Supports I2C or SPI.
 * Once instantiated, this class will obtain updated data from the Pixy every 50 ms.
 * Box data is stored for a configurable number of signatures (set at construction time)
 * Signatures can be retrieved by ID (1-8) if available
 */
public class Pixy {

    /**
     * Indicates either an SPI or I2C Pixy connection
     */
    public enum LinkType {
        SPI, I2C;
    }

    // Variables used for SPI comms, derived from https://github.com/omwah/pixy_rpi
    private static final byte PIXY_SYNC_BYTE = 0x5a;
    private static final byte PIXY_SYNC_BYTE_DATA = 0x5b;
    private static final int PIXY_OUTBUF_SIZE = 6;
    private static final int PIXY_START_WORD = 0xaa55;
    private static final int PIXY_START_WORDX = 0x55aa;
    private static final int BLOCK_LEN = 5;

    private static final double PIXY_UPDATE_PERIOD_SECONDS = 0.050; // Update every 50ms.
    private static final int PIXY_MAX_BOXES = 1000;

    private final Hashtable<Integer, ArrayList<BoundingBox>> signatures;
    private final Notifier pixyUpdateNotifier;

    private IPixyLink pixyLink;
    private int maxBoxes;
    private int numSignatures;

    private boolean skipStart = false;
    private boolean debug = false; // set true to log to console
    private long getStart = 0;
  
    /**
     * Create a new Pixy for the specified link type
     * @param linkType Communications link to the Pixy (I2C or SPI)
     * @param numSignatures Number of signatures to retrieve and store (1-8)
     * @param maxBoxes
     */
    public Pixy(LinkType linkType, int numSignatures, int maxBoxes) {
        this(linkType == LinkType.I2C ? new I2CPixyLink() :
             linkType == LinkType.SPI ? new SPIPixyLink() :
             new SPIPixyLink(), // Default to SPI (even though there are only two options currently)
             numSignatures, maxBoxes);
    }
    /**
     * Instantiate new Pixy
     * @param pixyLink The IPixyLink (SPI or I2C) of the pixy
     * @param numSignatures The number of signature slots in use on the Pixy
     * @param maxBoxes Maximum number of blocks (configured in Pixy Blocks tab)
     */
    public Pixy(IPixyLink pixyLink, int numSignatures, int maxBoxes) {

        this.pixyLink = pixyLink;
        this.numSignatures = MercMath.clamp(numSignatures, 1, 8);
        this.maxBoxes = MercMath.clamp(maxBoxes, 1, PIXY_MAX_BOXES);

        signatures = new Hashtable<Integer, ArrayList<BoundingBox>>();
        for(int i = 1; i <= maxBoxes; i++) {
            signatures.put(i, new ArrayList<BoundingBox>());
        }

        pixyUpdateNotifier = new Notifier(this::getSignatureBoxes);
        pixyUpdateNotifier.startPeriodic(PIXY_UPDATE_PERIOD_SECONDS);
    }

    /**
     * Return the boxes for the specified signature
     * @param signum Signature to get boxes for (1-8)
     * @return Arraylist containing 0 or more boxes detected by Pixy
     */
    public ArrayList<BoundingBox> getBoxes(int signum) {
        if (signum > 0 && signum <= numSignatures) {
            return signatures.get(signum);
        } else {
            return null;
        }
    }

    /**
     * Reads from SPI for data "words," and parses
     * all words into bounding boxes.
     */
    private void getSignatureBoxes() {
        if (debug) System.out.println("getSignatureBoxes(): Entering.");
        long count = 0;
        boolean loading = true;

        // If we haven't found the start of a block, find it.
        if (!skipStart) {
            // If we can't find the start of a block, drop out.
            if (!getStart()) {
                loading = false;
                if (debug) System.out.println("getSignatureBoxes(): getStart() got nothing.");
            }
        } else {
            // Clear flag that tells us to find the next block as the logic below will loop
            // the appropriate number of times to retrieve a complete block.
            skipStart = false;
        }

        // Loop until we hit the maximum number of blocks.
        while (loading && count < maxBoxes) {
            if (count == 0) {
                // Beginning of loading - clear the previous signatures.
                for(int i = 1; i <= maxBoxes; i++) {
                    signatures.get(i).clear();
                }
            }

            // Since this is our first time in, bytes 2 and 3 are the checksum, grab them and store for future use.
            // NOTE: getWord grabs the entire 16 bits in one shot.
            int checksum = pixyLink.getWord();
            int trialsum = 0;

            // See if the checksum is really the beginning of the next block,
            // in which case return the current set of BOXES found and set the flag
            // to skip looking for the beginning of the next block since we already found it.
            if (checksum == PIXY_START_WORD) {
                if (debug) System.out.println("getSignatureBoxes(): checksum is start word.");
                skipStart = true;
                loading = false;
            }
            // See if we received a empty buffer, if so, assume end of comms for now and return what we have.
            else if (checksum == 0) {
                if (debug) System.out.println("getSignatureBoxes(): empty buffer");
                loading = false;
            }

            if (loading) {
                // Start constructing BOXES
                // Only need 5 slots since the first 3 slots, the double start BOXES and checksum, have been retrieved already.
                int[] box = new int[5];
                for (int i = 0; i < BLOCK_LEN; i++) {
                    box[i] = pixyLink.getWord();
                    trialsum += box[i];
                }

                // See if we received the data correctly.
                // Also make sure the target is for the first signature
                int signum = box[0];
                if (checksum == trialsum && signum <= numSignatures) {
                    if (debug) System.out.println("getSignatureBoxes: count: " + count
                                    + " box[0]: " + box[0]
                                    + " box[1]: " + box[1]
                                    + " box[2]: " + box[2]
                                    + " box[3]: " + box[3]
                                    + " box[4]: " + box[4]);

                    BoundingBox bound = new BoundingBox(
                        box[1], // X
                        box[2], // Y
                        box[3], // W
                        box[4]  // H
                    );
                    // Data has been validated, add the current block of data to the overall BOXES buffer.
                    signatures.get(signum).add(bound);
                    count++;
                }

                // Check the next word from the Pixy to confirm it's the start of the next block.
                // Pixy sends two aa55 words at start of block, this should pull the first one.
                // The top of the loop should pull the other one.
                int w = pixyLink.getWord();

                if (w != PIXY_START_WORD) {
                    if (debug) System.out.println("getSignatureBoxes(): Not start word: " + w);
                    loading = false;
                } else {
                    if (debug) System.out.println("getSignatureBoxes(): Got start word: " + w);
                }
            }
        }

        // Sort arrays before returning
        for(int i = 1; i <= maxBoxes; i++) {
            signatures.get(i).sort(BoundingBox::compareTo);
        }
        // Should never get here, but if we happen to get a massive number of BOXES
        // and exceed the limit it will happen. In that case something is wrong
        // or you have a super natural Pixy and SPI link.
        //DriverStation.reportWarning("PIXY: Massive number of boxes!", false);
        if (debug) System.out.println("getSignatureBoxes(): LEaving.");

    }

    private boolean getStart() {
        int lastw = 0xff;
        int count = 0;

        // Loop until we get a start word from the Pixy.
        while (true) {
            int w = pixyLink.getWord();
            if (debug) System.out.println("getStart: count: " + count++);
    
            if ((w == 0x00) && (lastw == 0x00))
                // Could delay a bit to give time for next data block, but to get accurate time would tie up cpu.
                // So might as well return and let caller call this getStart again.
                return false;
            else if ((int) w == PIXY_START_WORD && (int) lastw == PIXY_START_WORD)
                return true;

            lastw = w;
        }
    }
}


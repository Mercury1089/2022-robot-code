package frc.robot.sensors.pixy;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayDeque;

/**
 * Pixy comms implementation using SPI interface
 */
public class SPIPixyLink implements IPixyLink {

    private SPI spi;

    // Variables used for SPI comms, derived from https://github.com/omwah/pixy_rpi
    private static final byte PIXY_SYNC_BYTE = 0x5a;
    private static final byte PIXY_SYNC_BYTE_DATA = 0x5b;
    private static final int PIXY_SPI_CLOCKRATE = 2000;

    private ArrayDeque<Byte> outBuf = new ArrayDeque<>(); // Future use for sending commands to Pixy.

    private boolean debug = false;

    public SPIPixyLink() {
        this(SPI.Port.kOnboardCS0);
    }

    /**
     * Instantiate new PixySPI link
     * @param port SPI port Pixy is connected
     */
    public SPIPixyLink(SPI.Port port) {
        spi = new SPI(port);
        // Set some SPI parameters.
        spi.setClockRate(PIXY_SPI_CLOCKRATE);
        spi.setMSBFirst();
        spi.setSampleDataOnTrailingEdge();
        spi.setClockActiveLow();
        spi.setChipSelectActiveLow();
    }

    // region Comms
    // Pixy SPI comm functions derived from https://docs.pixycam.com/wiki/doku.php?id=wiki:v1:porting_guide
    public int getWord() {
        int word = 0x00;

        ByteBuffer writeBuf = ByteBuffer.allocateDirect(2);
        writeBuf.order(ByteOrder.BIG_ENDIAN);
        ByteBuffer readBuf = ByteBuffer.allocateDirect(2);
        readBuf.order(ByteOrder.BIG_ENDIAN);

        if (outBuf.size() > 0)
            writeBuf.put(PIXY_SYNC_BYTE_DATA);
        else
            writeBuf.put(PIXY_SYNC_BYTE);

        // Flip the writeBuf so it's ready to be read.
        writeBuf.flip();

        // Send the sync / data bit / 0 to get the Pixy to return data appropriately.
        spi.transaction(writeBuf, readBuf, 2);

        if (debug) System.out.println("Pixy: getWord: read sync: " + bbToString(readBuf));

        // Set the position back to 0 in the buffer so we read it from the beginning next time.
        readBuf.rewind();

        // Store the contents of the buffer in a int that will be returned to the caller.
        word = (int) (readBuf.getShort() & 0xffff);

        // Clear the buffers, not needed, but nice to know they are cleaned out.
        writeBuf.clear();
        readBuf.clear();
        return (word);
    }

    // Debugging functions
	final protected static char[] hexArray = "0123456789ABCDEF".toCharArray();
    public static String bytesToHex(byte[] bytes) {
		char[] hexChars = new char[bytes.length * 2];
		for ( int j = 0; j < bytes.length; j++ ) {
			int v = bytes[j] & 0xFF;
			hexChars[j * 2] = hexArray[v >>> 4];
			hexChars[j * 2 + 1] = hexArray[v & 0x0F];
		}
		return new String(hexChars);
	}
	public String bbToString(ByteBuffer bb) {
		final byte[] b = new byte[bb.remaining()];
		bb.duplicate().get(b);
		bb.rewind();
		return new String(bytesToHex(b));
    }
    
    // Need to come back to this one, used only for send control data to Pixy.
    public int send(byte data) {
        return (0);
    }
}


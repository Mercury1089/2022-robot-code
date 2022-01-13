/*package frc.robot.commands;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.command.Command;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import frc.robot.Robot;
import frc.robot.subsystems.PDP;

import java.io.*;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

/**
 * Logs various information about the PDP to a CSV file
 * on the rio. Logs are done every second.
 * 
 * This helps to debug info such as brownouts as it prints
 * info about power, voltage, amps, current, etc.
 *
 */
/*public class LogPDPInfo extends Command {
    private Logger log = LogManager.getLogger(LogPDPInfo.class);
	private static final DateTimeFormatter fileTimestampFormat = DateTimeFormatter.ofPattern("MMdd_HHmmss");
	private static final DateTimeFormatter statusTimestampFormat = DateTimeFormatter.ofPattern("HH:mm:ss.SSS a");
	private static final double MILLIS_BETWEEN_LOGS = 1_000;
	private static final String CSV_HEADER;

	static {
		StringBuilder sb = new StringBuilder("Time,TimeMillis,TotalCurrent,Temperature,TotalEnergy,TotalPower,Voltage");
		for(int i = 0; i < 16; i++) {
			sb.append(",Channel").append(i);
		}
		CSV_HEADER = sb.toString();
	}

	private static String getCSVStatusLine(PowerDistributionPanel panel) {
		LocalDateTime now = LocalDateTime.now();
		StringBuilder sb = new StringBuilder(statusTimestampFormat.format(now));
		sb.append(',').append(System.currentTimeMillis()).append(',');
		sb.append(',').append(panel.getTotalCurrent());
		sb.append(',').append(panel.getTemperature());
		sb.append(',').append(panel.getTotalEnergy());
		sb.append(',').append(panel.getTotalPower());
		sb.append(',').append(panel.getVoltage());
		for (int channel = 0; channel < 16; channel++) {
			sb.append(',').append(panel.getCurrent(channel));
		}
		return sb.toString();
	}

	private PDP pdpSubsystem;
	private PrintWriter writer;
	private long lastLogTime;
	private File outputDir;

	public LogPDPInfo() {
		this(new File("/home/lvuser/logs"));
	}

	public LogPDPInfo(File outputDir) {
		log.info(getName() + " Beginning constructor");
		this.outputDir = outputDir;
		requires(Robot.pdp);
		log.debug("LogPDPInfo command created");
	}
	
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		pdpSubsystem = Robot.pdp;
		initializeLogFile();
	}

	//Initialize the logging file, using date
	private void initializeLogFile() {
		if (writer == null) {
			String fileName = "pdp-" + fileTimestampFormat.format(LocalDateTime.now()) + ".csv";
			File logFile = new File(outputDir, fileName);
			log.info("Creating new pdp log file " + fileName);
			try {
				writer = new PrintWriter(new BufferedWriter(new FileWriter(logFile)));
				writer.println(CSV_HEADER);
			} catch (IOException e) {
				log.error("Could not create pdp log file", e);
			}
		}
	}
	
	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (writer == null) return; //although never expected, this will prevent exceptions
		long currentMillis = System.currentTimeMillis();

		//Checks if MILLIS_BETWEEN_LOGS milliseconds has passed since last line
		if (currentMillis >= lastLogTime + MILLIS_BETWEEN_LOGS) {
			writer.println(getCSVStatusLine(pdpSubsystem.pdpPanel));
			lastLogTime = currentMillis;
		}

	}
	
	//Gets PDP status in CSV format.
	private String getCSVStatusLine() {
		StringBuilder sb = new StringBuilder();
		PowerDistributionPanel panel = pdpSubsystem.pdpPanel;
		sb.append(String.format("%s,%.2f,%.2f,%.2f,%.2f,%.2f", statusTimestampFormat.format(LocalDateTime.now()),
				panel.getTotalCurrent(),
				panel.getTemperature(),
				panel.getTotalEnergy(),
				panel.getTotalPower(),
				panel.getVoltage()));
		for(int i = 0; i < 16; i++) {
			sb.append(String.format(",%.2f", panel.getCurrent(i)));
		}
		return sb.toString();
	}
	
	private void closeLogFile() {
		if (writer != null) {
			writer.flush();
			writer.close();
			writer = null;
			lastLogTime = 0;
			log.info("PDP log file closed");
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		// prevent execute being called when we don't have a file to write to
		return writer == null; 
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		closeLogFile();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
*/
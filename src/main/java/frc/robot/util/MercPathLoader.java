/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

import frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix.motion.TrajectoryPoint;
/**
 * Add your docs here.
 */
public class MercPathLoader {
    private static final String BASE_PATH_LOCATION = "/home/lvuser/deploy/trajectories/output/";
    /**
     * @param pathName name + wpilib.json
     */
    public static List<TrajectoryPoint> loadPath(String pathName, double angleOffset, boolean lastTrajectory) {
        List<TrajectoryPoint> trajectoryPoints = new ArrayList<TrajectoryPoint>();
        List<Trajectory.State> trajectoryStates;
        Trajectory trajectory = null;

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(BASE_PATH_LOCATION + pathName + ".wpilib.json");
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + pathName, ex.getStackTrace());
            return null;
        }
        if (trajectory != null) {
            trajectoryStates = trajectory.getStates();
            Trajectory.State prevState = null;
            int prevTime = 0;
            double prevHeading = 0, headingOffset = 0;
            double pos = 0.0;
            for(Trajectory.State state : trajectoryStates) {
                TrajectoryPoint point = new TrajectoryPoint();
                double headingDelta, heading, velocity;
                int time;

                //Time
                time = MercMath.secondsToMilliseconds(state.timeSeconds);
                point.timeDur = time - prevTime;
                prevTime = time;

                //Velocity
                velocity = state.velocityMetersPerSecond;
                point.velocity = MercMath.metersPerSecondToTicksPerTenth(velocity);

                //Distance
                if (prevState == null) {
                    point.position = 0.0;
                    point.zeroPos = true;
                } else {
                    double prevX, prevY, x, y;
                    prevX = prevState.poseMeters.getTranslation().getX();
                    prevY = prevState.poseMeters.getTranslation().getY();
                    x = state.poseMeters.getTranslation().getX();
                    y = state.poseMeters.getTranslation().getY();
                    pos += MercMath.distanceFormula(prevX, x, prevY, y);
                    state.poseMeters.getTranslation().getDistance(prevState.poseMeters.getTranslation());
                    point.position = MercMath.metersToEncoderTicks(pos);
                    point.zeroPos = false;
                }
                prevState = state;

                if (point.zeroPos) {
                    // Our trajectories assume an initial heading of 0 degrees
                    // Offset all points based on the heading of the first point
                    headingOffset = state.poseMeters.getRotation().getDegrees();
                }
                // Calculate heading delta based on absolute heading from path trajectory
                heading = state.poseMeters.getRotation().getDegrees() - headingOffset;
                heading += angleOffset;
                if (heading > 90 && heading <= 180 && prevHeading < -90 && prevHeading > -180) {
                    headingDelta = heading - prevHeading - 360;
                } else if (heading > -180 && heading < -90 && prevHeading <= 180 && prevHeading > 90) {
                    headingDelta = heading - prevHeading + 360;
                } else {
                    headingDelta = heading - prevHeading;
                }
                prevHeading = heading;

                if (trajectoryPoints.size() == 0) {
                    // First point - delta is heading
                    point.auxiliaryPos = MercMath.degreesToPigeonUnits(heading);
                } else {
                    // Apply delta to previous pos
                    point.auxiliaryPos = 
                        trajectoryPoints.get(trajectoryPoints.size() - 1).auxiliaryPos +
                        MercMath.degreesToPigeonUnits(headingDelta);
                }

                //PID Profile
                point.profileSlotSelect0 = DriveTrain.DRIVE_MOTION_PROFILE_SLOT;
                point.profileSlotSelect1 = DriveTrain.DRIVE_SMOOTH_MOTION_SLOT;
                point.useAuxPID = true;
                //Says that point is not a last point
                point.isLastPoint = false;
                //Append point to point
                trajectoryPoints.add(point);
                /*
                    System.out.println(pathName + 
                                       " time: " + time + 
                                       " radians: " + state.poseMeters.getRotation().getRadians() + 
                                       " degrees: " + state.poseMeters.getRotation().getDegrees() +
                                       " heading: " + MercMath.pigeonUnitsToDegrees(point.auxiliaryPos) 
                    );*/
                /*
                System.out.println("time: " + time + 
                                   " velocity: " + MercMath.inchesPerSecondToRevsPerMinute(state.velocityMetersPerSecond) + 
                                   " heading: " + heading +
                                   " pos: " + pos +
                                   " TicksPerTenth Values " + MercMath.revsPerMinuteToTicksPerTenth(MercMath.inchesPerSecondToRevsPerMinute(state.velocityMetersPerSecond)) +
                                   " point.time: " + point.timeDur +
                                   " point.velocity: " + point.velocity + 
                                   " point.auxiliaryPos: " + point.auxiliaryPos +
                                   " point.position: " + point.position
                );*/
            }

            trajectoryPoints.get(trajectoryPoints.size() - 1).isLastPoint = lastTrajectory;
        }
        return trajectoryPoints;
    }

    public static List<TrajectoryPoint> loadPath(String pathName) {
        return loadPath(pathName, 0, true);
    }

    public static String getBasePathLocation() {
        return BASE_PATH_LOCATION;
    }
}

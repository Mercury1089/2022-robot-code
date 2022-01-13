/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import com.ctre.phoenix.motion.TrajectoryPoint;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;
/**
 * Add your docs here.
 */
public class MercMotionProfile {
    private final double ANGLE_OFFSET;
    private final List<TrajectoryPoint> TRAJECTORY_POINTS;
    private final ProfileDirection DIRECTION;
    private final String NAME;
    private final String PATH_DIRECTORY;

    public MercMotionProfile(String name, ProfileDirection direction) {
        this(name, direction, 0);
    }

    public MercMotionProfile(String name, ProfileDirection direction, double angleOffset) {
        this.NAME = name;
        this.DIRECTION = direction;
        this.ANGLE_OFFSET = angleOffset;
        PATH_DIRECTORY = MercPathLoader.getBasePathLocation() + name + ".wpilib.json"; 
        TRAJECTORY_POINTS = MercPathLoader.loadPath(name, angleOffset, true);
        if(this.DIRECTION == ProfileDirection.BACKWARD)
            driveBackwards();
    }

    public MercMotionProfile(String name, ProfileDirection direction, double angleOffset, boolean lastPoint) {
        this.NAME = name;
        this.DIRECTION = direction;
        this.ANGLE_OFFSET = angleOffset;
        PATH_DIRECTORY = MercPathLoader.getBasePathLocation() + name + ".wpilib.json"; 
        TRAJECTORY_POINTS = MercPathLoader.loadPath(name, angleOffset, lastPoint);
        if(this.DIRECTION == ProfileDirection.BACKWARD)
            driveBackwards();
    }

    public MercMotionProfile(String ... names) {
        NAME = "Paths:";
        DIRECTION = ProfileDirection.FORWARD;
        ANGLE_OFFSET = 0.0;
        PATH_DIRECTORY = MercPathLoader.getBasePathLocation() + "[Many Paths]" + ".wpilib.json";
        
        List<TrajectoryPoint> loader = new ArrayList<TrajectoryPoint>();
        for (String name : names) {
            NAME.concat(" " + name);
            loader.addAll(MercPathLoader.loadPath(name, ANGLE_OFFSET, false));
        }
        loader.get(loader.size() - 1).isLastPoint = true;
        TRAJECTORY_POINTS = loader;
    }

    public String getName() {
        return NAME;
    }

    public double getAngleOffset() {
        return ANGLE_OFFSET;
    }

    public String getPathDirectory() {
        return PATH_DIRECTORY;
    }

    public List<TrajectoryPoint> getTrajectoryPoints() {
        return TRAJECTORY_POINTS;
    }

    public void driveBackwards() {
        if(TRAJECTORY_POINTS == null) {
            DriverStation.reportError("No Trajectory To Load", false);
            return;
        }
        for (int i = 0; i < TRAJECTORY_POINTS.size(); i++) {
            TRAJECTORY_POINTS.get(i).velocity *= -1;
            TRAJECTORY_POINTS.get(i).position *= -1;
            //TRAJECTORY_POINTS.get(i).auxiliaryPos += MercMath.degreesToPigeonUnits(180);
        }
    }

    public enum ProfileDirection{
        FORWARD,
        BACKWARD
    }
}

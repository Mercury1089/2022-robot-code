package frc.robot.util;

import java.util.Scanner;
import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.MercMotionProfile.ProfileDirection;

import java.util.List;
import java.util.NoSuchElementException;
import java.util.ArrayList;


public class MercPathGroup{
    /*
    private List<MercMotionProfile> profiles;
    private final String BASE_PATH_LOCATION = "/home/lvuser/deploy/trajectories/PathWeaver/Groups/";

    public MercPathGroup(String name){
        profiles = new ArrayList<MercMotionProfile>();

        try{
            Scanner groupReader = new Scanner(BASE_PATH_LOCATION + name);
            List<String> pathNames = new ArrayList<String>();
            String path;

            while (true){
                try{
                    path = groupReader.nextLine();
                    path = path.substring(0, path.indexOf(".path"));
                } catch (NoSuchElementException ex){
                    break;
                }
                pathNames.add(path);
            }

            for (String p : pathNames){
                profiles.add((p.charAt(0) == 'B') ? new MercMotionProfile(p, ProfileDirection.BACKWARDS) : new MercMotionProfile(p, ProfileDirection.FORWARD));
            }
            
            groupReader.close();

        } catch (Exception ex){
            DriverStation.reportError("Problem Loading Path Group", ex.getStackTrace());
        }

    }

    public List<MercMotionProfile> getProfiles(){
        return profiles;
    }
    */
}
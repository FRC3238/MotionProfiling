package ProfileGenerator;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.List;

import jaci.pathfinder.*;
import jaci.pathfinder.modifiers.TankModifier;

public class AutoDriveMotionProfileHelper {

    public enum AllianceColors
    {
        BLUE,
        RED
    }

    public enum DriveSides
    {
        LEFT,
        RIGHT
    }
    public static Waypoint[] getWaypointsSideWallToCenterLift() {
        Waypoint[] result = new Waypoint[2];
        result[0] = new Waypoint(0,0,Math.PI/2);
        result[1] = new Waypoint(0, ConstantsMP.kDriverWallToCenterLift - (ConstantsMP.kRobotLengthWithBumpers/2) - ConstantsMP.kFrontOfGearToRobotCenter , Math.PI/2);
        return result;
    }
    public static Waypoint[] GetWaypointsBoilerCornerToSideLift()
    {
        Waypoint[] result = new Waypoint[3];

        //starting point:
        //Robot starts with back corner of robot at intersection of driver station wall and boiler face, robot facing other alliance station wall
        //Calculations based on geometric center of the robot.
        result[0] = new Waypoint(0, 0, 0); 

        //ending point with gear on lift
        double x_final = ConstantsMP.kDriverWallToSideLift 
                - ConstantsMP.kFrontOfGearToRobotCenter * Math.cos(Math.toRadians(60)) 
                - ConstantsMP.kRobotLengthWithBumpers/2;

        double y_final = ConstantsMP.kSideWallToBoilerSideLift 
                - ConstantsMP.kFrontOfGearToRobotCenter * Math.sin(Math.toRadians(60)) 
                - ConstantsMP.kRobotWidthWithBumpers/2;
        
        result[2] = new Waypoint(x_final, y_final, Math.toRadians(60)); 

        //intermediate point three feet straight out from ending point at lift so we come straight at the lift for a decent distance
        double x = x_final - 36 * Math.cos(Math.toRadians(60));
        double y = y_final - 36 * Math.sin(Math.toRadians(60));
        result[1] = new Waypoint(x, y, Math.toRadians(60)); 
        
        return result;      
    }
    
    
    public static Trajectory GetTrajectoryBoilerCornerToSideLift(double timestepInSECONDS, double maxSpeed, double maxAccel, double maxJerk)
    {
        //returns single trajectory of the center of geometry
        
        //timestep in SECONDS, i.e. pass 0.01 for 10 milliseconds
        //maxSpeed, maxAccel, and maxJerk in consistent units.
        //these are the max values to use for this trajectory not necessarily the max the robot can achieve
        
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, timestepInSECONDS, maxSpeed, maxAccel, maxJerk);
        Trajectory trajectory = Pathfinder.generate(GetWaypointsBoilerCornerToSideLift(), config);

        return trajectory;
    }
    public static Trajectory getCenterLiftTrajectory(double timestepInSECONDS, double maxSpeed, double maxAccel, double maxJerk)
    {
        //returns single trajectory of the center of geometry
        
        //timestep in SECONDS, i.e. pass 0.01 for 10 milliseconds
        //maxSpeed, maxAccel, and maxJerk in consistent units.
        //these are the max values to use for this trajectory not necessarily the max the robot can achieve
        
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, timestepInSECONDS, maxSpeed, maxAccel, maxJerk);
        Trajectory trajectory = Pathfinder.generate(getWaypointsSideWallToCenterLift(), config);

        return trajectory;
    }
    public static List<TalonMotionProfilePoint> getMotionProfileLeftRight(Trajectory trajectory, AllianceColors color, DriveSides side)
    {
        //converts raw trajectory for center of geometry into left and right side motion profiles
        //based on wheel size, wheelbase width, and alliance color

        TankModifier modifier = new TankModifier(trajectory);
        modifier.modify(ConstantsMP.kWheelbaseWidth);

        Trajectory sideTrajectory  = null;
        
        //left/right reversed for blue alliance
        if (color == AllianceColors.RED)
        {
            if (side == DriveSides.LEFT)
                sideTrajectory  = modifier.getLeftTrajectory();
            else
                sideTrajectory = modifier.getRightTrajectory();
        }
        else
        {
            if (side == DriveSides.LEFT)
                sideTrajectory  = modifier.getRightTrajectory();
            else
                sideTrajectory = modifier.getLeftTrajectory();
        }

        //to verify testing
        String trajectoryFile = "C:\\Users\\Team 3238\\Documents\\Motion Profiling\\";
        if (side == DriveSides.LEFT)
            trajectoryFile += "left.csv";
        else
            trajectoryFile += "right.csv";
        
        //output to file to look for problems
        //File myTrajectoryFile = new File(trajectoryFile);
        //Pathfinder.writeToCSV(myTrajectoryFile, sideTrajectory);

        //convert trajectories into motion profiles based on wheel size
        List<TalonMotionProfilePoint> sideMP = new ArrayList<TalonMotionProfilePoint>();
        for(int i = 0; i < sideTrajectory.length(); i++)
        {
            //PathFinder position is in distance units (whatever units were passed in) but Talon wants revolutions so divide by pi * D
            //PathFinder velocity is in distance units per time (whatever units were passed in) but Talon wants RPMs so multiple by 60/(pi * D)
            //PathFinder trajectories have timestep in seconds but Talon wants timestep in milliseconds
            sideMP.add(new TalonMotionProfilePoint(
                    sideTrajectory.segments[i].position / (Math.PI * ConstantsMP.kWheelDiameter),
                    sideTrajectory.segments[i].velocity * 60 / (Math.PI * ConstantsMP.kWheelDiameter),
                    sideTrajectory.segments[i].dt * 1000));
        }               

        return sideMP;
    }

    public static void WriteTalonMPToCSV(String filename, List<TalonMotionProfilePoint> sideMP)
    {
        try {
            FileWriter writer = new FileWriter(filename);
//            writer.append(PositionInRevolutions,SpeedInRPMs,TimestepInMilliseconds\r\n"); //Windows Carriage Return/Linefeed
            for (int i = 0; i < sideMP.size(); i++) {
                //new BigDecimal(String.valueOf(double)).setScale(yourScale, BigDecimal.ROUND_HALF_UP)
                TalonMotionProfilePoint point = sideMP.get(i);          
                writer.append(
                        ("{"+new BigDecimal(String.valueOf(point.GetPositionInRevolutions())).setScale(6, BigDecimal.ROUND_HALF_UP)).toString() + ", " +
                        (new BigDecimal(String.valueOf(point.GetSpeedInRPMs())).setScale(6, BigDecimal.ROUND_HALF_UP)).toString() + "," +
                        String.valueOf(point.GetTimestepInMilliseconds()) + "},\r\n"); //Windows Carriage Return/Linefeed
            }
            writer.toString();    
            writer.flush();
            writer.close();                 
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

    }   
    
}

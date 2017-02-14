package ProfileGenerator;

import java.io.File;
import java.util.List;

import javax.swing.JOptionPane;

//import AutoDriveMotionProfileHelper.DriveSides;
import jaci.pathfinder.*;
import jaci.pathfinder.modifiers.*;

public class Generator
{
    double maxVel = 60, maxAccel = 60, maxJerk = 1200, dt = 0.01;
    List<TalonMotionProfilePoint> leftSideProfile, rightSideProfile;

    public void init()
    {
//        // Trajectory boilerCornerToLiftTrajectory =
//        // AutoDriveMotionProfileHelper.GetTrajectoryBoilerCornerToSideLift(0.01,
//        // 60, 60, 1200);
//        Trajectory boilerCornerToLiftTrajectory = AutoDriveMotionProfileHelper
//                .GetTrajectoryBoilerCornerToSideLift(dt, maxVel, maxAccel,
//                        maxJerk);
//        Trajectory mainLiftTrajectory = AutoDriveMotionProfileHelper
//                .getCenterLiftTrajectory(dt, maxVel, maxAccel, maxJerk);
//        List<TalonMotionProfilePoint> sideMP = AutoDriveMotionProfileHelper
//                .getMotionProfileLeftRight(boilerCornerToLiftTrajectory,
//                        AutoDriveMotionProfileHelper.AllianceColors.RED,
//                        AutoDriveMotionProfileHelper.DriveSides.LEFT);
//        List<TalonMotionProfilePoint> sideMR = AutoDriveMotionProfileHelper
//                .getMotionProfileLeftRight(boilerCornerToLiftTrajectory,
//                        AutoDriveMotionProfileHelper.AllianceColors.RED,
//                        AutoDriveMotionProfileHelper.DriveSides.RIGHT);
//        List<TalonMotionProfilePoint> mainLift = AutoDriveMotionProfileHelper
//                .getMotionProfileLeftRight(mainLiftTrajectory,
//                        AutoDriveMotionProfileHelper.AllianceColors.RED,
//                        AutoDriveMotionProfileHelper.DriveSides.RIGHT);
//
//        AutoDriveMotionProfileHelper.WriteTalonMPToCSV(
//                "C:\\Users\\Team 3238\\Documents\\Motion Profiling\\mainLift.csv",
//                mainLift);
//        AutoDriveMotionProfileHelper.WriteTalonMPToCSV(
//                "C:\\Users\\Team 3238\\Documents\\Motion Profiling\\rightTalonMP.csv",
//                sideMR);
//
//        AutoDriveMotionProfileHelper.WriteTalonMPToCSV(
//                "C:\\Users\\Team 3238\\Documents\\Motion Profiling\\leftTalonMP.csv",
//                sideMP);
//
//        JOptionPane.showMessageDialog(null, "done");
    }

    public void generateSideProfile()
    {
        Trajectory boilerCornerToLiftTrajectory = AutoDriveMotionProfileHelper
                .GetTrajectoryBoilerCornerToSideLift(dt, maxVel, maxAccel,
                        maxJerk);
        leftSideProfile = AutoDriveMotionProfileHelper
                .getMotionProfileLeftRight(boilerCornerToLiftTrajectory,
                        AutoDriveMotionProfileHelper.AllianceColors.RED,
                        AutoDriveMotionProfileHelper.DriveSides.LEFT);
        rightSideProfile = AutoDriveMotionProfileHelper
                .getMotionProfileLeftRight(boilerCornerToLiftTrajectory,
                        AutoDriveMotionProfileHelper.AllianceColors.RED,
                        AutoDriveMotionProfileHelper.DriveSides.RIGHT);
    }

    public double[][] getRightSideLift()
    {

        return TalonListToDouble(rightSideProfile);
    }

    public double[][] getLeftSideLift()
    {

        return TalonListToDouble(leftSideProfile);
    }

    public double[][] getMainLift()
    {
        Trajectory mainLiftTrajectory = AutoDriveMotionProfileHelper
                .getCenterLiftTrajectory(dt, maxVel, maxAccel, maxJerk);
        List<TalonMotionProfilePoint> mainLift = AutoDriveMotionProfileHelper
                .getMotionProfileLeftRight(mainLiftTrajectory,
                        AutoDriveMotionProfileHelper.AllianceColors.RED,
                        AutoDriveMotionProfileHelper.DriveSides.RIGHT);

        return TalonListToDouble(mainLift);
    }
    public double[][] TalonListToDouble(List<TalonMotionProfilePoint> profile) {
        double[][] returner = new double[profile.size()][3];
        for(int i = 0; i < profile.size(); i++){
            returner[i][0] = profile.get(i).GetPositionInRevolutions();
            returner[i][1] = profile.get(i).GetSpeedInRPMs();
            returner[i][2] = profile.get(i).GetTimestepInMilliseconds();
        }
        return returner;
    }

}

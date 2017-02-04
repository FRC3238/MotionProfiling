package org.usfirst.frc.team3238.robot;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class Profiles
{
    public static double dt = 0.1, vel_max = 1.7, accel_max = 0.1, jerk_max = 1.0, wheelbase_width = 0.6;
    
    public void populateProfiles() {
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, 
                Trajectory.Config.SAMPLES_HIGH, dt, vel_max, accel_max, jerk_max);
        Waypoint[] points = new Waypoint[] {
                new Waypoint(-4, -1, Pathfinder.d2r(-45)),
                new Waypoint(-2, -2, 0),
                new Waypoint(0, 0, 0)
        };

        Trajectory trajectory = Pathfinder.generate(points, config);

        // Wheelbase Width = 0.5m
        TankModifier modifier = new TankModifier(trajectory).modify(wheelbase_width);

        // Do something with the new Trajectories...
        Trajectory left = modifier.getLeftTrajectory();
        Trajectory right = modifier.getRightTrajectory();
    }
    public double[][] leftProfile, rightProfile;
    
    
}

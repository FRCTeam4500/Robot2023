/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utility;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
//import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;


/**
 * Utilities for running autonomous paths
 * A bunch of the included methods will not be used, for the most part we will only be using getDeployedTrajectory()
 */
public class ExtendedTrajectoryUtilities {
    private static Trajectory getDeployedTrajectoryExcept(String trajectoryName) throws IOException {

        var trajectoryJSON = "paths/"+trajectoryName+".wpilib.json";
        //Stolen pretty much from the example code
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        return trajectory;
    }
    public static Trajectory getDeployedTrajectory(String trajectoryName){
        try{
            return getDeployedTrajectoryExcept(trajectoryName);
        }catch(IOException ex){
            DriverStation.reportError("Unable to open trajectory: " + trajectoryName, ex.getStackTrace());
            return new Trajectory(null);
        }
    }
    public static Trajectory regenerateTrajectory(Trajectory trajectory, TrajectoryConfig config){
        return TrajectoryGenerator.generateTrajectory(Arrays.asList(trajectory.getStates().stream().map(s -> s.poseMeters).toArray(Pose2d[]::new)), config);
    }

   
   
   
    public static HolonomicDriveController createBasicController(double kPx, double kPy, double kPw, double maxRotationalSpeed, double maxRotationlAcceleration){
        return new HolonomicDriveController(new PIDController(kPx, 0, 0), new PIDController(kPy, 0, 0), new ProfiledPIDController(kPw, 0, 0, new TrapezoidProfile.Constraints(maxRotationalSpeed, maxRotationlAcceleration) ));
    }
}

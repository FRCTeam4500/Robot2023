package frc.robot.subsystem.swerve.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.vision.Vision;

public class AlignToTargetCommand extends CommandBase {
    private Swerve swerve;
    private Vision vision;

    boolean alignSkew = true;
    boolean alignDepth = true;
    boolean alignHorizontal = true;
    boolean aligned = false;
    
    public AlignToTargetCommand(Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;
    }

    public void initialize () {
        reset();
        alignSkew();
        new WaitCommand(.5);
        alignSkew = false;
        alignHorizontal();
        new WaitCommand(.5);
        alignHorizontal = false;
        alignDepth();
        new WaitCommand(.5);
        alignDepth = false;
        aligned = true;
    }

    public void exectue() {}

    public boolean isFinished() {
        return aligned;
    }

    public void reset() {
        aligned = false;
        alignDepth = true;
        alignHorizontal = true;
        alignSkew = true;
    }

    public void alignHorizontal() {
        while(Math.abs(vision.getHorizontalOffsetFromTarget()) > VisionConstants.MAX_OFFSET_TO_TARGET && alignHorizontal) {
            swerve.moveRobotCentric(0, ceiling(vision.getHorizontalOffsetFromTarget(), 1), 0);
        }
    }

    public void alignDepth() {
        while(vision.getDepthToTarget() > VisionConstants.MAX_DEPTH_TO_TARGET && alignDepth){
            swerve.moveRobotCentric(ceiling(vision.getDepthToTarget()/10, 1), 0, 0);
        }
    }

    public void alignSkew() {
        while(Math.abs(vision.getSkew()) > 1 && alignSkew) {
            swerve.moveRobotCentric(0, 0, ceiling(vision.getSkew(), 1));
        }
    }

    public void offsetHorizontal() {
    }

    public double ceiling(double target, double max) {
        if (Math.abs(target) > Math.abs(max)) {
            if (target > 0) {return Math.abs(max);}
            if (target < 0) {return -Math.abs(max);}
        }
        return target;
    }


}

// package frc.robot.utility;

// import edu.wpi.first.math.Pair;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.util.sendable.Sendable;
// import edu.wpi.first.util.sendable.SendableBuilder;
// // import frc.robot.subsystem.swerve.Swerve;
// // import frc.robot.subsystem.turret.Turret;
// // import frc.robot.subsystem.vision.Vision;

// public class PolarVelocityCalculator implements Sendable{
//     private Vision vision;
//     private Swerve swerve;
//     private Turret turret;
//     private double pi = Math.PI;

//     public PolarVelocityCalculator(Swerve swerve, Vision vision, Turret turret){
//         this.vision = vision;
//         this.swerve = swerve;
//         this.turret = turret;
//     }

//     /**
//      * Yes, all the cos(90-angle) can be expressed as sin(angle), but
//      * all of the velocity vectors are based from the center of the robot,
//      * so working with cosine is much more intuitive when thinking about them geometrically.
//      * There is also no difference when the angle is more than 90, bc cosine is still positive when the angle is negative
//      * and goes negative when the angle is over 90, so there is no need to conditionally negate the component vectors
//      * if the angle is over 90.
//      * ^That's mostly just a note for myself, I thought it was way more complicated than it is, trigonometry is hard
//      */

//     /**
//      * Translates speeds from x-y directions to polar (radial and tangential) speeds.
//      * @return Pair consisting of radial and tangential speeds
//      */
//     public Pair<Double, Double> getPolarSpeeds(){
//         double angleFromTarget = pi/2 - (vision.getVisionAngle() + turret.getAngle()); //angle between the robot's x axis and the target, will be pi/2 if the robot is facing the target
//         ChassisSpeeds speeds = swerve.getSpeeds();
//         /**
//          * translates speeds from robot xy directions to tangential and radial directions
//          * it does this by scalar projection of the xy vectors onto the polar vectors
//          * A = angle between robot x axis and the target
//          * vr = vxcosA  + vycos(90-A)
//          * vt = vxcos(90-A) + vycos(A)
//          */
//         double radialVelocity = (speeds.vxMetersPerSecond * Math.cos(angleFromTarget)) +
//                 (speeds.vyMetersPerSecond * Math.cos(pi/2 - angleFromTarget));
//         double tangentialVelocity = (speeds.vxMetersPerSecond * Math.cos(pi/2 - angleFromTarget)) +
//                 (speeds.vyMetersPerSecond * Math.cos(angleFromTarget));
//         return new Pair<> (radialVelocity, tangentialVelocity);
//     }

//     public double getRadialSpeed(){
//        ChassisSpeeds speeds = swerve.getSpeeds();
//        double angleFromTarget = pi/2 - (vision.getVisionAngle() + turret.getAngle()); //angle between the robot's x axis and the target, will be pi/2 if the robot is facing the target
//        return (speeds.vxMetersPerSecond * Math.cos(angleFromTarget)) + (speeds.vyMetersPerSecond * Math.cos(pi/2 - angleFromTarget));
//     }

//     public double getTangentialSpeed(){
//         ChassisSpeeds speeds = swerve.getSpeeds();
//         double angleFromTarget = pi/2 - (vision.getVisionAngle() + turret.getAngle()); //angle between the robot's x axis and the target, will be pi/2 if the robot is facing the target
//         return (speeds.vxMetersPerSecond * Math.cos(pi/2 - angleFromTarget)) + (speeds.vyMetersPerSecond * Math.cos(angleFromTarget));
//     }

//     /**
//      * calculates cartesian x-y speeds from target polar speeds, in order to drive the robot in polar coords.
//      * This is necessary because the swerve system works based on x-y speeds relative to the robot.
//      * @param targetRadialSpeed The intended radial speed (toward/away from center)
//      * @param targetTangentialSpeed the intended tangential speed (around counterclockwise/clockwise) from center
//      * @return Pair<Double,Double> the x and y speeds relative to the robot which would correspond to the target radial/tangential speeds
//      */
//     public Pair<Double, Double> calculateCartesianSpeeds(double targetRadialSpeed, double targetTangentialSpeed){
//         double angleFromTarget = pi/2 - (vision.getVisionAngle() + turret.getAngle()); //angle between the robot's x axis and the target, will be pi/2 if the robot is facing the target
//         double xVelocity = targetTangentialSpeed * Math.cos(pi/2 - angleFromTarget) + targetRadialSpeed * Math.cos(angleFromTarget);
//         double yVelocity = targetTangentialSpeed * Math.cos(angleFromTarget) + targetRadialSpeed * Math.cos(angleFromTarget);
//         return new Pair<Double, Double>(xVelocity, yVelocity);
//     }

//     public void initSendable(SendableBuilder builder){
//         builder.addDoubleProperty("Radial Speed", ()-> getRadialSpeed(), null);
//         builder.addDoubleProperty("Tangential Speed", ()-> getTangentialSpeed(), null);
//     }
// }

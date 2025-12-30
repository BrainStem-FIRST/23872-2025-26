package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.util.Angle;

public class Coordinates {

    boolean redSide;


    // Far Auto Coordinates
    //FIXME MAKE SURE THAT ALL THE POSES ARE CORRECT
    Pose2d farStartPose = new Pose2d(-72, 35.56, Math.toRadians(90));


    // Close Auto Coordinates - all for red side
    //FIXME MAKE SURE THAT ALL THE POSES ARE CORRECT
    private final Pose2d closeStartPose = new Pose2d(-22, 13, Math.toRadians(145));
    private final Pose2d close1ShootingPose = new Pose2d(-24, 24,Math.toRadians(145));
    private final Pose2d closeCollect3ballsPrePose = new Pose2d(-12.0, 30.0, Math.toRadians(90)); // a prepose is used to go to a place right before you need to make a small accurate move
    private final Pose2d closeCollect1BallPose = new Pose2d(closeCollect3ballsPrePose.position.x, closeCollect3ballsPrePose.position.y + 5, closeCollect3ballsPrePose.heading.toDouble()); // the collect poses are the same as the pre pose, but just moving forward enough to collect the ball
    private final Pose2d closeCollect2BallPose = new Pose2d(closeCollect1BallPose.position.x, closeCollect1BallPose.position.y + 5, closeCollect1BallPose.heading.toDouble());
    private final Pose2d closeCollect3BallPose = new Pose2d(closeCollect2BallPose.position.x, closeCollect2BallPose.position.y + 5, closeCollect2BallPose.heading.toDouble());
    private final Pose2d close2ShootingPose = new Pose2d(-24.0, 24.0, Math.toRadians(145));
    private final Pose2d close3ShootingPose = new Pose2d(-24, 24, Math.toRadians(145));
    private final Pose2d close2CollectPose = new Pose2d(12, 24, Math.toRadians(145));


    // Close Spline Tangents
    private final double closeCollect3BallsDriveTangent = Math.toRadians(45);
    private final double collectTangenet = Math.toRadians(90);
    private final double closeShootDriveTangent = Math.toRadians(135);

    public Coordinates(boolean red){
        this.redSide = red;
    }



    /*
            these are functions that are public so that the auto classes can access them.
            each function is called get and then the name of the pose. inside each function, it returns
            the coordinate using a ternary operator. a ternary operator is just a fancy if else statement
            and in this case, it is checking to see if it is a coordinate for the blue side, and if so then
            it will return the pose that is flipped. this means that your red side autos and your blue side autos are
            just mirrored of each-other.
     */

    // far side get functions
    public Pose2d getFarStartPose(){
        return ((redSide) ? farStartPose : flipPose(farStartPose));
    }



    // close side get functions


    public Pose2d getCloseStartPose(){
        return (redSide ? closeStartPose : flipPose(closeStartPose));
    }

    public Pose2d getClose1ShootingPose(){
        return (redSide ? close1ShootingPose : flipPose(close1ShootingPose));
    }

    public Pose2d getCloseCollect3ballsPrePose(){
        return (redSide ? closeCollect3ballsPrePose : flipPose(closeCollect3ballsPrePose));
    }

    public Pose2d getCloseCollect1BallPose(){
        return (redSide ? closeCollect1BallPose : flipPose(closeCollect1BallPose));
    }

    public Pose2d getCloseCollect2BallPose(){
        return (redSide ? closeCollect2BallPose : flipPose(closeCollect2BallPose));
    }

    public Pose2d getCloseCollect3BallPose(){
        return (redSide ? closeCollect3BallPose : flipPose(closeCollect3BallPose));
    }

    public Pose2d getClose2ShootingPose(){
        return (redSide ? close2ShootingPose : flipPose(close2ShootingPose));
    }

    public Pose2d getClos3ShootingPose(){
        return (redSide ? close3ShootingPose : flipPose(close3ShootingPose));
    }

    public double getCloseCollect3BallsDriveTangent(){
        return (redSide ? closeCollect3BallsDriveTangent : flipHeading(closeCollect3BallsDriveTangent));
    }

    public double getCollectTangenet(){
        return (redSide ? collectTangenet : flipHeading(collectTangenet));
    }

    public double getShootingDriveTangent(){
        return (redSide ? closeShootDriveTangent : flipHeading(closeCollect3BallsDriveTangent));
    }



    //Util
    private double flipHeading(double heading) { // radians
        return Angle.norm(2.0 * Math.PI - heading);
    }
    private Pose2d flipPose(Pose2d pose) {
        return new Pose2d(pose.position.x, -pose.position.y, flipHeading(pose.heading.toDouble()));
    }

    public Pose2d getClose3ShootingPose() {
        return null;
    }
}

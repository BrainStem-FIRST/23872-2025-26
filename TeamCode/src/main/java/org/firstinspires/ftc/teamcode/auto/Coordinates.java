package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.util.Angle;

public class Coordinates {

    boolean redSide;


    // Far Auto Coordinates
    //FIXME MAKE SURE THAT ALL THE POSES ARE CORRECT
    Pose2d farStartPose = new Pose2d(-72, 35.56, Math.toRadians(90));


    // Close Auto Coordinates - all for red side
    private Pose2d closeStartPose = new Pose2d(-63, 36, 0);

    private Pose2d close1ShootingPose = new Pose2d(-15, 22, Math.toRadians(135));
    private Pose2d close1CollectPrePose = new Pose2d(-13, 28, Math.toRadians(90));
    private Pose2d collect1Pose = new Pose2d(-13, 43, Math.toRadians(90));
    private Pose2d collect2Pose = new Pose2d(-13, 48, Math.toRadians(90));
    private Pose2d collect3Pose = new Pose2d(-13, 53, Math.toRadians(90));
    private Pose2d close2ShootingPose = new Pose2d(-15, 22, Math.toRadians(135));



    /*
            these are functions that are public so that the auto classes can access them.
            each function is called get and then the name of the pose. inside each function, it returns
            the coordinate using a ternary operator. a ternary operator is just a fancy if else statement
            and in this case, it is checking to see if it is a coordinate for the blue side, and if so then
            it will return the pose that is flipped. this means that your red side autos and your blue side autos are
            just mirrored of each-other.
     */

    public Coordinates(boolean red){
        this.redSide = red;
    }

    // far side get functions
    public Pose2d getFarStartPose(){
        return ((redSide) ? farStartPose : flipPose(farStartPose));
    }



    // close side get functions
    public Pose2d getCloseStartPose(){
        return ((redSide) ? closeStartPose : flipPose(closeStartPose));
    }

    public Pose2d getClose1ShootingPose(){
        return ((redSide) ? close1ShootingPose : flipPose(close1ShootingPose));
    }

    public Pose2d getClose1CollectPrePose(){
        return ((redSide) ? close1CollectPrePose : flipPose(close1CollectPrePose));
    }

    public Pose2d getCollect1Pose(){
        return ((redSide) ? collect1Pose : flipPose(collect1Pose));
    }

    public Pose2d getCollect2Pose(){
        return ((redSide) ? collect2Pose : flipPose(collect2Pose));
    }

    public Pose2d getCollect3Pose(){
        return ((redSide) ? collect3Pose : flipPose(collect3Pose));
    }

    public Pose2d getClose2ShootingPose(){
        return ((redSide) ? close2ShootingPose : flipPose(close2ShootingPose));
    }








    //Util
    public double flipHeading(double heading) { // radians
        return Angle.norm(2.0 * Math.PI - heading);
    }
    public Pose2d flipPose(Pose2d pose) {
        return new Pose2d(pose.position.x, -pose.position.y, flipHeading(pose.heading.toDouble()));
    }

    public Pose2d getClose3ShootingPose() {
        return null;
    }

    public static class closeStartPose {
        public closeStartPose(int i, int i1, double radians) {
        }
    }
}

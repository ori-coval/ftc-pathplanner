package org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

public class TrajectoryPoses {

    private final RobotControl robot;

    //region Methods

    public double getAngle(double angle) {
        double result;
        if (robot.allianceColor == AllianceColor.RED) {
            result = Math.toRadians(angle);
        } else {
            result = Math.toRadians(180 - angle);
        }
        return result;
    }

    public double getX(double x) {
        if (robot.allianceColor == AllianceColor.BLUE) {
            return x + 5;
        } else return -x;
    }

    private MecanumVelocityConstraint reduceVelocity(double newVelocity) {
        return new MecanumVelocityConstraint(DriveConstants.MAX_VEL * newVelocity, DriveConstants.TRACK_WIDTH);
    }

    private ProfileAccelerationConstraint reduceAcceleration(double newAcceleration) {
        return new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL * newAcceleration);
    }
    //endregion

    //region Poses And Movements Variables

    public static Pose2d realBackdropFarPoseRed = new Pose2d(-15.75, -64.13, Math.toRadians(90));
    public static Pose2d realBackdropClosePoseRed = new Pose2d(0, 0, 0); //todo need to find this
    public static Pose2d realBackdropFarPoseBlue = new Pose2d(17.7, -64.9, Math.toRadians(90));
    public static Pose2d realBackdropClosePoseBlue = new Pose2d(0, 0, 0); //todo need to find this
    public static Pose2d stackPoseRed = new Pose2d(-13, 58, Math.toRadians(90));
    public static Pose2d stackPoseBlue = new Pose2d(16, 57, Math.toRadians(90));
    public Pose2d farPurpleFarPart1Red;
    public Pose2d farPurpleFarPart1Blue;
    public Pose2d farPurpleFarPart2Red;
    public Pose2d farPurpleFarPart2Blue;

    public MecanumVelocityConstraint farStackVelocity;
    public ProfileAccelerationConstraint farStackAcceleration;

    public Pose2d farPurpleCenterPart1Red;
    public Pose2d farPurpleCenterPart1Blue;
    public Pose2d farPurpleCenterPart2Red;
    public Pose2d farPurpleCenterPart2Blue;
    public Pose2d farStackCenterRed;
    public Pose2d farStackCenterBlue;

    public MecanumVelocityConstraint centerStackVelocityPart1;
    public MecanumVelocityConstraint centerStackVelocityPart2;
    public ProfileAccelerationConstraint centerStackAccelerationPart1;
    public ProfileAccelerationConstraint centerStackAccelerationPart2;

    public Pose2d farPurpleClosePart1Red;
    public Pose2d farPurpleClosePart1Blue;
    public Vector2d farPurpleClosePart2Red;
    public Vector2d farPurpleClosePart2Blue;
    public Vector2d farStackCloseRed;
    public Vector2d farStackCloseBlue;

    public MecanumVelocityConstraint closeStackVelocity;
    public ProfileAccelerationConstraint closeStackAcceleration;

    public Pose2d closePurpleClosePart1Red;
    public Pose2d closePurpleClosePart1Blue;
    public Pose2d closePurpleClosePart2Red;
    public Pose2d closePurpleClosePart2Blue;
    public Pose2d closeYellowClosePart1Red;
    public Pose2d closeYellowClosePart1Blue;
    public Vector2d closeYellowClosePart2Red;
    public Vector2d closeYellowClosePart2Blue;

    public Pose2d closePurpleCenterPart1Red;
    public Pose2d closePurpleCenterPart1Blue;
    public Pose2d closePurpleCenterPart2Red;
    public Pose2d closePurpleCenterPart2Blue;
    public Pose2d closeYellowCenterRed;
    public Pose2d closeYellowCenterBlue;


    public Pose2d closePurpleFarPart1Red;
    public Pose2d closePurpleFarPart1Blue;
    public Vector2d closePurpleFarPart2Red;
    public Vector2d closePurpleFarPart2Blue;
    public Pose2d closeYellowFarRed;
    public Pose2d closeYellowFarBlue;

    public MecanumVelocityConstraint biteForwardVelocity;
    public MecanumVelocityConstraint biteBackwardVelocity;
    public ProfileAccelerationConstraint biteForwardAcceleration;
    public ProfileAccelerationConstraint biteBackwardAcceleration;

    public Pose2d backdropFrontRed;
    public Pose2d backdropFrontBlue;

    public Pose2d stackAndBackdropPart1Red;
    public Pose2d stackAndBackdropPart1Blue;
    public Pose2d stackAndBackdropPart2Red;
    public Pose2d stackAndBackdropPart2Blue;
    public Pose2d stackAndBackdropPart3Red;
    public Pose2d stackAndBackdropPart3Blue;
    public Pose2d stackAndBackdropPart4Red;
    public Pose2d stackAndBackdropPart4Blue;
    public MecanumVelocityConstraint enterBackdropVelocity;
    public ProfileAccelerationConstraint enterBackdropAcceleration;

    public Pose2d aBitBeforeStackRed;
    public Pose2d aBitBeforeStackBlue;
    public MecanumVelocityConstraint beforeStackVelocityCycle;
    public ProfileAccelerationConstraint beforeStackAccelerationCycle;

    public Pose2d annoyingStackRed;

    public Vector2d parkingFarPart1Red;
    public Vector2d parkingFarPart1Blue;
    public Vector2d parkingFarPart2Red;
    public Vector2d parkingFarPart2Blue;

    public MecanumVelocityConstraint parkingVelocity;
    public ProfileAccelerationConstraint parkingAcceleration;

    public Vector2d parkingCloseMiddleRed;
    public Vector2d parkingCloseMiddleBlue;
    public Vector2d parkingCloseRed;
    public Vector2d parkingCloseBlue;


    //endregion

    public TrajectoryPoses(RobotControl robot) {
        this.robot = robot;

        //Poses
//        stackPoseRed = new Pose2d(-13, 58, Math.toRadians(90));
//        stackPoseBlue = new Pose2d(16, 57, Math.toRadians(90));

//        realBackdropPoseRed = new Pose2d(-15.75, -64.13, Math.toRadians(90));
//        realBackdropPoseBlue = new Pose2d(17.7, -64.9, Math.toRadians(90));

        //FAR
        //Far (Far) Purple RED
        farPurpleFarPart1Red = new Pose2d(-40, 50, Math.toRadians(60));
        farPurpleFarPart2Red = new Pose2d(-30, 50, Math.toRadians(60));

        //Far (Far) Purple BLUE
        farPurpleFarPart1Blue = new Pose2d(40, 50, Math.toRadians(120));
        farPurpleFarPart2Blue = new Pose2d(30, 50, Math.toRadians(120));

        //Far (Far) Stack Velocities & Acceleration
        double farStackConstant = 0.6;
        farStackVelocity = reduceVelocity(farStackConstant);
        farStackAcceleration = reduceAcceleration(farStackConstant);

        //Far (Center) Purple RED
        farPurpleCenterPart1Red = new Pose2d(-40, 37, Math.toRadians(60));
        farPurpleCenterPart2Red = new Pose2d(-25, 37, Math.toRadians(40));

        //Far (Center) Purple BLUE
        farPurpleCenterPart1Blue = new Pose2d(40, 37, Math.toRadians(120));
        farPurpleCenterPart2Blue = new Pose2d(25, 37, Math.toRadians(140));

        //Far (Center) Stack RED
        farStackCenterRed = new Pose2d(stackPoseRed.getX(), 47, Math.toRadians(90));

        //Far (Center) Stack BLUE
        farStackCenterBlue = new Pose2d(stackPoseBlue.getX(), 47, Math.toRadians(90));

        //Far (Center) Stack Velocities & Acceleration
        double centerStackConstantPart1 = 0.8;
        centerStackVelocityPart1 = reduceVelocity(centerStackConstantPart1);
        centerStackAccelerationPart1 = reduceAcceleration(centerStackConstantPart1);

        double centerStackConstantPart2 = 0.5;
        centerStackVelocityPart2 = reduceVelocity(centerStackConstantPart2);
        centerStackAccelerationPart2 = reduceAcceleration(centerStackConstantPart2);

        //Far (Close) Purple RED
        farPurpleClosePart1Red = new Pose2d(-50, robot.startPose.getY(), Math.toRadians(45));
        farPurpleClosePart2Red = new Vector2d(-34, 29);

        //Far (Close) Purple BLUE
        farPurpleClosePart1Blue = new Pose2d(50, robot.startPose.getY(), Math.toRadians(135));
        farPurpleClosePart2Blue = new Vector2d(36, 26);

        //Far (Close) Stack RED
        farStackCloseRed = new Vector2d(-22, 40);
        //Far (Close) Stack BLUE
        farStackCloseBlue = new Vector2d(22, 40);

        //Far (Close) Stack Velocities & Acceleration
        double closeStackConstant = 0.5;
        closeStackVelocity = reduceVelocity(closeStackConstant);
        closeStackAcceleration = reduceAcceleration(closeStackConstant);

        //CLOSE
        //Close (Close) Purple RED
        closePurpleClosePart1Red = new Pose2d(-40, -15, Math.toRadians(-60));
        closePurpleClosePart2Red = new Pose2d(-30, -5, Math.toRadians(-60));

        //Close (Close) Purple BLUE
        closePurpleClosePart1Blue = new Pose2d(40, -15, Math.toRadians(240));
        closePurpleClosePart2Blue = new Pose2d(30, -5, Math.toRadians(-60));

        //Close (Close) Yellow RED
        closeYellowClosePart1Red = new Pose2d(-7, -40, Math.toRadians(-90));
        closeYellowClosePart2Red = new Vector2d(-15, -64);

        //Close (Close) Yellow BLUE
        closeYellowClosePart1Blue = new Pose2d(7, -40, Math.toRadians(-90));
        closeYellowClosePart2Blue = new Vector2d(15, -64);

        //Close (Center) Purple
        closePurpleCenterPart1Red = new Pose2d(-40, -13, Math.toRadians(-60));
        closePurpleCenterPart2Red = new Pose2d(-22, -13, Math.toRadians(-45));

        //Close (Center) Yellow
        closeYellowCenterRed = new Pose2d(-60, -64, Math.toRadians(90));

        //Close (Far) Purple
        closePurpleFarPart1Red = new Pose2d(-50, -10, Math.toRadians(-45));
        closePurpleFarPart2Red = new Vector2d(-34, -22);

        //Close (Far) Yellow
        closeYellowFarRed = new Pose2d(-60, -64, Math.toRadians(90));

        //Bite Velocities & Acceleration
        double biteForwardConstant = 0.6;
        biteForwardVelocity = reduceVelocity(biteForwardConstant);
        biteForwardAcceleration = reduceAcceleration(biteForwardConstant);

        double biteBackwardConstant = 0.8;
        biteBackwardVelocity = reduceVelocity(biteBackwardConstant);
        biteBackwardAcceleration = reduceAcceleration(biteBackwardConstant);

        //Scoring Poses (Only Far)
        //Stack <-> Backdrop RED
        stackAndBackdropPart1Red = new Pose2d(stackPoseRed.getX() + 3, -15, Math.toRadians(90));
        stackAndBackdropPart2Red = new Pose2d(-9, -40, Math.toRadians(90));
        stackAndBackdropPart3Red = new Pose2d(-15, -61, Math.toRadians(90));
        stackAndBackdropPart4Red = new Pose2d(-22, -62, Math.toRadians(90));

        //Special Case - Close (Red Right)
        backdropFrontRed = new Pose2d(-40, -36, Math.toRadians(90));

        //Special Case - Close (Blue Left)
        backdropFrontBlue = new Pose2d(40, -36, Math.toRadians(90));


        //Stack <-> Backdrop BLUE
        stackAndBackdropPart1Blue = new Pose2d(stackPoseBlue.getX() - 2, -15, Math.toRadians(90));
        stackAndBackdropPart2Blue = new Pose2d(9, -40, Math.toRadians(90));
        stackAndBackdropPart3Blue = new Pose2d(15, -58, Math.toRadians(90));
        stackAndBackdropPart4Blue = new Pose2d(28, -63, Math.toRadians(90));


        double enterBackdropConstant = 0.6;
        enterBackdropVelocity = reduceVelocity(enterBackdropConstant);
        enterBackdropAcceleration = reduceAcceleration(enterBackdropConstant);

        //Cycles
        double cycleConstant = 0.4;
        beforeStackVelocityCycle = reduceVelocity(cycleConstant);
        beforeStackAccelerationCycle = reduceAcceleration(cycleConstant);
        //RED
        aBitBeforeStackRed = new Pose2d(stackPoseRed.getX(), 48, Math.toRadians(90));
        //BLUE
        aBitBeforeStackBlue = new Pose2d(stackPoseBlue.getX(), 48, Math.toRadians(90));

        annoyingStackRed = new Pose2d(-14, 58, Math.toRadians(90));

        //Parking
        //FAR RED
        parkingFarPart1Red = new Vector2d(-10, -57);
        parkingFarPart2Red = new Vector2d(-13, -51);

        //FAR BLUE
        parkingFarPart1Blue = new Vector2d(10, -57);
        parkingFarPart2Blue = new Vector2d(13, -51);

        double parkingConstant = 0.7;
        parkingVelocity = reduceVelocity(parkingConstant);
        parkingAcceleration = reduceAcceleration(parkingConstant);

        //CLOSEz
        //Middle of field
        parkingCloseMiddleRed = new Vector2d(-15, -53);
        //Normal
        parkingCloseRed = new Vector2d(-60, -53);

    }
}
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
        if(robot.allianceColor == AllianceColor.RED) {
            result = Math.toRadians(angle);
        } else {
            result = Math.toRadians(180 - angle);
        }
        return result;
    }

    public double getX(double x) {
        if(robot.allianceColor == AllianceColor.BLUE) {
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
    public Pose2d stackPose;
    public Pose2d farPurpleFarPart1;
    public Pose2d farPurpleFarPart2;

    public MecanumVelocityConstraint farStackVelocity;
    public ProfileAccelerationConstraint farStackAcceleration;

    public Pose2d farPurpleCenterPart1;
    public Pose2d farPurpleCenterPart2;
    public Pose2d farStackCenter;

    public MecanumVelocityConstraint centerStackVelocityPart1;
    public MecanumVelocityConstraint centerStackVelocityPart2;
    public ProfileAccelerationConstraint centerStackAccelerationPart1;
    public ProfileAccelerationConstraint centerStackAccelerationPart2;

    public Pose2d farPurpleClosePart1;
    public Vector2d farPurpleClosePart2;
    public Vector2d farStackClose;

    public MecanumVelocityConstraint closeStackVelocity;
    public ProfileAccelerationConstraint closeStackAcceleration;

    public Pose2d closePurpleClosePart1;
    public Pose2d closePurpleClosePart2;
    public Pose2d closeYellowClosePart1;
    public Vector2d closeYellowClosePart2;

    public Pose2d closePurpleCenterPart1;
    public Pose2d closePurpleCenterPart2;
    public Pose2d closeYellowCenter;


    public Pose2d closePurpleFarPart1;
    public Vector2d closePurpleFarPart2;
    public Pose2d closeYellowFar;

    public MecanumVelocityConstraint biteForwardVelocity;
    public MecanumVelocityConstraint biteBackwardVelocity;
    public ProfileAccelerationConstraint biteForwardAcceleration;
    public ProfileAccelerationConstraint biteBackwardAcceleration;

    public Pose2d stackAndBackdropPart1;
    public Pose2d stackAndBackdropPart2;
    public Pose2d stackAndBackdropPart3;
    public Pose2d aBitBeforeStack;
    public MecanumVelocityConstraint beforeStackVelocityCycle;
    public ProfileAccelerationConstraint beforeStackAccelerationCycle;

    public Vector2d parkingFarPart1;
    public Vector2d parkingFarPart2;

    public Vector2d parkingCloseMiddle;
    public Vector2d parkingClose;



    //endregion

    public TrajectoryPoses(RobotControl robot) {
        this.robot = robot;

        //Poses
        stackPose = new Pose2d(getX(12), 57, getAngle(90));

        //FAR
        //Far (Far) Purple
        farPurpleFarPart1 = new Pose2d(getX(35), 47, getAngle(60));
        farPurpleFarPart2 = new Pose2d(getX(25), 47, getAngle(60));

        //Far (Far) Stack Velocities & Acceleration
        double farStackConstant = 0.4;
        farStackVelocity = reduceVelocity(farStackConstant);
        farStackAcceleration = reduceAcceleration(farStackConstant);

        //Far (Center) Purple
        farPurpleCenterPart1 = new Pose2d(getX(40), 35, getAngle(60));
        farPurpleCenterPart2 = new Pose2d(getX(22), 35, getAngle(45));

        //Far (Center) Stack
        farStackCenter = new Pose2d(getX(12), 47, getAngle(90));

        //Far (Center) Stack Velocities & Acceleration
        double centerStackConstantPart1 = 0.6;
        centerStackVelocityPart1 = reduceVelocity(centerStackConstantPart1);
        centerStackAccelerationPart1 = reduceAcceleration(centerStackConstantPart1);

        double centerStackConstantPart2 = 0.4;
        centerStackVelocityPart2 = reduceVelocity(centerStackConstantPart2);
        centerStackAccelerationPart2 = reduceAcceleration(centerStackConstantPart2);

        //Far (Close) Purple
        farPurpleClosePart1 = new Pose2d(getX(50), 38, getAngle(45));
        farPurpleClosePart2 = new Vector2d(getX(34), 26);

        //Far (Close) Stack
        farStackClose = new Vector2d(getX(22), 40);

        //Far (Close) Stack Velocities & Acceleration
        double closeStackConstant = 0.4;
        closeStackVelocity = reduceVelocity(closeStackConstant);
        closeStackAcceleration = reduceAcceleration(closeStackConstant);

        //CLOSE
        //Close (Close) Purple
        closePurpleClosePart1 = new Pose2d(getX(40), -15, getAngle(-60));
        closePurpleClosePart2 = new Pose2d(getX(30), -5, getAngle(-60));

        //Close (Close) Yellow
        closeYellowClosePart1 = new Pose2d(getX(7), -40, getAngle(-90));
        closeYellowClosePart2 = new Vector2d(getX(15), -64);

        //Close (Center) Purple
        closePurpleCenterPart1 = new Pose2d(getX(40), -13, getAngle(-60));
        closePurpleCenterPart2 = new Pose2d(getX(22), -13, getAngle(-45));

        //Close (Center) Yellow
        closeYellowCenter = new Pose2d(getX(60), -64, getAngle(90));

        //Close (Far) Purple
        closePurpleFarPart1 = new Pose2d(getX(50), -10, getAngle(-45));
        closePurpleFarPart2 = new Vector2d(getX(34), -22);

        //Close (Far) Yellow
        closeYellowFar = new Pose2d(getX(60), -64, getAngle(90));

        //Bite Velocities & Acceleration
        double biteForwardConstant = 0.6;
        biteForwardVelocity = reduceVelocity(biteForwardConstant);
        biteForwardAcceleration = reduceAcceleration(biteForwardConstant);

        double biteBackwardConstant = 0.8;
        biteBackwardVelocity = reduceVelocity(biteBackwardConstant);
        biteBackwardAcceleration = reduceAcceleration(biteBackwardConstant);

        //Scoring Poses (Only Far)
        //Stack <-> Backdrop
        stackAndBackdropPart1 = new Pose2d(getX(12), -10, getAngle(90));
        stackAndBackdropPart2 = new Pose2d(getX(7), -40, getAngle(90));
        stackAndBackdropPart3 = new Pose2d(getX(18), -65, getAngle(90));

        //Cycles
        double cycleConstant = 0.3;
        aBitBeforeStack = new Pose2d(getX(12), 35, getAngle(90));
        beforeStackVelocityCycle = reduceVelocity(cycleConstant);
        beforeStackAccelerationCycle = reduceAcceleration(cycleConstant);


        //Parking
        //FAR
        parkingFarPart1 = new Vector2d(getX(5), -60);
        parkingFarPart2 = new Vector2d(getX(9), -51);

        //CLOSE
        //Middle of field
        parkingCloseMiddle = new Vector2d(getX(15), -53);
        //Normal
        parkingClose = new Vector2d(getX(60), -53);

    }

}

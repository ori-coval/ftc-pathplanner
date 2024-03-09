package org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RobotControl;

import java.util.HashMap;

public class Trajectories {

    /**
     * This class should only be used for -
     * <p>
     * Trajectory Names
     * <p>
     * New Trajectories
     * <p>
     * Trajectory Tangents
     * <p>
     * Adding More Points
     * <p>
     * For anything else, use TrajectoryPoses.
     *
     */

    public TrajectoryPoses trajectoryPoses;
    private final HashMap<String, TrajectorySequence> trajectorySequenceHashMap = new HashMap<>();

    public Trajectories(RobotControl robot, Pose2d startPose) {

        trajectoryPoses = new TrajectoryPoses(robot);

        //BLUE

        //Purple Pixel Trajectories
        //Robot Far From Backdrop
        //Far From Truss Detected
        trajectorySequenceHashMap.put("Far Purple (Far Detected) Blue", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(
                        trajectoryPoses.farPurpleFarPart1Blue,
                        Math.toRadians(180) //Tangent
                )
                .splineToSplineHeading(
                        trajectoryPoses.farPurpleFarPart2Blue,
                        Math.toRadians(180) //Tangent
                )
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack (Far Detected) Blue", robot.autoDriveTrain.trajectorySequenceBuilder(get("Far Purple (Far Detected) Blue").end())
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(
                        trajectoryPoses.stackPoseBlue,
                        Math.toRadians(135), //Tangent
                        trajectoryPoses.farStackVelocity,
                        trajectoryPoses.farStackAcceleration
                )
                .build()
        );

        //Center Detected
        trajectorySequenceHashMap.put("Far Purple (Center Detected) Blue", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(
                        trajectoryPoses.farPurpleCenterPart1Blue,
                        Math.toRadians(180) //Tangent
                )
                .splineToSplineHeading(
                        trajectoryPoses.farPurpleCenterPart2Blue,
                        Math.toRadians(180)) //Tangent
                .build()
        );

        trajectorySequenceHashMap.put("Driving to stack (Center Detected) Blue", robot.autoDriveTrain.trajectorySequenceBuilder(get("Far Purple (Center Detected) Blue").end())
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(
                        trajectoryPoses.farStackCenterBlue,
                        Math.toRadians(90), //Tangent
                        trajectoryPoses.centerStackVelocityPart1,
                        trajectoryPoses.centerStackAccelerationPart1
                )
                .splineToSplineHeading(
                        trajectoryPoses.stackPoseBlue,
                        Math.toRadians(90), //Tangent
                        trajectoryPoses.centerStackVelocityPart2,
                        trajectoryPoses.centerStackAccelerationPart2
                )
                .build()
        );

        //Close To Truss Detected
        trajectorySequenceHashMap.put("Far Purple (Close Detected) Blue", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(
                        trajectoryPoses.farPurpleClosePart1Blue,
                        Math.toRadians(180) //Tangent
                )
                .splineToConstantHeading(
                        trajectoryPoses.farPurpleClosePart2Blue,
                        Math.toRadians(225) //Tangent
                )
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack (Close Detected) Blue", robot.autoDriveTrain.trajectorySequenceBuilder(get("Far Purple (Close Detected) Blue").end())
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(
                        trajectoryPoses.farStackCloseBlue,
                        Math.toRadians(135) //Tangent
                )
                .splineToSplineHeading(
                        trajectoryPoses.stackPoseBlue,
                        Math.toRadians(90), //Tangent
                        trajectoryPoses.closeStackVelocity,
                        trajectoryPoses.closeStackAcceleration
                )
                .build()
        );


        //Bites
        trajectorySequenceHashMap.put("Drive back from stack Blue", robot.autoDriveTrain.trajectorySequenceBuilder(get("Driving to stack (Close Detected) Blue").end())
                .back(8,
                        trajectoryPoses.biteBackwardVelocity,
                        trajectoryPoses.biteBackwardAcceleration
                )
                .build()
        );
        trajectorySequenceHashMap.put("Drive back to stack Blue", robot.autoDriveTrain.trajectorySequenceBuilder(get("Drive back from stack Blue").end())
                .forward(8,
                        trajectoryPoses.biteForwardVelocity,
                        trajectoryPoses.biteForwardAcceleration
                )
                .build()
        );

        //Scoring Far
        trajectorySequenceHashMap.put("Go to backdrop (Far Side) Blue", robot.autoDriveTrain.trajectorySequenceBuilder(get("Driving to stack (Close Detected) Blue").end())
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(
                        trajectoryPoses.stackAndBackdropPart1Blue,
                        Math.toRadians(270) //Tangent
                )
                .splineToLinearHeading(
                        trajectoryPoses.stackAndBackdropPart2Blue,
                        Math.toRadians(270) //Tangent
                )
                .splineToLinearHeading(
                        trajectoryPoses.stackAndBackdropPart3Blue,
                        Math.toRadians(320) //Tangent
                )
                .splineToLinearHeading(
                        trajectoryPoses.stackAndBackdropPart4Blue,
                        Math.toRadians(-20), //Tangent
                        trajectoryPoses.enterBackdropVelocity,
                        trajectoryPoses.enterBackdropAcceleration
                )
                .build()
        );

        //Second Cycle
        trajectorySequenceHashMap.put("Back to stack (Second Cycle) Part 1 Blue", robot.autoDriveTrain.trajectorySequenceBuilder(trajectoryPoses.realBackdropPoseBlue)
                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(
                        trajectoryPoses.stackAndBackdropPart2Blue,
                        Math.toRadians(90) //Tangent
                )
                .splineToLinearHeading(
                        trajectoryPoses.stackAndBackdropPart1Blue,
                        Math.toRadians(85) //Tangent
                )
                .build()
        );

        trajectorySequenceHashMap.put("Back to stack (Second Cycle) Part 2 Blue", robot.autoDriveTrain.trajectorySequenceBuilder(get("Back to stack (Second Cycle) Part 1 Blue").end())
                .setTangent(Math.toRadians(85))
                .splineToLinearHeading(
                        trajectoryPoses.aBitBeforeStackBlue,
                        Math.toRadians(90) //Tangent
                )
                .splineToLinearHeading(
                        trajectoryPoses.stackPoseBlue,
                        Math.toRadians(90), //Tangent
                        trajectoryPoses.beforeStackVelocityCycle,
                        trajectoryPoses.beforeStackAccelerationCycle
                )
                .build()
        );



        //Parking
        trajectorySequenceHashMap.put("Parking Arm To Intake (Far Side) Blue", robot.autoDriveTrain.trajectorySequenceBuilder(trajectoryPoses.realBackdropPoseBlue)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(
                        trajectoryPoses.parkingFarPart1Blue,
                        Math.toRadians(90) //Tangent
                )
                .splineToConstantHeading(
                        trajectoryPoses.parkingFarPart2Blue,
                        Math.toRadians(45), //Tangent
                        trajectoryPoses.parkingVelocity,
                        trajectoryPoses.parkingAcceleration
                )
                .build()
        );


        //RED

        //Purple Pixel Trajectories
        //Robot Far From Backdrop
        //Far From Truss Detected
        trajectorySequenceHashMap.put("Far Purple (Far Detected) Red", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(
                        trajectoryPoses.farPurpleFarPart1Red,
                        Math.toRadians(0) //Tangent
                )
                .splineToSplineHeading(
                        trajectoryPoses.farPurpleFarPart2Red,
                        Math.toRadians(0) //Tangent
                )
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack (Far Detected) Red", robot.autoDriveTrain.trajectorySequenceBuilder(get("Far Purple (Far Detected) Red").end())
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(
                        trajectoryPoses.stackPoseRed,
                        Math.toRadians(45), //Tangent
                        trajectoryPoses.farStackVelocity,
                        trajectoryPoses.farStackAcceleration
                )
                .build()
        );

        //Center Detected
        trajectorySequenceHashMap.put("Far Purple (Center Detected) Red", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(
                        trajectoryPoses.farPurpleCenterPart1Red,
                        Math.toRadians(0) //Tangent
                )
                .splineToSplineHeading(
                        trajectoryPoses.farPurpleCenterPart2Red,
                        Math.toRadians(0)) //Tangent
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack (Center Detected) Red", robot.autoDriveTrain.trajectorySequenceBuilder(get("Far Purple (Center Detected) Red").end())
                .setTangent(Math.toRadians(40))
                .splineToSplineHeading(
                        trajectoryPoses.farStackCenterRed,
                        Math.toRadians(90), //Tangent
                        trajectoryPoses.centerStackVelocityPart1,
                        trajectoryPoses.centerStackAccelerationPart1
                )
                .splineToSplineHeading(
                        trajectoryPoses.stackPoseRed,
                        Math.toRadians(90), //Tangent
                        trajectoryPoses.centerStackVelocityPart2,
                        trajectoryPoses.centerStackAccelerationPart2
                )
                .build()
        );

        //Close To Truss Detected
        trajectorySequenceHashMap.put("Far Purple (Close Detected) Red", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(
                        trajectoryPoses.farPurpleClosePart1Red,
                        Math.toRadians(0) //Tangent
                )
                .splineToConstantHeading(
                        trajectoryPoses.farPurpleClosePart2Red,
                        Math.toRadians(-45) //Tangent
                )
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack (Close Detected) Red", robot.autoDriveTrain.trajectorySequenceBuilder(get("Far Purple (Close Detected) Red").end())
                .setTangent(Math.toRadians(45))
                .splineToConstantHeading(
                        trajectoryPoses.farStackCloseRed,
                        Math.toRadians(75) //Tangent
                )
                .splineToSplineHeading(
                        trajectoryPoses.stackPoseRed,
                        Math.toRadians(45), //Tangent
                        trajectoryPoses.closeStackVelocity,
                        trajectoryPoses.closeStackAcceleration
                )
                .build()
        );

        //Robot Close To Backdrop Side
        //Close to truss
        trajectorySequenceHashMap.put("Close Purple (Close Detected) Red", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(
                        trajectoryPoses.closePurpleClosePart1Red,
                        Math.toRadians(0) //Tangent
                )
                .splineToLinearHeading(
                        trajectoryPoses.closePurpleClosePart2Red,
                        Math.toRadians(90) //Tangent
                )
                .build()
        );

        trajectorySequenceHashMap.put("Close Yellow (Close Detected) Red", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Purple (Close Detected) Red").end())
                .setTangent(Math.toRadians(-45))
                .splineToSplineHeading(
                        trajectoryPoses.closeYellowClosePart1Red,
                        Math.toRadians(-90) //Tangent
                )
                .splineToConstantHeading(
                        trajectoryPoses.closeYellowClosePart2Red,
                        Math.toRadians(180) //Tangent
                )
                .build()
        );

        //Center
        trajectorySequenceHashMap.put("Close Purple (Center Detected) Red", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(
                        trajectoryPoses.closePurpleCenterPart1Red,
                        Math.toRadians(0) //Tangent
                )
                .splineToSplineHeading(
                        trajectoryPoses.closePurpleCenterPart2Red,
                        Math.toRadians(0) //Tangent
                )
                .build()
        );

        trajectorySequenceHashMap.put("Close Yellow (Center Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Purple (Center Detected) Red").end())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(
                        trajectoryPoses.closeYellowCenterRed,
                        Math.toRadians(-90) //Tangent
                )
                .build()
        );

        //Far from truss
        trajectorySequenceHashMap.put("Close Purple (Far Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(
                        trajectoryPoses.closePurpleFarPart1Red,
                        Math.toRadians(0) //Tangent
                )
                .splineToConstantHeading(
                        trajectoryPoses.closePurpleFarPart2Red,
                        Math.toRadians(-45) //Tangent
                )
                .build()
        );
        
        trajectorySequenceHashMap.put("Close Yellow (Far Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Purple (Far Detected)").end())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(
                        trajectoryPoses.closeYellowFarRed,
                        Math.toRadians(-90) //Tangent
                )
                .build()
        );


        //Bites
        trajectorySequenceHashMap.put("Drive back from stack Red", robot.autoDriveTrain.trajectorySequenceBuilder(get("Driving to stack (Close Detected) Red").end())
                .back(8,
                        trajectoryPoses.biteBackwardVelocity,
                        trajectoryPoses.biteBackwardAcceleration
                )
                .build()
        );
        trajectorySequenceHashMap.put("Drive back to stack Red", robot.autoDriveTrain.trajectorySequenceBuilder(get("Drive back from stack Red").end())
                .forward(8,
                        trajectoryPoses.biteForwardVelocity,
                        trajectoryPoses.biteForwardAcceleration
                )
                .build()
        );

        //Scoring Far
        trajectorySequenceHashMap.put("Go to backdrop (Far Side) Red", robot.autoDriveTrain.trajectorySequenceBuilder(get("Driving to stack (Close Detected) Red").end())
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(
                        trajectoryPoses.stackAndBackdropPart1Red,
                        Math.toRadians(-90) //Tangent
                )
                .splineToLinearHeading(
                       trajectoryPoses.stackAndBackdropPart2Red,
                        Math.toRadians(-90) //Tangent
                )
                .splineToLinearHeading(
                        trajectoryPoses.stackAndBackdropPart3Red,
                        Math.toRadians(-140) //Tangent
                )
                .build()
        );

        //Second Cycle
        trajectorySequenceHashMap.put("Back to stack (Second Cycle) Part 1 Red", robot.autoDriveTrain.trajectorySequenceBuilder(get("Go to backdrop (Far Side) Red").end())
                .setTangent(Math.toRadians(30))
                .splineToLinearHeading(
                        trajectoryPoses.stackAndBackdropPart2Red,
                        Math.toRadians(90) //Tangent
                )
                .splineToLinearHeading(
                        trajectoryPoses.stackAndBackdropPart1Red,
                        Math.toRadians(95) //Tangent
                )
                .build()
        );

        trajectorySequenceHashMap.put("Back to stack (Second Cycle) Part 2 Red", robot.autoDriveTrain.trajectorySequenceBuilder(get("Back to stack (Second Cycle) Part 1 Red").end())
                .splineToLinearHeading(
                        trajectoryPoses.aBitBeforeStackRed,
                        Math.toRadians(85) //Tangent
                )
                .splineToLinearHeading(
                        trajectoryPoses.stackPoseRed,
                        Math.toRadians(90), //Tangent
                        trajectoryPoses.beforeStackVelocityCycle,
                        trajectoryPoses.beforeStackAccelerationCycle
                )
                .build()
        );


        //Parking
        trajectorySequenceHashMap.put("Parking Arm To Intake (Far Side) Red", robot.autoDriveTrain.trajectorySequenceBuilder(get("Go to backdrop (Far Side) Red").end())
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(
                        trajectoryPoses.parkingFarPart1Red,
                        Math.toRadians(30) //Tangent
                )
                .splineToConstantHeading(
                        trajectoryPoses.parkingFarPart2Red,
                        Math.toRadians(180) //Tangent
                )
                .build()
        );

        trajectorySequenceHashMap.put("Backdrop Intake Close (Close Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Yellow (Close Detected) Red").end())
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(
                        trajectoryPoses.parkingCloseMiddleRed,
                        Math.toRadians(90) //Tangent
                )
                .build()
        );

        trajectorySequenceHashMap.put("Backdrop Intake Close", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Yellow (Far Detected)").end())
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(
                        trajectoryPoses.parkingCloseRed,
                        Math.toRadians(90) //Tangent
                )
                .build()
        );
    }

    public TrajectorySequence get(String trajectoryKey) {
        if (!trajectorySequenceHashMap.containsKey(trajectoryKey)) {
            throw new RuntimeException(trajectoryKey + " - Trajectory does not exist.");
        }
        return trajectorySequenceHashMap.get(trajectoryKey);
    }
}

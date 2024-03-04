package org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

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

    private final HashMap<String, TrajectorySequence> trajectorySequenceHashMap = new HashMap<>();
    private final RobotControl robot;
    private final TrajectoryPoses trajectoryPoses;

    public Trajectories(RobotControl robot, Pose2d startPose) {
        this.robot = robot;

        trajectoryPoses = new TrajectoryPoses(robot);

        //Purple Pixel Trajectories
        //Robot Far From Backdrop
        //Far From Truss Detected
        trajectorySequenceHashMap.put("Far Purple (Far Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(
                        trajectoryPoses.farPurpleFarPart1,
                        trajectoryPoses.getAngle(0) //Tangent
                )
                .splineToSplineHeading(
                        trajectoryPoses.farPurpleFarPart2,
                        trajectoryPoses.getAngle(0) //Tangent
                )
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack (Far Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Far Purple (Far Detected)").end())
                .setTangent(trajectoryPoses.getAngle(45))
                .splineToLinearHeading(
                        trajectoryPoses.stackPose,
                        trajectoryPoses.getAngle(45), //Tangent
                        trajectoryPoses.farStackVelocity,
                        trajectoryPoses.farStackAcceleration
                )
                .build()
        );

        //Center Detected
        trajectorySequenceHashMap.put("Far Purple (Center Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(
                        trajectoryPoses.farPurpleCenterPart1,
                        trajectoryPoses.getAngle(0) //Tangent
                )
                .splineToSplineHeading(
                        trajectoryPoses.farPurpleCenterPart2,
                        trajectoryPoses.getAngle(0)) //Tangent
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack (Center Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Far Purple (Center Detected)").end())
                .setTangent(trajectoryPoses.getAngle(40))
                .splineToSplineHeading(
                        trajectoryPoses.farStackCenter,
                        trajectoryPoses.getAngle(90), //Tangent
                        trajectoryPoses.centerStackVelocityPart1,
                        trajectoryPoses.centerStackAccelerationPart1
                )
                .splineToSplineHeading(
                        trajectoryPoses.stackPose,
                        trajectoryPoses.getAngle(90), //Tangent
                        trajectoryPoses.centerStackVelocityPart2,
                        trajectoryPoses.centerStackAccelerationPart2
                )
                .build()
        );

        //Close To Truss Detected
        trajectorySequenceHashMap.put("Far Purple (Close Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(
                        trajectoryPoses.farPurpleClosePart1,
                        trajectoryPoses.getAngle(0) //Tangent
                )
                .splineToConstantHeading(
                        trajectoryPoses.farPurpleClosePart2,
                        trajectoryPoses.getAngle(-45) //Tangent
                )
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack (Close Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Far Purple (Close Detected)").end())
                .setTangent(trajectoryPoses.getAngle(45))
                .splineToConstantHeading(
                        trajectoryPoses.farStackClose,
                        trajectoryPoses.getAngle(75) //Tangent
                )
                .splineToSplineHeading(
                        trajectoryPoses.stackPose,
                        trajectoryPoses.getAngle(45), //Tangent
                        trajectoryPoses.closeStackVelocity,
                        trajectoryPoses.closeStackAcceleration
                )
                .build()
        );

        //Robot Close To Backdrop Side]
        //Close to truss
        trajectorySequenceHashMap.put("Close Purple (Close Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(
                        trajectoryPoses.closePurpleClosePart1,
                        trajectoryPoses.getAngle(0) //Tangent
                )
                .splineToLinearHeading(
                        trajectoryPoses.closePurpleClosePart2,
                        trajectoryPoses.getAngle(90) //Tangent
                )
                .build()
        );

        trajectorySequenceHashMap.put("Close Yellow (Close Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Purple (Close Detected)").end())
                .setTangent(trajectoryPoses.getAngle(-45))
                .splineToSplineHeading(
                        trajectoryPoses.closeYellowClosePart1,
                        trajectoryPoses.getAngle(-90) //Tangent
                )
                .splineToConstantHeading(
                        trajectoryPoses.closeYellowClosePart2,
                        trajectoryPoses.getAngle(180) //Tangent
                )
                .build()
        );

        //Center
        trajectorySequenceHashMap.put("Close Purple (Center Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(
                        trajectoryPoses.closePurpleCenterPart1,
                        trajectoryPoses.getAngle(0) //Tangent
                )
                .splineToSplineHeading(
                        trajectoryPoses.closePurpleCenterPart2,
                        trajectoryPoses.getAngle(0) //Tangent
                )
                .build()
        );

        trajectorySequenceHashMap.put("Close Yellow (Center Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Purple (Center Detected)").end())
                .setTangent(trajectoryPoses.getAngle(180))
                .splineToLinearHeading(
                        trajectoryPoses.closeYellowCenter,
                        trajectoryPoses.getAngle(-90) //Tangent
                )
                .build()
        );

        //Far from truss
        trajectorySequenceHashMap.put("Close Purple (Far Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(
                        trajectoryPoses.closePurpleFarPart1,
                        trajectoryPoses.getAngle(0) //Tangent
                )
                .splineToConstantHeading(
                        trajectoryPoses.closePurpleFarPart2,
                        trajectoryPoses.getAngle(-45) //Tangent
                )
                .build()
        );
        
        trajectorySequenceHashMap.put("Close Yellow (Far Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Purple (Far Detected)").end())
                .setTangent(trajectoryPoses.getAngle(180))
                .splineToLinearHeading(
                        trajectoryPoses.closeYellowFar,
                        trajectoryPoses.getAngle(-90) //Tangent
                )
                .build()
        );


        //Bites
        trajectorySequenceHashMap.put("Drive back from stack", robot.autoDriveTrain.trajectorySequenceBuilder(get("Driving to stack (Close Detected)").end())
                .back(8,
                        trajectoryPoses.biteBackwardVelocity,
                        trajectoryPoses.biteBackwardAcceleration
                )
                .build()
        );
        trajectorySequenceHashMap.put("Drive back to stack", robot.autoDriveTrain.trajectorySequenceBuilder(get("Drive back from stack").end())
                .forward(8,
                        trajectoryPoses.biteForwardVelocity,
                        trajectoryPoses.biteForwardAcceleration
                )
                .build()
        );

        //Scoring Far
        trajectorySequenceHashMap.put("Go to backdrop (Far Side)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Driving to stack (Close Detected)").end())
                .setTangent(trajectoryPoses.getAngle(-90))
                .splineToSplineHeading(
                        trajectoryPoses.stackAndBackdropPart1,
                        trajectoryPoses.getAngle(-90) //Tangent
                )
                .splineToLinearHeading(
                       trajectoryPoses.stackAndBackdropPart2,
                        trajectoryPoses.getAngle(-90) //Tangent
                )
                .splineToLinearHeading(
                        trajectoryPoses.stackAndBackdropPart3,
                        trajectoryPoses.getAngle(-180) //Tangent
                )
                .build()
        );

        //Second Cycle
        trajectorySequenceHashMap.put("Back to stack (Second Cycle)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Go to backdrop (Far Side)").end())
                .setTangent(trajectoryPoses.getAngle(0))
                .splineToLinearHeading(
                        trajectoryPoses.stackAndBackdropPart2,
                        trajectoryPoses.getAngle(90) //Tangent
                )
                .splineToLinearHeading(
                        trajectoryPoses.stackAndBackdropPart1,
                        trajectoryPoses.getAngle(90) //Tangent
                )
                .splineToLinearHeading(
                        trajectoryPoses.aBitBeforeStack,
                        trajectoryPoses.getAngle(90) //Tangent
                )
                .splineToLinearHeading(
                        trajectoryPoses.stackPose,
                        trajectoryPoses.getAngle(90), //Tangent
                        trajectoryPoses.beforeStackVelocityCycle,
                        trajectoryPoses.beforeStackAccelerationCycle
                )
                .build()
        );

        //Parking
        trajectorySequenceHashMap.put("Parking Arm To Intake (Far Side)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Go to backdrop (Far Side)").end())
                .setTangent(trajectoryPoses.getAngle(0))
                .splineToConstantHeading(
                        trajectoryPoses.parkingFarPart1,
                        trajectoryPoses.getAngle(30) //Tangent
                )
                .splineToConstantHeading(
                        trajectoryPoses.parkingFarPart2,
                        trajectoryPoses.getAngle(180) //Tangent
                )
                .build()
        );

        trajectorySequenceHashMap.put("Backdrop Intake Close (Close Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Yellow (Close Detected)").end())
                .setTangent(trajectoryPoses.getAngle(90))
                .splineToConstantHeading(
                        trajectoryPoses.parkingCloseMiddle,
                        trajectoryPoses.getAngle(90) //Tangent
                )
                .build()
        );

        trajectorySequenceHashMap.put("Backdrop Intake Close", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Yellow (Far Detected)").end())
                .setTangent(trajectoryPoses.getAngle(90))
                .splineToConstantHeading(
                        trajectoryPoses.parkingClose,
                        trajectoryPoses.getAngle(90) //Tangent
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

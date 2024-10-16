package com.pathplanner.lib.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.missingWpilibClasses.Commands;
import com.pathplanner.lib.missingWpilibClasses.Pair;
import com.pathplanner.lib.missingWpilibClasses.math.geometry.Pose2d;
import com.pathplanner.lib.missingWpilibClasses.math.geometry.Rotation2d;
import com.pathplanner.lib.missingWpilibClasses.math.geometry.Translation2d;
import com.pathplanner.lib.missingWpilibClasses.math.kinematics.ChassisSpeeds;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Base command for following a path */
public class FollowPathCommand extends CommandBase {
  private final ElapsedTime timer = new ElapsedTime();
  private final PathPlannerPath originalPath;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedsSupplier;
  private final Consumer<ChassisSpeeds> output;
  private final PathFollowingController controller;
  private final ReplanningConfig replanningConfig;
  private final BooleanSupplier shouldFlipPath;

  // For event markers
  private final Map<Command, Boolean> currentEventCommands = new HashMap<>();
  private final List<Pair<Double, Command>> untriggeredEvents = new ArrayList<>();

  private PathPlannerPath path;
  private PathPlannerTrajectory generatedTrajectory;

  /**
   * Construct a base path following command
   *
   * @param path The path to follow
   * @param poseSupplier Function that supplies the current field-relative pose of the robot
   * @param speedsSupplier Function that supplies the current robot-relative chassis speeds
   * @param outputRobotRelative Function that will apply the robot-relative output speeds of this
   *     command
   * @param controller Path following controller that will be used to follow the path
   * @param replanningConfig Path replanning configuration
   * @param shouldFlipPath Should the path be flipped to the other side of the field? This will
   *     maintain a global blue alliance origin.
   * @param requirements Subsystems required by this command, usually just the drive subsystem
   */
  public FollowPathCommand(
      PathPlannerPath path,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      Consumer<ChassisSpeeds> outputRobotRelative,
      PathFollowingController controller,
      ReplanningConfig replanningConfig,
      BooleanSupplier shouldFlipPath,
      Subsystem... requirements) {
    this.originalPath = path;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    this.output = outputRobotRelative;
    this.controller = controller;
    this.replanningConfig = replanningConfig;
    this.shouldFlipPath = shouldFlipPath;

    Set<Subsystem> driveRequirements = Set.of(requirements);
    m_requirements.addAll(driveRequirements);

    for (EventMarker marker : this.originalPath.getEventMarkers()) {
      var reqs = marker.getCommand().getRequirements();

      if (!Collections.disjoint(driveRequirements, reqs)) {
        throw new IllegalArgumentException(
            "Events that are triggered during path following cannot require the drive subsystem");
      }

      m_requirements.addAll(reqs);
    }
  }

  @Override
  public void initialize() {
    if (shouldFlipPath.getAsBoolean() && !originalPath.preventFlipping) {
      path = originalPath.flipPath();
    } else {
      path = originalPath;
    }

    Pose2d currentPose = poseSupplier.get();
    ChassisSpeeds currentSpeeds = speedsSupplier.get();

    controller.reset(currentPose, currentSpeeds);

    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.getRotation());
    Rotation2d currentHeading =
        new Rotation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    Rotation2d targetHeading =
        path.getPoint(1).position.minus(path.getPoint(0).position).getAngle();
    Rotation2d headingError = currentHeading.minus(targetHeading);
    boolean onHeading =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond) < 0.25
            || Math.abs(headingError.getDegrees()) < 30;

    if (!path.isChoreoPath()
        && replanningConfig.enableInitialReplanning
        && (currentPose.getTranslation().getDistance(path.getPoint(0).position) > 0.25
            || !onHeading)) {
      replanPath(currentPose, currentSpeeds);
    } else {
      generatedTrajectory = path.getTrajectory(currentSpeeds, currentPose.getRotation());
      PathPlannerLogging.logActivePath(path);
    }

    // Initialize marker stuff
    currentEventCommands.clear();
    untriggeredEvents.clear();
    untriggeredEvents.addAll(generatedTrajectory.getEventCommands());

    timer.reset();
  }

  @Override
  public void execute() {
    double currentTime = timer.seconds();
    PathPlannerTrajectory.State targetState = generatedTrajectory.sample(currentTime);
    if (!controller.isHolonomic() && path.isReversed()) {
      targetState = targetState.reverse();
    }

    Pose2d currentPose = poseSupplier.get();
    ChassisSpeeds currentSpeeds = speedsSupplier.get();

    if (!path.isChoreoPath() && replanningConfig.enableDynamicReplanning) {
      double previousError = Math.abs(controller.getPositionalError());
      double currentError = currentPose.getTranslation().getDistance(targetState.positionMeters);

      if (currentError >= replanningConfig.dynamicReplanningTotalErrorThreshold
          || currentError - previousError
              >= replanningConfig.dynamicReplanningErrorSpikeThreshold) {
        replanPath(currentPose, currentSpeeds);
        timer.reset();
        targetState = generatedTrajectory.sample(0);
      }
    }

    ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);

    double currentVel =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

    PathPlannerLogging.logCurrentPose(currentPose);

    if (controller.isHolonomic()) {
      PathPlannerLogging.logTargetPose(targetState.getTargetHolonomicPose());
    } else {
      PathPlannerLogging.logTargetPose(targetState.getDifferentialPose());
    }

    output.accept(targetSpeeds);

    if (!untriggeredEvents.isEmpty() && timer.seconds() >= (untriggeredEvents.get(0).getFirst())) {
      // Time to trigger this event command
      Pair<Double, Command> event = untriggeredEvents.remove(0);

      for (var runningCommand : currentEventCommands.entrySet()) {
        if (!runningCommand.getValue()) {
          continue;
        }

        if (!Collections.disjoint(
            runningCommand.getKey().getRequirements(), event.getSecond().getRequirements())) {
          runningCommand.getKey().end(true);
          runningCommand.setValue(false);
        }
      }

      event.getSecond().initialize();
      currentEventCommands.put(event.getSecond(), true);
    }

    // Run event marker commands
    for (Map.Entry<Command, Boolean> runningCommand : currentEventCommands.entrySet()) {
      if (!runningCommand.getValue()) {
        continue;
      }

      runningCommand.getKey().execute();

      if (runningCommand.getKey().isFinished()) {
        runningCommand.getKey().end(false);
        runningCommand.setValue(false);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return timer.seconds() >= (generatedTrajectory.getTotalTimeSeconds());
  }

  @Override
  public void end(boolean interrupted) {

    // Only output 0 speeds when ending a path that is supposed to stop, this allows interrupting
    // the command to smoothly transition into some auto-alignment routine
    if (!interrupted && path.getGoalEndState().getVelocity() < 0.1) {
      output.accept(new ChassisSpeeds());
    }

    PathPlannerLogging.logActivePath(null);

    // End markers
    for (Map.Entry<Command, Boolean> runningCommand : currentEventCommands.entrySet()) {
      if (runningCommand.getValue()) {
        runningCommand.getKey().end(true);
      }
    }
  }

  private void replanPath(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
    PathPlannerPath replanned = path.replan(currentPose, currentSpeeds);
    generatedTrajectory = replanned.getTrajectory(currentSpeeds, currentPose.getRotation());
    PathPlannerLogging.logActivePath(replanned);
  }

  public static Command warmupCommand() {
    List<Translation2d> bezierPoints =
        PathPlannerPath.bezierFromPoses(
            new Pose2d(3.0, 3.0, new Rotation2d()), new Pose2d(6.0, 6.0, new Rotation2d()));
    PathPlannerPath path =
        new PathPlannerPath(
            bezierPoints,
            new PathConstraints(4.0, 4.0, 4.0, 4.0),
            new GoalEndState(0.0, Rotation2d.fromDegrees(90), true));

    return new FollowPathHolonomic(
            path,
            Pose2d::new,
            ChassisSpeeds::new,
            (speeds) -> {},
            new HolonomicPathFollowerConfig(4.5, 0.4, new ReplanningConfig()),
            () -> true)
        .andThen(Commands.print("[PathPlanner] FollowPathCommand finished warmup"));
  }
}

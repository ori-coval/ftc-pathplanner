package com.pathplanner.lib.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.pathplanner.lib.missingWpilibClasses.math.geometry.Pose2d;
import com.pathplanner.lib.path.PathPlannerPath;
import java.util.function.Supplier;

/** Command that will run a path following command and trigger event markers along the way */
public class FollowPathWithEvents extends CommandBase {
  private final Command pathFollowingCommand;

  /**
   * Constructs a new FollowPathWithEvents command.
   *
   * @param pathFollowingCommand the command to follow the path
   * @param path the path to follow
   * @param poseSupplier a supplier for the robot's current pose
   * @throws IllegalArgumentException if an event command requires the drive subsystem
   * @deprecated No longer needed, as the path following command will now handle events
   */
  public FollowPathWithEvents(
          Command pathFollowingCommand, PathPlannerPath path, Supplier<Pose2d> poseSupplier) {
    this.pathFollowingCommand = pathFollowingCommand;

    m_requirements.addAll(pathFollowingCommand.getRequirements());
  }

  @Override
  public void initialize() {
    pathFollowingCommand.initialize();
  }

  @Override
  public void execute() {
    pathFollowingCommand.execute();
  }

  @Override
  public boolean isFinished() {
    return pathFollowingCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    pathFollowingCommand.end(interrupted);
  }
}

package com.pathplanner.lib.auto;

import android.annotation.SuppressLint;
import android.content.Context;
import android.content.res.AssetManager;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;
import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.missingWpilibClasses.Commands;
import com.pathplanner.lib.missingWpilibClasses.math.geometry.Pose2d;
import com.pathplanner.lib.missingWpilibClasses.math.geometry.Rotation2d;
import com.pathplanner.lib.missingWpilibClasses.math.kinematics.ChassisSpeeds;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/** Utility class used to build auto routines */
public class AutoBuilder {
  private static boolean configured = false;

  private static Function<PathPlannerPath, Command> pathFollowingCommandBuilder;
  private static Supplier<Pose2d> getPose;
  private static Consumer<Pose2d> resetPose;
  private static BooleanSupplier shouldFlipPath;

  @SuppressLint("StaticFieldLeak")
  public static Context context;

  // Pathfinding builders
  private static boolean pathfindingConfigured = false;
  private static QuadFunction<Pose2d, PathConstraints, Double, Double, Command>
      pathfindToPoseCommandBuilder;
  private static TriFunction<PathPlannerPath, PathConstraints, Double, Command>
      pathfindThenFollowPathCommandBuilder;

  public static void setContext(Context context) {
    AutoBuilder.context = context.getApplicationContext();
  }

  /**
   * Configures the AutoBuilder for a holonomic drivetrain.
   *
   * @param poseSupplier a supplier for the robot's current pose
   * @param resetPose a consumer for resetting the robot's pose
   * @param robotRelativeSpeedsSupplier a supplier for the robot's current robot relative chassis
   *     speeds
   * @param robotRelativeOutput a consumer for setting the robot's robot-relative chassis speeds
   * @param config {@link com.pathplanner.lib.util.HolonomicPathFollowerConfig} for configuring the
   *     path following commands
   * @param shouldFlipPath Supplier that determines if paths should be flipped to the other side of
   *     the field. This will maintain a global blue alliance origin.
   * @param driveSubsystem the subsystem for the robot's drive
   */
  public static void configureHolonomic(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
      Consumer<ChassisSpeeds> robotRelativeOutput,
      HolonomicPathFollowerConfig config,
      BooleanSupplier shouldFlipPath,
      Subsystem driveSubsystem) {
    if (configured) {
      System.out.println("AutoBuilder is already configured");
    }
    AutoBuilder.pathFollowingCommandBuilder =
        (path) ->
            new FollowPathHolonomic(
                path,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                robotRelativeOutput,
                config,
                shouldFlipPath,
                driveSubsystem);
    AutoBuilder.getPose = poseSupplier;
    AutoBuilder.resetPose = resetPose;
    AutoBuilder.configured = true;
    AutoBuilder.shouldFlipPath = shouldFlipPath;

    AutoBuilder.pathfindToPoseCommandBuilder =
        (pose, constraints, goalEndVel, rotationDelayDistance) ->
            new PathfindHolonomic(
                pose,
                constraints,
                goalEndVel,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                robotRelativeOutput,
                config,
                rotationDelayDistance,
                driveSubsystem);
    AutoBuilder.pathfindThenFollowPathCommandBuilder =
        (path, constraints, rotationDelayDistance) ->
            new PathfindThenFollowPathHolonomic(
                path,
                constraints,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                robotRelativeOutput,
                config,
                rotationDelayDistance,
                shouldFlipPath,
                driveSubsystem);
    AutoBuilder.pathfindingConfigured = true;
  }

  /**
   * Configures the AutoBuilder with custom path following command builder. Building pathfinding
   * commands is not supported if using a custom command builder. Custom path following commands
   * will not have the path flipped for them, and event markers will not be triggered automatically.
   *
   * @param pathFollowingCommandBuilder a function that builds a command to follow a given path
   * @param poseSupplier a supplier for the robot's current pose
   * @param resetPose a consumer for resetting the robot's pose
   * @param shouldFlipPose Supplier that determines if the starting pose should be flipped to the
   *     other side of the field. This will maintain a global blue alliance origin. NOTE: paths will
   *     not be flipped when configured with a custom path following command. Flipping the paths
   *     must be handled in your command.
   */
  public static void configureCustom(
      Function<PathPlannerPath, Command> pathFollowingCommandBuilder,
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      BooleanSupplier shouldFlipPose) {
    if (configured) {
      System.out.println("AutoBuilder is already configured");
    }

    AutoBuilder.pathFollowingCommandBuilder = pathFollowingCommandBuilder;
    AutoBuilder.getPose = poseSupplier;
    AutoBuilder.resetPose = resetPose;
    AutoBuilder.configured = true;
    AutoBuilder.shouldFlipPath = shouldFlipPose;

    AutoBuilder.pathfindingConfigured = false;
  }

  /**
   * Configures the AutoBuilder with custom path following command builder. Building pathfinding
   * commands is not supported if using a custom command builder. Custom path following commands
   * will not have the path flipped for them, and event markers will not be triggered automatically.
   *
   * @param pathFollowingCommandBuilder a function that builds a command to follow a given path
   * @param poseSupplier a supplier for the robot's current pose
   * @param resetPose a consumer for resetting the robot's pose
   */
  public static void configureCustom(
      Function<PathPlannerPath, Command> pathFollowingCommandBuilder,
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose) {
    configureCustom(pathFollowingCommandBuilder, poseSupplier, resetPose, () -> false);
  }

  /**
   * Returns whether the AutoBuilder has been configured.
   *
   * @return true if the AutoBuilder has been configured, false otherwise
   */
  public static boolean isConfigured() {
    return configured;
  }

  /**
   * Returns whether the AutoBuilder has been configured for pathfinding.
   *
   * @return true if the AutoBuilder has been configured for pathfinding, false otherwise
   */
  public static boolean isPathfindingConfigured() {
    return pathfindingConfigured;
  }

  /**
   * Builds a command to follow a path with event markers.
   *
   * @param path the path to follow
   * @return a path following command with events for the given path
   * @throws AutoBuilderException if the AutoBuilder has not been configured
   * @deprecated Renamed to "followPath"
   */
  public static Command followPathWithEvents(PathPlannerPath path) {
    return followPath(path);
  }

  /**
   * Builds a command to follow a path. PathPlannerLib commands will also trigger event markers
   * along the way.
   *
   * @param path the path to follow
   * @return a path following command with for the given path
   * @throws AutoBuilderException if the AutoBuilder has not been configured
   */
  public static Command followPath(PathPlannerPath path) {
    if (!isConfigured()) {
      throw new AutoBuilderException(
          "Auto builder was used to build a path following command before being configured");
    }

    return pathFollowingCommandBuilder.apply(path);
  }

  /**
   * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
   * rotation and rotation delay distance will have no effect.
   *
   * @param pose The pose to pathfind to
   * @param constraints The constraints to use while pathfinding
   * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
   * @param rotationDelayDistance The distance the robot should move from the start position before
   *     attempting to rotate to the final rotation
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPose(
      Pose2d pose,
      PathConstraints constraints,
      double goalEndVelocity,
      double rotationDelayDistance) {
    if (!isPathfindingConfigured()) {
      throw new AutoBuilderException(
          "Auto builder was used to build a pathfinding command before being configured");
    }

    return pathfindToPoseCommandBuilder.apply(
        pose, constraints, goalEndVelocity, rotationDelayDistance);
  }

  /**
   * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
   * rotation will have no effect.
   *
   * @param pose The pose to pathfind to
   * @param constraints The constraints to use while pathfinding
   * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPose(
      Pose2d pose, PathConstraints constraints, double goalEndVelocity) {
    return pathfindToPose(pose, constraints, goalEndVelocity, 0);
  }

  /**
   * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
   * rotation will have no effect.
   *
   * @param pose The pose to pathfind to
   * @param constraints The constraints to use while pathfinding
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPose(Pose2d pose, PathConstraints constraints) {
    return pathfindToPose(pose, constraints, 0);
  }

  /**
   * Build a command to pathfind to a given pose that will be flipped based on the value of the path
   * flipping supplier when this command is run. If not using a holonomic drivetrain, the pose
   * rotation and rotation delay distance will have no effect.
   *
   * @param pose The pose to pathfind to. This will be flipped if the path flipping supplier returns
   *     true
   * @param constraints The constraints to use while pathfinding
   * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
   * @param rotationDelayDistance The distance the robot should move from the start position before
   *     attempting to rotate to the final rotation
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPoseFlipped(
      Pose2d pose,
      PathConstraints constraints,
      double goalEndVelocity,
      double rotationDelayDistance) {
    return Commands.either(
        pathfindToPose(
            GeometryUtil.flipFieldPose(pose), constraints, goalEndVelocity, rotationDelayDistance),
        pathfindToPose(pose, constraints, goalEndVelocity, rotationDelayDistance),
        shouldFlipPath);
  }

  /**
   * Build a command to pathfind to a given pose that will be flipped based on the value of the path
   * flipping supplier when this command is run. If not using a holonomic drivetrain, the pose
   * rotation and rotation delay distance will have no effect.
   *
   * @param pose The pose to pathfind to. This will be flipped if the path flipping supplier returns
   *     true
   * @param constraints The constraints to use while pathfinding
   * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPoseFlipped(
      Pose2d pose, PathConstraints constraints, double goalEndVelocity) {
    return pathfindToPoseFlipped(pose, constraints, goalEndVelocity, 0);
  }

  /**
   * Build a command to pathfind to a given pose that will be flipped based on the value of the path
   * flipping supplier when this command is run. If not using a holonomic drivetrain, the pose
   * rotation and rotation delay distance will have no effect.
   *
   * @param pose The pose to pathfind to. This will be flipped if the path flipping supplier returns
   *     true
   * @param constraints The constraints to use while pathfinding
   * @return A command to pathfind to a given pose
   */
  public static Command pathfindToPoseFlipped(Pose2d pose, PathConstraints constraints) {
    return pathfindToPoseFlipped(pose, constraints, 0);
  }

  /**
   * Build a command to pathfind to a given path, then follow that path. If not using a holonomic
   * drivetrain, the pose rotation delay distance will have no effect.
   *
   * @param goalPath The path to pathfind to, then follow
   * @param pathfindingConstraints The constraints to use while pathfinding
   * @param rotationDelayDistance The distance the robot should move from the start position before
   *     attempting to rotate to the final rotation
   * @return A command to pathfind to a given path, then follow the path
   */
  public static Command pathfindThenFollowPath(
      PathPlannerPath goalPath,
      PathConstraints pathfindingConstraints,
      double rotationDelayDistance) {
    if (!isPathfindingConfigured()) {
      throw new AutoBuilderException(
          "Auto builder was used to build a pathfinding command before being configured");
    }

    return pathfindThenFollowPathCommandBuilder.apply(
        goalPath, pathfindingConstraints, rotationDelayDistance);
  }

  /**
   * Build a command to pathfind to a given path, then follow that path.
   *
   * @param goalPath The path to pathfind to, then follow
   * @param pathfindingConstraints The constraints to use while pathfinding
   * @return A command to pathfind to a given path, then follow the path
   */
  public static Command pathfindThenFollowPath(
      PathPlannerPath goalPath, PathConstraints pathfindingConstraints) {
    return pathfindThenFollowPath(goalPath, pathfindingConstraints, 0);
  }


  /**
   * Get a list of all auto names in the project
   *
   * @return List of all auto names
   */
  public static List<String> getAllAutoNames() {
    AssetManager assetManager = context.getAssets();
    List<String> autoNames = new ArrayList<>();

    try {
      // List files in the "pathplanner/autos" directory within assets
      String[] autoFiles = assetManager.list("deploy/pathplanner/autos");
      if (autoFiles != null) {
        for (String file : autoFiles) {
          // Only add .auto files to the list
          if (file.endsWith(".auto")) {
            autoNames.add(file.substring(0, file.lastIndexOf(".")));
          }
        }
      }
    } catch (IOException e) {
      e.printStackTrace(); // Handle the exception
    }

    return autoNames;
  }

  /**
   * Get the starting pose from its JSON representation. This is only used internally.
   *
   * @param startingPoseJson JSON object representing a starting pose.
   * @return The Pose2d starting pose
   */
  public static Pose2d getStartingPoseFromJson(JSONObject startingPoseJson) {
    JSONObject pos = (JSONObject) startingPoseJson.get("position");
    double x = ((Number) pos.get("x")).doubleValue();
    double y = ((Number) pos.get("y")).doubleValue();
    double deg = ((Number) startingPoseJson.get("rotation")).doubleValue();

    return new Pose2d(x, y, Rotation2d.fromDegrees(deg));
  }

  /**
   * Builds an auto command for the given auto name.
   *
   * @param autoName the name of the auto to build
   * @return an auto command for the given auto name
   */
  public static Command buildAuto(String autoName) {
    try {
      // Access the AssetManager
      AssetManager assetManager = context.getAssets();

      // Open the input stream for the specified auto file
      InputStream inputStream = assetManager.open("deploy/pathplanner/autos/" + autoName + ".auto");
      BufferedReader br = new BufferedReader(new InputStreamReader(inputStream));

      StringBuilder fileContentBuilder = new StringBuilder();
      String line;
      while ((line = br.readLine()) != null) {
        fileContentBuilder.append(line);
      }

      String fileContent = fileContentBuilder.toString();
      JSONObject json = (JSONObject) new JSONParser().parse(fileContent);
      return getAutoCommandFromJson(json);
    } catch (Exception e) {
      throw new RuntimeException(String.format("Error building auto: %s", autoName), e);
    }
  }

  /**
   * Builds an auto command from the given JSON object.
   *
   * @param autoJson the JSON object to build the command from
   * @return an auto command built from the JSON object
   */
  public static Command getAutoCommandFromJson(JSONObject autoJson) {
    JSONObject commandJson = (JSONObject) autoJson.get("command");
    boolean choreoAuto = autoJson.get("choreoAuto") != null && (boolean) autoJson.get("choreoAuto");

    Command autoCommand = CommandUtil.commandFromJson(commandJson, choreoAuto);
    if (autoJson.get("startingPose") != null) {
      Pose2d startPose = getStartingPoseFromJson((JSONObject) autoJson.get("startingPose"));
      return Commands.sequence(
          Commands.runOnce(
              () -> {
                boolean flip = shouldFlipPath.getAsBoolean();
                if (flip) {
                  resetPose.accept(GeometryUtil.flipFieldPose(startPose));
                } else {
                  resetPose.accept(startPose);
                }
              }),
          autoCommand);
    } else {
      return autoCommand;
    }
  }

  /** Functional interface for a function that takes 3 inputs */
  @FunctionalInterface
  public interface TriFunction<In1, In2, In3, Out> {
    /**
     * Apply the inputs to this function
     *
     * @param in1 Input 1
     * @param in2 Input 2
     * @param in3 Input 3
     * @return Output
     */
    Out apply(In1 in1, In2 in2, In3 in3);
  }

  /** Functional interface for a function that takes 4 inputs */
  @FunctionalInterface
  public interface QuadFunction<In1, In2, In3, In4, Out> {
    /**
     * Apply the inputs to this function
     *
     * @param in1 Input 1
     * @param in2 Input 2
     * @param in3 Input 3
     * @param in4 Input 4
     * @return Output
     */
    Out apply(In1 in1, In2 in2, In3 in3, In4 in4);
  }
}

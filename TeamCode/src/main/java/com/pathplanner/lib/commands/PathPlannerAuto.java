package com.pathplanner.lib.commands;

import android.content.res.AssetManager;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.missingWpilibClasses.math.geometry.Pose2d;
import com.pathplanner.lib.path.PathPlannerPath;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/** A command that loads and runs an autonomous routine built using PathPlanner. */
public class PathPlannerAuto extends CommandBase {
  private static int instances = 0;

  private Command autoCommand;

  /**
   * Constructs a new PathPlannerAuto command.
   *
   * @param autoName the name of the autonomous routine to load and run
   * @throws RuntimeException if AutoBuilder is not configured before attempting to load the
   *     autonomous routine
   */
  public PathPlannerAuto(String autoName) {
    if (!AutoBuilder.isConfigured()) {
      throw new RuntimeException(
              "AutoBuilder was not configured before attempting to load a PathPlannerAuto from file");
    }

    this.autoCommand = AutoBuilder.buildAuto(autoName);
    m_requirements = autoCommand.getRequirements();
    setName(autoName);

    instances++;
  }

  /**
   * Get the starting pose from the given auto file
   *
   * @param autoName Name of the auto to get the pose from
   * @return Starting pose from the given auto
   */
  public static Pose2d getStartingPoseFromAutoFile(String autoName) {
    try {
      // Access the AssetManager
      AssetManager assetManager = AutoBuilder.context.getAssets();

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
      return AutoBuilder.getStartingPoseFromJson((JSONObject) json.get("startingPose"));
    } catch (Exception e) {
      throw new RuntimeException(e.getMessage());
    }
  }

  /**
   * Get a list of every path in the given auto (depth first)
   *
   * @param autoName Name of the auto to get the path group from
   * @return List of paths in the auto
   */
  public static List<PathPlannerPath> getPathGroupFromAutoFile(String autoName) {
    try {
      // Access the AssetManager
      AssetManager assetManager = AutoBuilder.context.getAssets();

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

      boolean choreoAuto = json.get("choreoAuto") != null && (boolean) json.get("choreoAuto");
      return pathsFromCommandJson((JSONObject) json.get("command"), choreoAuto);
    } catch (Exception e) {
      throw new RuntimeException(e.getMessage());
    }
  }

  /**
   * Reloads the autonomous routine with the given JSON object and updates the requirements of this
   * command.
   *
   * @param autoJson the JSON object representing the updated autonomous routine
   */
  public void hotReload(JSONObject autoJson) {
    autoCommand = AutoBuilder.getAutoCommandFromJson(autoJson);
    m_requirements = autoCommand.getRequirements();
  }

  @Override
  public void initialize() {
    autoCommand.initialize();
  }

  @Override
  public void execute() {
    autoCommand.execute();
  }

  @Override
  public boolean isFinished() {
    return autoCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    autoCommand.end(interrupted);
  }

  private static List<PathPlannerPath> pathsFromCommandJson(
          JSONObject commandJson, boolean choreoPaths) {
    List<PathPlannerPath> paths = new ArrayList<>();

    String type = (String) commandJson.get("type");
    JSONObject data = (JSONObject) commandJson.get("data");

    if (type.equals("path")) {
      String pathName = (String) data.get("pathName");
      if (choreoPaths) {
        paths.add(PathPlannerPath.fromChoreoTrajectory(pathName));
      } else {
        paths.add(PathPlannerPath.fromPathFile(pathName));
      }
    } else if (type.equals("sequential")
            || type.equals("parallel")
            || type.equals("race")
            || type.equals("deadline")) {
      for (Object cmdJson : (JSONArray) data.get("commands")) {
        paths.addAll(pathsFromCommandJson((JSONObject) cmdJson, choreoPaths));
      }
    }

    return paths;
  }
}
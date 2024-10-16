package com.pathplanner.lib.auto;

import com.arcrobotics.ftclib.command.Command;
import com.pathplanner.lib.missingWpilibClasses.Commands;
import com.pathplanner.lib.missingWpilibClasses.Pair;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Utility class for managing named commands */
public class NamedCommands {
  private static final HashMap<String, Command> namedCommands = new HashMap<>();

  /**
   * Registers a command with the given name.
   *
   * @param name the name of the command
   * @param command the command to register
   */
  public static void registerCommand(String name, Command command) {
    namedCommands.put(name, command);
  }

  /**
   * Registers a list of commands with their associated names.
   *
   * @param commands the list of commands to register
   */
  public static void registerCommands(List<Pair<String, Command>> commands) {
    for (var pair : commands) {
      registerCommand(pair.getFirst(), pair.getSecond());
    }
  }

  /**
   * Registers a map of commands with their associated names.
   *
   * @param commands the map of commands to register
   */
  public static void registerCommands(Map<String, Command> commands) {
    namedCommands.putAll(commands);
  }

  /**
   * Returns whether a command with the given name has been registered.
   *
   * @param name the name of the command to check
   * @return true if a command with the given name has been registered, false otherwise
   */
  public static boolean hasCommand(String name) {
    return namedCommands.containsKey(name);
  }

  /**
   * Returns the command with the given name.
   *
   * @param name the name of the command to get
   * @return the command with the given name, wrapped in a functional command, or a none command if
   *     it has not been registered
   */
  public static Command getCommand(String name) {
    if (hasCommand(name)) {
      return CommandUtil.wrappedEventCommand(namedCommands.get(name));
    } else {

      return Commands.none();
    }
  }
}

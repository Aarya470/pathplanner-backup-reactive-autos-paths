package frc.robot.commands;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.obstacle.GamePieceSettings;
import com.pathplanner.lib.obstacle.ObstacleSettings;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Dashboard controls for path regions (backup paths and game piece pickups), shown in Elastic next to the
 * Auton Type Chooser:
 *
 * <ul>
 *   <li>{@value #DIVERT_ENABLED_KEY} (Yes/No dropdown): No means no region ever diverts.
 *   <li>{@value #WAIT_ENABLED_KEY} (Yes/No dropdown): No means no timed wait at the end of a region.
 *   <li>{@value #PICKUP_ENABLED_KEY} (Yes/No dropdown): No means game piece regions are skipped.
 *   <li>{@value #DIVERT_OPTION_KEY}: which diverge path to use. Only used when diverting is on.
 *       "Path default" uses the diverge path saved on each region in the PathPlanner editor.
 * </ul>
 *
 * <p>The divert choices are the .path files in deploy/pathplanner/paths whose names start with
 * {@value #DIVERT_PREFIX}. To add one, save it in the editor with that name and deploy the code.
 */
public final class BackupPathOptions {
  public static final String DIVERT_ENABLED_KEY = "Backup/Divert Enabled";
  public static final String WAIT_ENABLED_KEY = "Backup/Wait Enabled";
  public static final String PICKUP_ENABLED_KEY = "Backup/Pickup Enabled";
  public static final String DIVERT_OPTION_KEY = "Backup/Divert Option";

  private static final String DIVERT_PREFIX = "Divert";
  private static final String PATH_DEFAULT = "Path default";

  private static final String YES = "Yes";
  private static final String NO = "No";

  private static SendableChooser<String> divertEnabledChooser;
  private static SendableChooser<String> waitEnabledChooser;
  private static SendableChooser<String> pickupEnabledChooser;
  private static SendableChooser<String> divertChooser;
  private static final Map<String, PathPlannerPath> loaded = new HashMap<>();

  private BackupPathOptions() {}

  /** Publish the controls and connect them to obstacle regions. Call once at startup. */
  public static void install() {
    divertEnabledChooser = yesNoChooser();
    SmartDashboard.putData(DIVERT_ENABLED_KEY, divertEnabledChooser);
    waitEnabledChooser = yesNoChooser();
    SmartDashboard.putData(WAIT_ENABLED_KEY, waitEnabledChooser);
    pickupEnabledChooser = yesNoChooser();
    SmartDashboard.putData(PICKUP_ENABLED_KEY, pickupEnabledChooser);

    divertChooser = new SendableChooser<>();
    divertChooser.setDefaultOption(PATH_DEFAULT, PATH_DEFAULT);
    for (String name : findDivertPathNames()) {
      divertChooser.addOption(name, name);
    }
    SmartDashboard.putData(DIVERT_OPTION_KEY, divertChooser);

    ObstacleSettings.setBackupOptions(
        () -> !NO.equals(divertEnabledChooser.getSelected()),
        () -> !NO.equals(waitEnabledChooser.getSelected()),
        BackupPathOptions::selectedDivertPath);
    GamePieceSettings.setEnabled(() -> !NO.equals(pickupEnabledChooser.getSelected()));
  }

  /** @return a Yes/No dropdown that starts on Yes */
  private static SendableChooser<String> yesNoChooser() {
    SendableChooser<String> chooser = new SendableChooser<>();
    chooser.setDefaultOption(YES, YES);
    chooser.addOption(NO, NO);
    return chooser;
  }

  /** @return the file names (without .path) of every divert path deployed with the code */
  private static List<String> findDivertPathNames() {
    List<String> names = new ArrayList<>();
    File[] files = new File(Filesystem.getDeployDirectory(), "pathplanner/paths").listFiles();
    if (files == null) {
      return names;
    }
    for (File file : files) {
      String fileName = file.getName();
      if (fileName.endsWith(".path") && fileName.startsWith(DIVERT_PREFIX)) {
        names.add(fileName.substring(0, fileName.length() - ".path".length()));
      }
    }
    Collections.sort(names);
    return names;
  }

  /**
   * @return the diverge path the driver picked, or null to use each region's own. A pick that
   *     fails to load also gives null, and the reason is reported to the Driver Station.
   */
  private static PathPlannerPath selectedDivertPath() {
    String name = divertChooser == null ? PATH_DEFAULT : divertChooser.getSelected();
    if (name == null || name.equals(PATH_DEFAULT)) {
      return null;
    }
    PathPlannerPath path = loaded.get(name);
    if (path == null) {
      try {
        path = PathPlannerPath.fromPathFile(name);
        loaded.put(name, path);
      } catch (Exception e) {
        DriverStation.reportWarning(
            "Divert path " + name + " failed to load, using the region's own: " + e.getMessage(),
            false);
      }
    }
    return path;
  }
}

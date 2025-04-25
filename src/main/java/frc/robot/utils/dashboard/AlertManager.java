package frc.robot.utils.dashboard;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.*;
import java.util.function.Predicate;

/**
 * A class for managing persistent alert notifications sent through NetworkTables. This class
 * handles the creation, grouping, and display of alerts with different severity levels.
 */
public class AlertManager {
  private static final Map<String, AlertGroup> alertGroups = new HashMap<>();

  private final AlertType alertType;
  private boolean isActive = false;
  private double activationTime = 0.0;
  private String alertMessage;

  /**
   * Creates a new alert in the default "Alerts" group. If this is the first instance, the
   * corresponding entry will be added to NetworkTables.
   *
   * @param alertMessage The text to display when the alert is active
   * @param alertType The severity level of the alert
   */
  public AlertManager(String alertMessage, AlertType alertType) {
    this("Alerts", alertMessage, alertType);
  }

  /**
   * Creates a new alert in the specified group. If this is the first instance in its group, the
   * corresponding entry will be added to NetworkTables.
   *
   * @param groupName The group identifier, also used as the NetworkTables title
   * @param alertMessage The text to display when the alert is active
   * @param alertType The severity level of the alert
   */
  public AlertManager(String groupName, String alertMessage, AlertType alertType) {
    if (!alertGroups.containsKey(groupName)) {
      alertGroups.put(groupName, new AlertGroup());
      SmartDashboard.putData(groupName, alertGroups.get(groupName));
    }

    this.alertMessage = alertMessage;
    this.alertType = alertType;
    alertGroups.get(groupName).alerts.add(this);
  }

  /**
   * Sets whether the alert should be currently displayed. When activated, the alert text will also
   * be sent to the console.
   *
   * @param isActive True to activate the alert, false to deactivate it
   */
  public void set(boolean isActive) {
    if (isActive && !this.isActive) {
      activationTime = Timer.getFPGATimestamp();
      switch (alertType) {
        case ERROR:
          DriverStation.reportError(alertMessage, false);
          break;
        case WARNING:
          DriverStation.reportWarning(alertMessage, false);
          break;
        case INFO:
          System.out.println(alertMessage);
          break;
      }
    }
    this.isActive = isActive;
  }

  /**
   * Updates the current alert text. If the alert is active, the new message will be reported to the
   * appropriate output based on the alert type.
   *
   * @param alertMessage The new text to display for this alert
   */
  public void setText(String alertMessage) {
    if (isActive && !alertMessage.equals(this.alertMessage)) {
      switch (alertType) {
        case ERROR:
          DriverStation.reportError(alertMessage, false);
          break;
        case WARNING:
          DriverStation.reportWarning(alertMessage, false);
          break;
        case INFO:
          System.out.println(alertMessage);
          break;
      }
    }
    this.alertMessage = alertMessage;
  }

  /**
   * Internal class representing a group of alerts that can be sent to NetworkTables. Implements
   * Sendable interface for dashboard integration.
   */
  private static class AlertGroup implements Sendable {
    public final List<AlertManager> alerts = new ArrayList<>();

    /**
     * Retrieves all active alert messages of a specific type, sorted by activation time.
     *
     * @param alertType The type of alerts to retrieve
     * @return Array of alert messages, sorted by most recent first
     */
    public String[] getAlertMessages(AlertType alertType) {
      Predicate<AlertManager> activeFilter =
          (AlertManager x) -> x.alertType == alertType && x.isActive;
      Comparator<AlertManager> timeSorter =
          (AlertManager a1, AlertManager a2) -> (int) (a2.activationTime - a1.activationTime);
      return alerts.stream()
          .filter(activeFilter)
          .sorted(timeSorter)
          .map((AlertManager a) -> a.alertMessage)
          .toArray(String[]::new);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Alerts");
      builder.addStringArrayProperty("errors", () -> getAlertMessages(AlertType.ERROR), null);
      builder.addStringArrayProperty("warnings", () -> getAlertMessages(AlertType.WARNING), null);
      builder.addStringArrayProperty("infos", () -> getAlertMessages(AlertType.INFO), null);
    }
  }

  /**
   * Enum representing the severity level of an alert. Each level has a specific visual
   * representation on the dashboard.
   */
  public enum AlertType {
    /**
     * High priority alert - displayed first on the dashboard with a red "X" symbol. Used for issues
     * that severely impact robot functionality and require immediate attention.
     */
    ERROR,

    /**
     * Medium priority alert - displayed second on the dashboard with a yellow "!" symbol. Used for
     * issues that may affect robot functionality but don't necessarily require immediate attention.
     */
    WARNING,

    /**
     * Low priority alert - displayed last on the dashboard with a green "i" symbol. Used for issues
     * that are unlikely to affect robot functionality, or any other alerts that don't fit into the
     * ERROR or WARNING categories.
     */
    INFO
  }
}

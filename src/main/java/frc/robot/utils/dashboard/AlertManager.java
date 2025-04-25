package frc.robot.utils.dashboard;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.*;
import java.util.function.Predicate;

/** 用于管理通过NetworkTables发送的持久警报通知的类 */
public class AlertManager {
  private static final Map<String, AlertGroup> alertGroups = new HashMap<>();

  private final AlertType alertType;
  private boolean isActive = false;
  private double activationTime = 0.0;
  private String alertMessage;

  /**
   * 在默认组"Alerts"中创建一个新的警报。如果是第一个实例， 相应的条目将被添加到NetworkTables中。
   *
   * @param alertMessage 警报激活时显示的文本
   * @param alertType 指定紧急程度的警报级别
   */
  public AlertManager(String alertMessage, AlertType alertType) {
    this("Alerts", alertMessage, alertType);
  }

  /**
   * 创建一个新的警报。如果是其组中的第一个实例， 相应的条目将被添加到NetworkTables中。
   *
   * @param groupName 组标识符，也用作NetworkTables标题
   * @param alertMessage 警报激活时显示的文本
   * @param alertType 指定紧急程度的警报级别
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

  /** 设置警报是否应该当前显示。当激活时， 警报文本也将被发送到控制台。 */
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

  /** 更新当前警报文本 */
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

  private static class AlertGroup implements Sendable {
    public final List<AlertManager> alerts = new ArrayList<>();

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

  /** 表示警报的紧急程度级别 */
  public enum AlertType {
    /** 高优先级警报 - 在仪表盘上以红色"X"符号首先显示。 用于严重影响机器人功能并需要立即关注的问题。 */
    ERROR,

    /** 中等优先级警报 - 在仪表盘上以黄色"!"符号第二显示。 用于可能影响机器人功能但不一定需要立即关注的问题。 */
    WARNING,

    /** 低优先级警报 - 在仪表盘上以绿色"i"符号最后显示。 用于不太可能影响机器人功能的问题，或任何不属于"ERROR"或"WARNING"的其他警报。 */
    INFO
  }
}

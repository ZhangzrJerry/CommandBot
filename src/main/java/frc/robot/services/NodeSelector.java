package frc.robot.services;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.ReefScape.GamePiece;
import frc.robot.interfaces.services.Service;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;
import java.nio.file.Paths;
import java.util.concurrent.atomic.AtomicBoolean;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class NodeSelector implements Service {
  @AutoLog
  public static class NodeSelectorIOInputs {
    public String gamepieceType = ""; // "A"(algae) or "C"(coral)
    public String branch = ""; // "A" -> "L"
    public String level = ""; // "1" -> "4"
    public boolean isIgnoreArmMoveCondition = false;
    public boolean isConnected = false;
    public String errorMessage = "";
  }

  private final NodeSelectorIOInputs inputs = new NodeSelectorIOInputs();
  private String errorMessage = "";

  // NetworkTables publishers and subscribers
  private final StringPublisher nodePublisher;
  private final StringSubscriber nodeSubscriber;
  private final BooleanSubscriber isIgnoreArmMoveConditionSubscriber;
  private final BooleanPublisher isIgnoreArmMoveConditionPublisher;

  private final IntegerPublisher timePublisher;
  private final BooleanPublisher isAutoPublisher;
  private final BooleanPublisher isConnectedPublisher;
  private final StringPublisher errorMessagePublisher;

  private final AtomicBoolean isNewCommand = new AtomicBoolean(false);
  private final AtomicBoolean isServerConnected = new AtomicBoolean(false);
  private Javalin server;
  private ServiceState state = ServiceState.STOPPED;

  public NodeSelector() {
    var table = NetworkTableInstance.getDefault().getTable("nodeselector");
    nodePublisher = table.getStringTopic("node_robot_2_dashboard").publish();
    nodeSubscriber = table.getStringTopic("node_dashboard_2_robot").subscribe("");
    isIgnoreArmMoveConditionSubscriber =
        table.getBooleanTopic("is_ignore_arm_move_condition").subscribe(false);
    isIgnoreArmMoveConditionPublisher =
        table.getBooleanTopic("is_ignore_arm_move_condition_robot_2_dashboard").publish();
    timePublisher = table.getIntegerTopic("match_time").publish();
    isAutoPublisher = table.getBooleanTopic("is_auto").publish();
    isConnectedPublisher = table.getBooleanTopic("is_connected").publish();
    errorMessagePublisher = table.getStringTopic("error_message").publish();
  }

  @Override
  public ServiceState getState() {
    return state;
  }

  @Override
  public void setState(ServiceState state) {
    this.state = state;
    if (state != ServiceState.ERROR) {
      errorMessage = "";
      errorMessagePublisher.set("");
    }
  }

  @Override
  public String getErrorMessage() {
    return errorMessage;
  }

  @Override
  public void setError(String errorMessage) {
    this.errorMessage = errorMessage;
    errorMessagePublisher.set(errorMessage);
    setState(ServiceState.ERROR);
  }

  @Override
  public void init() {
    try {
      server =
          Javalin.create(
              config ->
                  config.staticFiles.add(
                      Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "nodeselector")
                          .toString(),
                      Location.EXTERNAL));
      server.start(5800);
      isServerConnected.set(true);
      setState(ServiceState.RUNNING);
    } catch (Exception e) {
      String error = "Failed to start server: " + e.getMessage();
      errorMessage = error;
      errorMessagePublisher.set(error);
      setError(error);
    }
  }

  @Override
  public void update() {
    try {
      timePublisher.set((long) Math.floor(Math.max(0.0, DriverStation.getMatchTime())));
      isAutoPublisher.set(DriverStation.isAutonomous());
      isConnectedPublisher.set(isServerConnected.get());

      for (var val : isIgnoreArmMoveConditionSubscriber.readQueueValues()) {
        inputs.isIgnoreArmMoveCondition = val;
        isIgnoreArmMoveConditionPublisher.set(val);
      }

      for (var val : isIgnoreArmMoveConditionSubscriber.readQueueValues()) {
        inputs.isIgnoreArmMoveCondition = val;
        isIgnoreArmMoveConditionPublisher.set(val);
      }

      var selectedNode = "";
      for (var val : nodeSubscriber.readQueueValues()) {
        selectedNode = val;
      }

      if (selectedNode.length() < 2) {
        isNewCommand.set(false);
        return;
      }

      isNewCommand.set(true);
      inputs.gamepieceType = "" + selectedNode.charAt(0);
      inputs.branch = "" + selectedNode.charAt(1);
      inputs.level = inputs.gamepieceType.equals("A") ? "" : "" + selectedNode.charAt(2);
      inputs.isConnected = isServerConnected.get();
      inputs.errorMessage = "";
    } catch (Exception e) {
      String error = "Update error: " + e.getMessage();
      inputs.errorMessage = error;
      setError(error);
      isNewCommand.set(false);
    }
    Logger.recordOutput("Services/Node Selector/GamepieceType", inputs.gamepieceType);
    Logger.recordOutput("Services/Node Selector/Branch", inputs.branch);
    Logger.recordOutput("Services/Node Selector/Level", inputs.level);
    Logger.recordOutput(
        "Services/Node Selector/IsIgnoreArmMoveCondition", inputs.isIgnoreArmMoveCondition);
    Logger.recordOutput("Services/Node Selector/ErrorMessage", inputs.errorMessage);
  }

  public void setSelected(GamePiece.Type gamePieceType, String branch, String level) {
    if (!isConnected()) {
      String error = "Cannot set selection - server not connected";
      setError(error);
      return;
    }
    try {
      if (gamePieceType == GamePiece.Type.ALGAE) {
        nodePublisher.set("A" + branch);
      } else {
        nodePublisher.set("C" + branch + level);
      }
    } catch (Exception e) {
      String error = "Set selected error: " + e.getMessage();
      setError(error);
    }
  }

  public void setIgnoreArmMoveCondition(boolean isIgnoreArmMoveCondition) {
    if (!isConnected()) {
      String error = "Cannot set ignore condition - server not connected";
      setError(error);
      return;
    }
    try {
      isIgnoreArmMoveConditionPublisher.set(isIgnoreArmMoveCondition);
      inputs.isIgnoreArmMoveCondition = isIgnoreArmMoveCondition;
    } catch (Exception e) {
      String error = "Set ignore condition error: " + e.getMessage();
      setError(error);
    }
  }

  public boolean getIgnoreArmMoveCondition() {
    return inputs.isIgnoreArmMoveCondition;
  }

  public GamePiece.Type getSelectedGamePieceType() {
    if (!inputs.gamepieceType.isEmpty()) {
      return inputs.gamepieceType.equals("A") ? GamePiece.Type.ALGAE : GamePiece.Type.CORAL;
    }
    return null;
  }

  public String getSelectedBranch() {
    return inputs.branch;
  }

  public String getSelectedLevel() {
    return inputs.level;
  }

  public boolean isIgnoreArmMoveCondition() {
    return inputs.isIgnoreArmMoveCondition;
  }

  public boolean hasValidSelection() {
    return !inputs.branch.isEmpty() && !inputs.gamepieceType.isEmpty();
  }

  @AutoLogOutput(key = "Services/Node Selector/hasNewCommand")
  public boolean hasNewCommand() {
    return isNewCommand.get();
  }

  @Override
  public String getName() {
    return "Node Selector";
  }

  public void reset() {
    isNewCommand.set(false);
    try {
      nodePublisher.set("");
      errorMessagePublisher.set("");
    } catch (Exception e) {
      String error = "Reset error: " + e.getMessage();
      setError(error);
    }
  }

  public boolean isConnected() {
    return isServerConnected.get();
  }
}

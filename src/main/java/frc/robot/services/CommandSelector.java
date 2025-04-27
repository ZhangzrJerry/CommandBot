package frc.robot.services;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.interfaces.services.Service;
import frc.robot.utils.dashboard.SwitchableChooser;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class CommandSelector implements Service {
  private static final int MAX_QUESTIONS = 3;
  private static final CommandMode DEFAULT_MODE =
      new CommandMode("Silence", List.of(), rs -> Commands.none());
  @Getter private final String name;

  public record CommandMode(
      String name, List<CommandQuestion> questions, Function<List<String>, Command> modeBuilder) {}

  public record CommandQuestion(String question, List<String> options) {}

  private final LoggedDashboardChooser<CommandMode> modeChooser;
  private final List<StringPublisher> questionPublishers;
  private final List<SwitchableChooser> questionChoosers;

  private CommandMode lastMode = DEFAULT_MODE;
  private List<String> lastResponses = List.of();

  public CommandSelector(String name, String key) {
    this.name = name;
    modeChooser = new LoggedDashboardChooser<>(key + "/Mode");
    modeChooser.addDefaultOption(DEFAULT_MODE.name, DEFAULT_MODE);

    // Publish questions and option choosers
    questionPublishers = new ArrayList<>();
    questionChoosers = new ArrayList<>();
    for (int i = 0; i < MAX_QUESTIONS; i++) {
      var publisher =
          NetworkTableInstance.getDefault()
              .getStringTopic("/SmartDashboard/" + key + "/Question #" + (i + 1))
              .publish();
      publisher.set("NA");
      questionPublishers.add(publisher);
      questionChoosers.add(new SwitchableChooser(key + "/Question #" + (i + 1) + " Chooser"));
    }
  }

  @Override
  public void update() {
    // ServiceManager should pause this service during autonomous, so no need for
    // special handling here.
    var selectedMode = modeChooser.get();
    if (selectedMode == null) {
      return;
    }

    // When the mode is switched, update questions and options
    if (!selectedMode.equals(lastMode)) {
      System.out.println("Command mode switched -> " + selectedMode.name);
      var questions = selectedMode.questions();
      for (int i = 0; i < MAX_QUESTIONS; i++) {
        if (i < questions.size()) {
          questionPublishers.get(i).set(questions.get(i).question());
          questionChoosers.get(i).setOptions(questions.get(i).options().toArray(String[]::new));
        } else {
          questionPublishers.get(i).set("");
          questionChoosers.get(i).setOptions(new String[] {});
        }
      }
    }

    // Collect current responses
    var responses = new ArrayList<String>();
    for (int i = 0; i < selectedMode.questions().size(); i++) {
      var response = questionChoosers.get(i).get();
      responses.add(response == null ? selectedMode.questions().get(i).options().get(0) : response);
    }

    lastMode = selectedMode;
    lastResponses = responses;
    Logger.recordOutput("CommandSelector/Responses", lastResponses.toArray(String[]::new));
  }

  /** Add a mode with a fixed command (no questions). */
  public void addMode(String name, Command mode) {
    addMode(name, rs -> mode);
  }

  /** Add a mode with a command builder and no questions. */
  public void addMode(String name, Function<List<String>, Command> modeBuilder) {
    addMode(name, List.of(), modeBuilder);
  }

  /** Add a mode with questions and a command builder. */
  public void addMode(
      String name, List<CommandQuestion> questions, Function<List<String>, Command> modeBuilder) {
    modeChooser.addOption(name, new CommandMode(name, questions, modeBuilder));
  }

  /** Get the command built from the current mode and responses. */
  public Command getCmd() {
    System.out.println("Command mode built " + lastMode.name);
    return lastMode.modeBuilder.apply(lastResponses);
  }

  @AutoLogOutput(key = "CommandSelector/Is Silence")
  public BooleanSupplier isSilenceMode() {
    return () -> DEFAULT_MODE.equals(modeChooser.get());
  }
}

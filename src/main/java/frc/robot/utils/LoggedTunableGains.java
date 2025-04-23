package frc.robot.utils;

import frc.robot.Config;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Tunable gains class similar to LoggedTunableNumber */
public class LoggedTunableGains<T extends GainsUtil.Gains> {
  private static final String TABLE_KEY = "/Tuning";

  private final String fullKey;
  private final T defaultValue;
  private final LoggedNetworkNumber[] networkNumbers;
  private final Map<Integer, T> lastHasChangedValues = new HashMap<>();
  private static final Map<String, LoggedNetworkNumber[]> existingEntries = new HashMap<>();

  public LoggedTunableGains() {
    this("default", null);
  }

  public LoggedTunableGains(String subKey, T defaultValue) {
    this.fullKey = TABLE_KEY + "/" + subKey;
    this.defaultValue = defaultValue;

    if (Config.IS_LIVE_DEBUG) {
      // Check if the key already exists in the existing entries
      if (existingEntries.containsKey(fullKey)) {
        networkNumbers = existingEntries.get(fullKey);
      } else {
        String[] keys = defaultValue.getKeys();
        double[] values = defaultValue.getValues();
        networkNumbers = new LoggedNetworkNumber[keys.length];

        for (int i = 0; i < keys.length; i++) {
          networkNumbers[i] = new LoggedNetworkNumber(fullKey + "/" + keys[i], values[i]);
        }
        existingEntries.put(fullKey, networkNumbers);
      }
    } else {
      networkNumbers = null;
    }
  }

  /** Get current gains (tuned values if in debug mode) */
  @SuppressWarnings("unchecked")
  public T get() {
    if (!Config.IS_LIVE_DEBUG || networkNumbers == null) {
      return defaultValue;
    }

    double[] currentValues = new double[networkNumbers.length];
    for (int i = 0; i < networkNumbers.length; i++) {
      currentValues[i] = networkNumbers[i].get();
    }

    return (T) createGains(defaultValue.getClass(), currentValues);
  }

  /** Check if gains have changed since last call */
  public boolean hasChanged(int id) {
    T currentValue = get();
    T lastValue = lastHasChangedValues.get(id);

    if (lastValue == null || !currentValue.equals(lastValue)) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }
    return false;
  }

  /** Factory method to create appropriate gains type */
  private GainsUtil.Gains createGains(Class<?> type, double[] values) {
    if (type == GainsUtil.PidGains.class) {
      return new GainsUtil.PidGains(values[0], values[1], values[2]);
    } else if (type == GainsUtil.PdsGains.class) {
      return new GainsUtil.PdsGains(values[0], values[1], values[2]);
    } else if (type == GainsUtil.PidsgGains.class) {
      return new GainsUtil.PidsgGains(values[0], values[1], values[2], values[3], values[4]);
    }
    throw new IllegalArgumentException("Unknown gains type: " + type);
  }

  /** Static helper for multiple gain checks (similar to LoggedTunableNumber) */
  @SafeVarargs
  public static void ifChanged(int id, Runnable action, LoggedTunableGains<?>... gainsArray) {
    for (LoggedTunableGains<?> gains : gainsArray) {
      if (gains.hasChanged(id)) {
        action.run();
        return;
      }
    }
  }
}

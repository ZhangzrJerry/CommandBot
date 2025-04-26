package frc.robot.utils.dashboard;

import frc.robot.utils.Gains;
import java.util.Map;

public class TunableGains {

  public static final String KP_KEY = "kP";
  public static final String KI_KEY = "kI";
  public static final String KD_KEY = "kD";
  public static final String KS_KEY = "kS";
  public static final String KG_KEY = "kG";

  public static class TunablePidsgGains extends TunableNumbers implements Gains {

    public TunablePidsgGains(
        String baseKey, double kP, double kI, double kD, double kS, double kG) {
      super(baseKey, Map.of(KP_KEY, kP, KI_KEY, kI, KD_KEY, kD, KS_KEY, kS, KG_KEY, kG));
    }

    public TunablePidsgGains(String baseKey, Gains gains) {
      super(
          baseKey,
          Map.of(
              KP_KEY,
              gains.getKP(),
              KI_KEY,
              gains.getKI(),
              KD_KEY,
              gains.getKD(),
              KS_KEY,
              gains.getKS(),
              KG_KEY,
              gains.getKG()));
    }

    public double getKP() {
      return get(KP_KEY);
    }

    public double getKI() {
      return get(KI_KEY);
    }

    public double getKD() {
      return get(KD_KEY);
    }

    public double getKS() {
      return get(KS_KEY);
    }

    public double getKG() {
      return get(KG_KEY);
    }
  }

  public static class TunablePidGains extends TunableNumbers implements Gains {

    public TunablePidGains(String baseKey, double kP, double kI, double kD) {
      super(baseKey, Map.of(KP_KEY, kP, KI_KEY, kI, KD_KEY, kD));
    }

    public double getKP() {
      return get(KP_KEY);
    }

    public double getKI() {
      return get(KI_KEY);
    }

    public double getKD() {
      return get(KD_KEY);
    }
  }
}

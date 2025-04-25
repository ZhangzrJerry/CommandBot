package frc.robot.utils;

public interface Gains {

    public class GainsImpl implements Gains {
        public GainsImpl(double kP, double kI, double kD, double kS, double kG) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kS = kS;
            this.kG = kG;
        }

        private double kP;
        private double kI;
        private double kD;
        private double kS;
        private double kG;

        @Override
        public double getKP() {
            return kP;
        }

        @Override
        public double getKI() {
            return kI;
        }

        @Override
        public double getKD() {
            return kD;
        }

        @Override
        public double getKS() {
            return kS;
        }

        @Override
        public double getKG() {
            return kG;
        }
    }

    double getKP();

    default double getKI() {
        return 0;
    }

    default double getKD() {
        return 0;
    }

    default double getKS() {
        return 0;
    }

    default double getKG() {
        return 0;
    }
}

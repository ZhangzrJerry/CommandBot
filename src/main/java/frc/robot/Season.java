package frc.robot;

import edu.wpi.first.math.util.Units;
import lombok.RequiredArgsConstructor;

public class Season {
    public static class GamePiece {
        public static enum Type {
            ALGAE,
            CORAL
        }

        public static class Algae {
            public static final double DIAMETER = Units.inchesToMeters(16.0);
        }

        public static class Coral {
            public static final double LENGTH = Units.inchesToMeters(11.0 + 7.0 / 8.0);
            public static final double DIAMETER = Units.inchesToMeters(4.0);
        }
    }

    public static class Field {
        public static class Reef {

        }

        public static class Barge {

        }

        public static class Processor {

        }
    }
}

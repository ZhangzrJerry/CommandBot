package frc.robot;

import frc.robot.interfaces.hardwares.CanDevice;

public final class Constants {
  public static final double LOOP_PERIOD_SEC = 0.02;
  public static final boolean IS_LIVE_DEBUG = true;

  public final class Ascope {
    public static final class Component {
      public static final int ROBOT_FRAME = -1;
      public static final int SWERVE_FL = 0;
      public static final int SWERVE_BL = 1;
      public static final int SWERVE_BR = 2;
      public static final int SWERVE_FR = 3;
      public static final int ELEVATOR_L2 = 4;
      public static final int ELEVATOR_L3 = 5;
      public static final int ELEVATOR_CARRIAGE = 6;
      public static final int ARM = 7;
      public static final int INTAKE = 8;
      public static final int CLIMBER = 9;
      public static final int ALGAE = 10;
      public static final int CORAL = 11;
    }
  }

  public final class Ports {
    public static final class Can {
      public static final String RIO_BUS = "";
      public static final String CHASSIS_CANIVORE_BUS = "chassis";
      public static final String SUPERSTRUCTURE_CANIVORE_BUS = "super";

      // Led
      public static final CanDevice CANDLE = new CanDevice(0, RIO_BUS);

      // IMU
      public static final CanDevice CHASSIS_PIGEON = new CanDevice(0, CHASSIS_CANIVORE_BUS);

      // Swerve
      public static final CanDevice FL_DRIVE_MOTOR = new CanDevice(1, CHASSIS_CANIVORE_BUS);
      public static final CanDevice FL_STEER_MOTOR = new CanDevice(2, CHASSIS_CANIVORE_BUS);
      public static final CanDevice FL_STEER_SENSOR = new CanDevice(3, CHASSIS_CANIVORE_BUS);

      public static final CanDevice BL_DRIVE_MOTOR = new CanDevice(4, CHASSIS_CANIVORE_BUS);
      public static final CanDevice BL_STEER_MOTOR = new CanDevice(5, CHASSIS_CANIVORE_BUS);
      public static final CanDevice BL_STEER_SENSOR = new CanDevice(6, CHASSIS_CANIVORE_BUS);

      public static final CanDevice BR_DRIVE_MOTOR = new CanDevice(7, CHASSIS_CANIVORE_BUS);
      public static final CanDevice BR_STEER_MOTOR = new CanDevice(8, CHASSIS_CANIVORE_BUS);
      public static final CanDevice BR_STEER_SENSOR = new CanDevice(9, CHASSIS_CANIVORE_BUS);

      public static final CanDevice FR_DRIVE_MOTOR = new CanDevice(10, CHASSIS_CANIVORE_BUS);
      public static final CanDevice FR_STEER_MOTOR = new CanDevice(11, CHASSIS_CANIVORE_BUS);
      public static final CanDevice FR_STEER_SENSOR = new CanDevice(12, CHASSIS_CANIVORE_BUS);

      // Ground Intake
      public static final CanDevice ALGAE_GROUND_INTAKE_ROLLER =
          new CanDevice(2, SUPERSTRUCTURE_CANIVORE_BUS);
      public static final CanDevice ALGAE_GROUND_INTAKE_ARM =
          new CanDevice(3, SUPERSTRUCTURE_CANIVORE_BUS);

      // End Effector
      public static final CanDevice CORAL_END_EFFECTOR =
          new CanDevice(4, SUPERSTRUCTURE_CANIVORE_BUS);
      public static final CanDevice ALGAE_END_EFFECTOR =
          new CanDevice(5, SUPERSTRUCTURE_CANIVORE_BUS);

      // Arm
      public static final CanDevice ARM_ELBOW = new CanDevice(8, SUPERSTRUCTURE_CANIVORE_BUS);
      public static final CanDevice ARM_ELBOW_CANCODER =
          new CanDevice(9, SUPERSTRUCTURE_CANIVORE_BUS);
      public static final CanDevice ARM_CANDI = new CanDevice(11, SUPERSTRUCTURE_CANIVORE_BUS);

      // Climber
      public static final CanDevice CLIMBER = new CanDevice(12, RIO_BUS);

      // Elevator
      public static final CanDevice ARM_SHOULDER_MASTER = new CanDevice(13, CHASSIS_CANIVORE_BUS);
      public static final CanDevice ARM_SHOULDER_SLAVE = new CanDevice(14, CHASSIS_CANIVORE_BUS);
    }

    public static final class Digital {
      public static final int CORAL_STAGE = 1;
      public static final int ALGAE_STAGE = 0;
    }

    public static final class Joystick {
      public static final int DRIVER = 0;
    }

    private Ports() {}
  }

  public static final class Components {
    public static final int ROBOT_FRAME = -1;
    public static final int SWERVE_FL = 0;
    public static final int SWERVE_BL = 1;
    public static final int SWERVE_BR = 2;
    public static final int SWERVE_FR = 3;
    public static final int ELEVATOR_L2 = 4;
    public static final int ELEVATOR_L3 = 5;
    public static final int ELEVATOR_CARRIAGE = 6;
    public static final int ARM = 7;
    public static final int INTAKE = 8;
    public static final int CLIMBER = 9;
    public static final int ALGAE = 10;
    public static final int CORAL = 11;

    private Components() {}
  }

  private Constants() {}
}

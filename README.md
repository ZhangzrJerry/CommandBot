
- Commands
  - SwerveCommands
  - ArmCommands
  - ...
  - JoystickHumbleCommand

- Virtual Subsystems
  - Odometry *handles* SwerveSignal, VisionSignal
  - Visualizer *handles* SwerveSignal, ArmSignal, IntakeSignal, ClimberSignal, VisionSignal

- Subsystems

  - swerve
    - Swerve *owns* SwerveModules, GyroIO, GyroIOInputs, SwerveController
    - SwerveModule *owns* DCMotorIO, DCMotorIOInputs
    - SwerveController (interface)

  - Arm *owns* DCMotorIO, DCMotorIOInputs
  - Intake *owns* DCMotorIO, DCMotorIOInputs
  - Climber *owns* DCMotorIO, DCMotorIOInputs
  - EndEffector *owns* DCMotorIO, DCMotorIOInputs
  - Vision

- Drivers

  - dcmotor
    - DCMotorIO (interface) *updates* DCMotorIOInputs
    - DCMotorIOInputs
    - DCMotorIOSim *implements* DCMotorIO *updates* DCMotorIOInputs
    - DCMotorIOKraken *implements* DCMotorIO *updates* DCMotorIOInputs
    - DCMotorIOKrakenCancoder *extends* DCMotorIOKraken *updates* DCMotorIOInputs
  - gyro
    - GyroIO (interface) *updates* GyroIOInputs
    - GyroIOInputs
    - GyroIOPigeon2 *implements* GyroIO *updates* GyroIOInputs

- Hardware
  - WPILib
  - CTRE Phoenix
  - REVLib
  - PhotonVision

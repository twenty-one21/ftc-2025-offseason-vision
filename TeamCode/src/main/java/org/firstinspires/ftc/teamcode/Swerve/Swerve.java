// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Swerve;

import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.function.BooleanSupplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.MathUtil;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.kinematics.SwerveDriveKinematics;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.kinematics.SwerveModuleState;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.math.controller.SimpleMotorFeedforward;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.math.filter.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.util.Units;
import org.firstinspires.ftc.teamcode.Utils;

public class Swerve {

  private final GoBildaPinpointDriver odometry;
  private GoBildaPinpointDriver.DeviceStatus odometryStatus;

  private final SwerveDriveKinematics kinematics;
  private final double drivebaseRadius;

  public final Module[] modules = new Module[4];

  private final Telemetry telemetry;

  private final double speedMult;
  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter yawLimiter;

  private double lastHeading = 0;

  public Swerve(OpMode opMode) {
    odometry = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    odometry.setOffsets(110, 30);
    odometry.setEncoderResolution(goBILDA_4_BAR_POD);
    odometry.setEncoderDirections(FORWARD, FORWARD);

    double trackLengthMeters = Units.inchesToMeters(12.25);
    double trackWidthMeters = Units.inchesToMeters(14.5);
    kinematics =
        new SwerveDriveKinematics(
            new Translation2d(trackLengthMeters / 2, trackWidthMeters / 2),
            new Translation2d(trackLengthMeters / 2, -trackWidthMeters / 2),
            new Translation2d(-trackLengthMeters / 2, trackWidthMeters / 2),
            new Translation2d(-trackLengthMeters / 2, -trackWidthMeters / 2));
    drivebaseRadius = Math.hypot(trackLengthMeters / 2, trackWidthMeters / 2);

    for (int i = 0; i < 4; i++) {
      modules[i] = new Module(opMode, i);
    }

    odometry.update();
    odometryStatus = odometry.getDeviceStatus();

    this.telemetry = opMode.telemetry;

    speedMult = .6;
    double timeToFull = .5;

    xLimiter = new SlewRateLimiter((Module.maxDriveSpeedMetersPerSec * speedMult) / timeToFull);
    yLimiter = new SlewRateLimiter((Module.maxDriveSpeedMetersPerSec * speedMult) / timeToFull);
    yawLimiter =
        new SlewRateLimiter(
            ((Module.maxDriveSpeedMetersPerSec / drivebaseRadius) * speedMult) / timeToFull);
  }

  double maxErrorDeg = 1;

  public void drive(ChassisSpeeds speeds, double dt) {
    var translationalMagnitude = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    if (translationalMagnitude > Module.maxDriveSpeedMetersPerSec) {
      speeds.vxMetersPerSecond *= Module.maxDriveSpeedMetersPerSec / translationalMagnitude;
      speeds.vyMetersPerSecond *= Module.maxDriveSpeedMetersPerSec / translationalMagnitude;

      translationalMagnitude = Module.maxDriveSpeedMetersPerSec;
    }
    speeds.omegaRadiansPerSecond *=
        MathUtil.interpolate(
            1,
            .5,
            MathUtil.inverseInterpolate(
                0, Module.maxDriveSpeedMetersPerSec, translationalMagnitude));

    speeds = ChassisSpeeds.discretize(speeds, dt);

    var scalar = Math.cos(Units.degreesToRadians(maxErrorDeg));
    speeds =
        new ChassisSpeeds(
            xLimiter.calculate(speeds.vxMetersPerSecond * scalar),
            yLimiter.calculate(speeds.vyMetersPerSecond * scalar),
            yawLimiter.calculate(speeds.omegaRadiansPerSecond * scalar));

    var setpoint = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpoint, Module.maxDriveSpeedMetersPerSec * speedMult);

    maxErrorDeg = 1;
    for (int i = 0; i < 4; i++) {
      maxErrorDeg = Math.max(modules[i].run(setpoint[i]), maxErrorDeg);
    }
  }

  public void fieldRelativeDrive(ChassisSpeeds speeds, double dt) {
    var yaw =
        odometryStatus == GoBildaPinpointDriver.DeviceStatus.READY
            ? odometry.getHeading()
            : new Rotation2d();
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, yaw), dt);
    telemetry.addData("Swerve/Yaw", yaw.getDegrees());
  }

  public void teleopDrive(double xInput, double yInput, double yawInput, double dt) {
    var translationalMagnitude = Math.hypot(xInput, yInput);
    if (translationalMagnitude > 1) {
      xInput /= translationalMagnitude;
      yInput /= translationalMagnitude;
      translationalMagnitude = 1;
    }

    var rotationalScalar = MathUtil.interpolate(1, .5, translationalMagnitude);

    fieldRelativeDrive(
        new ChassisSpeeds(
            xInput * Module.maxDriveSpeedMetersPerSec * speedMult,
            yInput * Module.maxDriveSpeedMetersPerSec * speedMult,
            yawInput
                * (Module.maxDriveSpeedMetersPerSec
                    * speedMult
                    * rotationalScalar
                    / drivebaseRadius)),
        dt);

    if (yawInput == 0 && odometry.getHeading().getRadians() != lastHeading)
      telemetry.addData("Unwanted rotation: ", true);
    else telemetry.addData("Unwanted rotation: ", false);
    lastHeading = odometry.getHeading().getRadians();
  }

  public Pose2d getPose() {
    return odometry.getPose();
  }

  public void initGyro() {
    odometry.resetPosAndIMU();
    try {
      Thread.sleep((long) (.25 * 1e3));
    } catch (final InterruptedException ex) {
      Thread.currentThread().interrupt();
    }
  }

  public void alignWheels(BooleanSupplier opModeActiveSupplier) {
    var state = new SwerveModuleState();
    while (opModeActiveSupplier.getAsBoolean()) {
      double maxErrorDeg = 0;
      for (var module : modules) {
        module.run(state);

        maxErrorDeg =
            Math.max(
                maxErrorDeg,
                Math.min(
                    Rotation2d.kZero.minus(module.getServoPos()).getDegrees(),
                    Rotation2d.kPi.minus(module.getServoPos()).getDegrees()));
      }
      if (maxErrorDeg < 5) {
        return;
      }
    }
  }

  public void periodic() {
    odometry.update();
    odometryStatus = odometry.getDeviceStatus();
    telemetry.addData(
        "Swerve/Pinpoint status",
        odometryStatus == GoBildaPinpointDriver.DeviceStatus.READY ? "OK" : odometryStatus.name());
  }

  public static final class Module {
    private static final double conversionFactor;

    String pos = "";
    static final double maxDriveSpeedMetersPerSec;
    static final double maxSteerSpeedRadPerSec;

    // TODO figure this shit out(mostly done)
    static double countsPerRevolution = 537.7;
    static double gearRatio = 1.1;
    static double wheelCircumferenceMeters = (96.0 / 1000.0) * Math.PI;
    static double maxMotorVelocity = 436.0 / 60.0;

    static {
      conversionFactor = countsPerRevolution * gearRatio / wheelCircumferenceMeters;
      maxDriveSpeedMetersPerSec = (maxMotorVelocity / gearRatio) * wheelCircumferenceMeters;

      double maxSpeedSecondsPer60Degrees = .14 * .863;
      maxSteerSpeedRadPerSec = (2 * Math.PI) / (maxSpeedSecondsPer60Degrees * 6);
    }

    final DcMotorEx driveMotor;
    final Servo steerServo;
    final AnalogInput steerEncoder;

    final PIDController drivePID;
    final SimpleMotorFeedforward driveFeedforward;

    final PIDController steerPID;
    final SimpleMotorFeedforward steerFeedforward;

    Telemetry telemetry;
    int id;

    double kp = 5.5, ki = 0.1, kd = 0.1;

    Module(OpMode opMode, int id) {

      steerPID = new PIDController(kp, ki, kd);
      switch (id) {
        case 0 -> {
          pos = "FL";
          countsPerRevolution = 537.0;
        }
        case 1 -> {
          pos = "FR";
          countsPerRevolution = 537.7;
        }
        case 2 -> {
          pos = "BL";
          countsPerRevolution = 538.0;
        }
        case 3 -> {
          pos = "BR";
          countsPerRevolution = 537.0;
        }
        default -> throw new IllegalArgumentException("Module ID is out of range 0-3!");
      }

      driveMotor = (DcMotorEx) opMode.hardwareMap.dcMotor.get(pos + "Motor");
      steerServo = opMode.hardwareMap.servo.get(pos + "Servo");
      steerEncoder = opMode.hardwareMap.analogInput.get(pos + "Encoder");

      if (pos.equals("BL") || pos.equals("BR")) {
        driveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      }

      drivePID = new PIDController(0 / maxDriveSpeedMetersPerSec, 0, 0);
      driveFeedforward = new SimpleMotorFeedforward(0, 1 / maxDriveSpeedMetersPerSec);

      steerPID.enableContinuousInput(-Math.PI, Math.PI);
      steerFeedforward = new SimpleMotorFeedforward(0, 1 / maxSteerSpeedRadPerSec);

      this.telemetry = opMode.telemetry;
      this.id = id;
    }

    double run(SwerveModuleState state) {
      var servoPos = getServoPos();
      state.optimize(servoPos);
      state.cosineScale(servoPos);

      driveMotor.setPower(
          driveFeedforward.calculate(state.speedMetersPerSecond)
              + drivePID.calculate(getDriveVelocity(), state.speedMetersPerSecond));
      telemetry.addData("motor speed: ", driveMotor.getPower());

      var errorDeg = state.angle.minus(servoPos).getDegrees();
      telemetry.addData("Swerve/Module " + id + "/Angle error (Deg)", errorDeg);
      if (!MathUtil.isNear(0, errorDeg, .5)) {
        runServoVel(steerPID.calculate(getServoPos().getRadians(), state.angle.getRadians()));
      } else {
        runServoVel(0);
      }

      return errorDeg;
    }

    public double getDrivePosition() {
      return driveMotor.getCurrentPosition() / conversionFactor;
    }

    private double lastPos;
    private double lastTime = -1;

    // We calculate motor velocity ourselves because REV sucks and only calculates velocity at 20
    // hz.
    public double getDriveVelocity() {
      if (lastTime == -1) {
        lastPos = getDrivePosition();
        lastTime = Utils.getTimeSeconds();
        return 0;
      }
      var currentPos = getDrivePosition();
      var currentTime = Utils.getTimeSeconds();
      var velocity = (currentPos - lastPos) / (currentTime - lastTime);
      lastPos = currentPos;
      lastTime = currentTime;
      return velocity;
    }

    public Rotation2d getServoPos() {
      return new Rotation2d(
          MathUtil.angleModulus(
              (Math.PI * 2) * (steerEncoder.getVoltage() / steerEncoder.getMaxVoltage())));
    }

    private void runServoVel(double velRadPerSec) {
      steerServo.setPosition((1 - steerFeedforward.calculate(velRadPerSec)) / 2);
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/* Project that I'm working from:
  https://github.com/frc2377/CTRE-Sim/blob/master/src/main/java/frc/robot/subsystems/DriveSubsystem.java#L98
 */

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonSRX m_front_left = new WPI_TalonSRX(Constants.kDrivetrainFrontLeftMotor);
  private final WPI_TalonSRX m_back_left = new WPI_TalonSRX(Constants.kDrivetrainBackLeftMotor);
  private final WPI_TalonSRX m_front_right = new WPI_TalonSRX(Constants.kDrivetrainFrontRightMotor);
  private final WPI_TalonSRX m_back_right = new WPI_TalonSRX(Constants.kDrivetrainBackRightMotor);

  private final SpeedControllerGroup m_left = new SpeedControllerGroup(m_front_left, m_back_left);
  private final SpeedControllerGroup m_right = new SpeedControllerGroup(m_front_right, m_back_right);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  private final AHRS m_navx = new AHRS();
  private final DifferentialDriveOdometry m_odometry;

  private DifferentialDrivetrainSim m_driveSim;
  private Field2d m_field;
  private final TalonSRXSimCollection m_leftSim = new TalonSRXSimCollection(m_front_left);
  private final TalonSRXSimCollection m_rightSim = new TalonSRXSimCollection(m_front_right);


  /** Creates a new Drivetrain. */
  public Drivetrain() {

    m_front_left.setNeutralMode(NeutralMode.Brake);
    m_front_right.setNeutralMode(NeutralMode.Brake);
    m_back_left.setNeutralMode(NeutralMode.Brake);
    m_back_right.setNeutralMode(NeutralMode.Brake);

    m_back_left.setInverted(true);
    m_front_left.setInverted(true);
    m_back_right.setInverted(true);
    m_front_right.setInverted(true);

    m_drive.setRightSideInverted(false);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    if (RobotBase.isSimulation()) {
      m_driveSim = new DifferentialDrivetrainSim(
              DCMotor.getCIM(2),
              Constants.kDrivetrainGearing,
              Constants.kMomentOfInertia,
              Constants.kRobotMass,
              Units.inchesToMeters(Constants.kWheelDiameter),
              Constants.kTrackWidth,
              null
      );
      m_field = new Field2d();
      SmartDashboard.putData("Field", m_field);
    }
  }

  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), ctreUnitsToDistanceMeters(m_front_left.getSelectedSensorPosition()), ctreUnitsToDistanceMeters(m_front_right.getSelectedSensorPosition()));
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    m_driveSim.setInputs(m_front_left.getMotorOutputVoltage(), m_front_right.getMotorOutputVoltage());
    m_driveSim.update(Constants.kLoopTime);
    m_leftSim.setQuadratureRawPosition((int)distanceToCTRENativeUnits(m_driveSim.getLeftPositionMeters()));
    m_leftSim.setQuadratureVelocity((int)velocityToCTRENativeUnits(m_driveSim.getLeftVelocityMetersPerSecond()));
    m_rightSim.setQuadratureRawPosition((int)distanceToCTRENativeUnits(m_driveSim.getRightPositionMeters()));
    m_rightSim.setQuadratureVelocity((int)velocityToCTRENativeUnits(m_driveSim.getRightVelocityMetersPerSecond()));
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(m_driveSim.getHeading().getDegrees());

    SmartDashboard.putNumber("DriveSim Angle Value", m_driveSim.getHeading().getDegrees());
    SmartDashboard.putNumber("NavX Value", m_navx.getAngle());
  }

  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  public double getHeading() {
    return m_navx.getAngle();
  }

  public double ctreUnitsToDistanceMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / 2048; //Encoder CPR
    double wheelRotations = motorRotations * Constants.kDrivetrainGearing;
    double positionMeters = wheelRotations * (Math.PI * Constants.kWheelDiameter);
    return positionMeters;
  }

  public double distanceToCTRENativeUnits(double positionMeters) {
    double wheelRotations = positionMeters / (Math.PI * Constants.kWheelDiameter);
    double motorRotations = wheelRotations / Constants.kDrivetrainGearing;
    double sensorCounts = (motorRotations * 2048); // Encoder CPR
    return sensorCounts;
  }

  public double velocityToCTRENativeUnits(double velocityMetersPerSecond) {
    double wheelRotationsPerSecond = velocityMetersPerSecond / (Math.PI * Constants.kWheelDiameter);
    double motorRotationsPerSecond = wheelRotationsPerSecond / Constants.kDrivetrainGearing;
    double motorRotationsPer100ms = motorRotationsPerSecond / 10;
    double sensorCountsPer100ms = (motorRotationsPer100ms * 2048); //Encoder CPR
    return sensorCountsPer100ms;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GyroSim;
import frc.robot.Constants;

import java.util.Random;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX m_left_front = new WPI_TalonFX(Constants.kDrivetrainFrontLeftMotor);
  private final WPI_TalonFX m_left_back = new WPI_TalonFX(Constants.kDrivetrainBackLeftMotor);
  private final WPI_TalonFX m_right_front = new WPI_TalonFX(Constants.kDrivetrainFrontRightMotor);
  private final WPI_TalonFX m_right_back = new WPI_TalonFX(Constants.kDrivetrainBackRightMotor);

  private final SpeedControllerGroup m_left = new SpeedControllerGroup(m_left_front, m_left_back);
  private final SpeedControllerGroup m_right = new SpeedControllerGroup(m_right_front, m_right_back);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  private final AHRS m_navx = new AHRS();
  private final DifferentialDriveOdometry m_odometry;

  private DifferentialDrivetrainSim m_driveSim;
  private GyroSim m_gyroSim;
  private Encoder m_leftEncoder;
  private Encoder m_rightEncoder;
  private EncoderSim m_leftSim;
  private EncoderSim m_rightSim;
  private Field2d m_field;
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    m_drive.setRightSideInverted(false);

    if (RobotBase.isSimulation()) {
      m_leftEncoder = new Encoder(0, 1);
      m_rightEncoder = new Encoder(2, 3);
      m_leftSim = new EncoderSim(m_leftEncoder);
      m_rightSim = new EncoderSim(m_rightEncoder);

      m_leftEncoder.setDistancePerPulse(4.601942364E-4);
      m_rightEncoder.setDistancePerPulse(4.601942364E-4);
//3.897449094E-5 old distance per pulse
      m_gyroSim = new GyroSim("NavX");

      m_driveSim = new DifferentialDrivetrainSim(
              DCMotor.getFalcon500(2),
              Constants.kDrivetrainGearing,
              Constants.kMomentOfInertia + 3,
              Constants.kRobotMass - 50,
              Units.inchesToMeters(Constants.kWheelDiameter),
              Constants.kTrackWidth,
              null
      );


      m_field = new Field2d();
      m_odometry = new DifferentialDriveOdometry(m_gyroSim.getHeading(), new Pose2d(new Translation2d(5, 5), new Rotation2d()));
      SmartDashboard.putData("Field", m_field);
    } else {
      m_odometry = new DifferentialDriveOdometry(new Rotation2d(m_navx.getAngle()));
    }
  }

  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  @Override
  public void periodic() {
    if (RobotBase.isSimulation()) {
      m_odometry.update(m_gyroSim.getHeading(), m_leftSim.getDistance(), m_rightSim.getDistance());
    } else {
      m_odometry.update(new Rotation2d(m_navx.getAngle()), m_left.get(), m_right.get());
    }
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    m_driveSim.setInputs(m_left.get(), m_right.get());
    m_driveSim.update(Constants.kLoopTime);
    m_leftSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setHeading(m_driveSim.getHeading());
    SmartDashboard.putNumber("Avg, Speed", (m_driveSim.getLeftVelocityMetersPerSecond() + m_driveSim.getRightVelocityMetersPerSecond() / 2));
  }
}

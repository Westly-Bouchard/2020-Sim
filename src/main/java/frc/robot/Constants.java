// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int kDrivetrainFrontLeftMotor = 1;
    public static final int kDrivetrainFrontRightMotor = 2;
    public static final int kDrivetrainBackLeftMotor = 3;
    public static final int kDrivetrainBackRightMotor = 4;

    public static final double kDrivetrainGearing = (11.0/62) * (24.0/54) * 1.057;
    public static final int kRobotMass = 57;
    public static final double kWheelDiameter = 0.1524; //meters
    public static final double kMomentOfInertia = 3; //jKgMetersSquared
    public static final double kTrackWidth = 0.60658;

    public static final double kLoopTime = 0.02;

    public static final double kDrivetrainTeleOpFrictionDeadband = 0.2;
    public static final double kDrivetrainTeleOpFrictionAmount = 1.0 / 3;
}

package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;


public class TankDrive extends CommandBase {
    private final Drivetrain drivetrain;
    private double left;
    private double right;

    public TankDrive(Drivetrain drivetrain, Supplier<Double> left, Supplier<Double> right, boolean inverted) {
        this.drivetrain = drivetrain;
        this.left = left.get();
        this.right = right.get();
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
//        if(inverted) {
//            if (!invertDriveWasPressed) {
//                invert_driving *= -1;
//            }
//            invertDriveWasPressed = true;
//        } else {
//            invertDriveWasPressed = false;
//        }
//        drivetrain.setCurvedTeleopSpeed(invert_driving * left, invert_driving * right);
        drivetrain.tankDrive(left, right);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

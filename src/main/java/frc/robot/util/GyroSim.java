package frc.robot.util;


import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class GyroSim {
    private final SimDouble heading;

    public GyroSim(String name) {
        SimDevice device = SimDevice.create("Gyro[" + name + "]");
        heading = device.createDouble(name, SimDevice.Direction.kOutput, 0);
    }

    public void setHeading(Rotation2d heading) {
        this.heading.set(heading.getRadians());
    }

    public Rotation2d getHeading() {
        return new Rotation2d(heading.get());
    }
}

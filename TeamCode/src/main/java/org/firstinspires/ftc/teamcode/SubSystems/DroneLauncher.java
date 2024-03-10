package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class DroneLauncher extends SubsystemBase {
    private final Servo drone;

    public DroneLauncher(HardwareMap hardwareMap) {
        drone = hardwareMap.servo.get(Configuration.DRONE);
    }

    public void setPosition(double position) {
        drone.setPosition(position);
    }

    public double getPosition() {
        return drone.getPosition();
    }


}

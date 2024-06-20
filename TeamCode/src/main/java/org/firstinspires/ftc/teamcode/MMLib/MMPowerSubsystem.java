package org.firstinspires.ftc.teamcode.MMLib;

import com.arcrobotics.ftclib.command.SubsystemBase;

public abstract class MMPowerSubsystem<T> extends SubsystemBase {

    public abstract void setPower(T power);

}

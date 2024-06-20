package org.firstinspires.ftc.teamcode.MMLib;

import com.arcrobotics.ftclib.command.SubsystemBase;

public abstract class MMPositionSubsystem<T> extends SubsystemBase {

    public abstract void setPosition(T position);

}

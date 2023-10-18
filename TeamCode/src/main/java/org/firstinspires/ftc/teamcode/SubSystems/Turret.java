package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Turret extends SubsystemBase {
    DcMotor turretMotor;
    public Turret(DcMotor turretMotor){
        this.turretMotor = turretMotor;
    }

}

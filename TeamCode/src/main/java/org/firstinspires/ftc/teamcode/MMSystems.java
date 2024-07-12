
package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.devices.MMRevHub;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.ShooterIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ShooterPID;
import org.firstinspires.ftc.teamcode.SubSystems.ShooterTurret;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

/**
 * this class should help u contain all ur robot
 */
public class MMSystems {

    public OpModeType opModeType;
    public AllianceColor allianceColor;
    public AllianceSide robotSide;
    public HardwareMap hardwareMap;
    public MMRevHub controlHub;
    public MMRevHub expansionHub;
    public GamepadEx gamepadEx1;
    public GamepadEx gamepadEx2;
    public Telemetry telemetry;


    //Subsystems
    public Shooter shooter;
    public ShooterIntake shooterIntake;
    public ShooterPID shooterPID;
    public ShooterTurret shooterTurret;


}

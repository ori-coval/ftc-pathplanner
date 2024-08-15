
package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.devices.MMRevHub;
import org.firstinspires.ftc.teamcode.MMLib.Examples.ElevatorExample.ExampleElevator;
import org.firstinspires.ftc.teamcode.MMLib.Utils.MMBattery;
import org.firstinspires.ftc.teamcode.MMLib.Examples.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.MMLib.Examples.Subsystems.ShooterIntake;
import org.firstinspires.ftc.teamcode.MMLib.Examples.Subsystems.ShooterPID;
import org.firstinspires.ftc.teamcode.MMLib.Examples.Subsystems.ShooterTurret;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

/**
 * this class should contain all ur robot's attributes and systems
 */
public class MMSystems {

    //Attributes & Hardware
    public OpModeType opModeType;
    public AllianceColor allianceColor;
    public AllianceSide robotSide;
    public HardwareMap hardwareMap;
    public MMRevHub controlHub;
    public MMRevHub expansionHub;
    public GamepadEx gamepadEx1;
    public GamepadEx gamepadEx2;
    public Telemetry telemetry;
    public MMBattery battery;


    //Subsystems
    //For example:
    public Shooter shooter;
    public ShooterIntake shooterIntake;
    public ShooterPID shooterPID;
    public ShooterTurret shooterTurret;
    public ExampleElevator exampleElevator;


}

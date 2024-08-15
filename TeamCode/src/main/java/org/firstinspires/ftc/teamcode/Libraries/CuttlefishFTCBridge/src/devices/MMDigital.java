package org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices;

/**
 * Generic digital sensor
 * */
public class MMDigital {
    public int port;
    public MMRevHub hub;
    /**
     * @param revHub
     * @param digitalPort
     * */
    public MMDigital(MMRevHub revHub, int digitalPort)
    {
        hub = revHub;
        port = digitalPort;
    }
    /**
     * Get the state of the digital sensor.
     * */
    public boolean getState()
    {
        return hub.bulkData.getDigitalInput(port);
    }
}

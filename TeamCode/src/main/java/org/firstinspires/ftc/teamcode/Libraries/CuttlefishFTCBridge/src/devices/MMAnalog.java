package org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices;

/**
 * Generic Analog Sensor
 * */
public class MMAnalog {
    public int port;
    public MMRevHub hub;

    /**
     * Generic analog sensor.
     * @param revHub
     * @param analogPort Analog port number
     * */
    public MMAnalog(MMRevHub revHub, int analogPort)
    {
        hub = revHub;
        port = analogPort;
    }

    /**
     * Get the voltage of the sensor in volts
     * */
    public double getVoltage()
    {
        return ((double)hub.bulkData.getAnalogInput(port))/1000.0;
    }

    /**
     * Get the voltage of the sensor in millivolts
     * */
    public int getVoltageMillivolts()
    {
        return hub.bulkData.getAnalogInput(port);
    }
}

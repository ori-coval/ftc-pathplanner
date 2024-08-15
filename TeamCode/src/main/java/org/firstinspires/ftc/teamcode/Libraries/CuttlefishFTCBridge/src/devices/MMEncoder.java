package org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices;
import com.roboctopi.cuttlefish.components.RotaryEncoder;
import com.roboctopi.cuttlefish.utils.Direction;

/**
 * Rotary encoder connected through a motor encoder port
 * */
public class MMEncoder implements RotaryEncoder
{
    public MMRevHub hub;
    private final double encTicks;
    private int direction = 1;
    public int mPort;

    double home = 0.0;

    /**
     * @param revHub
     * @param port Motor port of the encoder
     * @param countsPerRevolution Number of counts per revolution of the encoder
     * */
    public MMEncoder(MMRevHub revHub, int port, double countsPerRevolution) {
        hub = revHub;
        encTicks = countsPerRevolution;
        mPort = port;
        home += getRotation()*direction;
    }

    /**
     * Get the rotation of the encoder in radians
     * */
    public double getRotation()
    {
        return (2*Math.PI*getCounts()/encTicks - home)*direction;
    }
    public double getVelocity()
    {
        return 2*Math.PI*hub.bulkData.getEncoderVelocity(mPort)/encTicks*direction;
    }

    /**
     * Get the number of counts that the encoder has turned
     * */
    public int getCounts()
    {
        return hub.bulkData.getEncoderPosition(mPort);
    }

    /**
     * idk what they did above so this a new one
     * @return rpm
     */
    public double getRPM() {
        return 60 * (hub.bulkData.getEncoderVelocity(mPort)/encTicks) * direction;
    }

    /**
     * Set the direction of the encoder.
     * @param direction
     * */
    public void setDirection(Direction direction)
    {
        if(direction == Direction.REVERSE)
        {
            this.direction = -1;
        }
        else
        {
            this.direction = 1;
        }
    }

}

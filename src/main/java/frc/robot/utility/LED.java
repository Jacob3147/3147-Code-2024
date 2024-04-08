package frc.robot.utility;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LEDConstants.*;

import java.util.function.BooleanSupplier;

public class LED extends SubsystemBase
{
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;
    int m_rainbowFirstPixelHue = 0;
    int flashBuffer = 0;

    public boolean flashing_note = false;
    public boolean holding_note = false;
    
    public LED(BooleanSupplier haveNote)
    {
        m_led = new AddressableLED(led_pwm_port);
        m_ledBuffer = new AddressableLEDBuffer(led_length);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();

        holding_note = haveNote.getAsBoolean();
        
    }

    @Override
    public void periodic() 
    {
        if (DriverStation.isDisabled())
        {
            rainbow();
        }
        else if (flashing_note)
        {
            flashOrange();
        }
        else if (holding_note)
        {
            LED_Orange();
        }
        else 
        {
            LED_alliance();
        }
        
    }

    public void LED_alliance()
    {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Blue)
            {
                for(var i = 0; i < m_ledBuffer.getLength(); i++)
                {
                    m_ledBuffer.setRGB(i, color_blue[0], color_blue[2], color_blue[1]);
                }
            }
            else
            {
                for(var i = 0; i < m_ledBuffer.getLength(); i++)
                {
                    m_ledBuffer.setRGB(i, color_red[0], color_red[2], color_red[1]);
                }
            }
        }
        else
        {
            for(var i = 0; i < m_ledBuffer.getLength(); i++)
            {
                m_ledBuffer.setRGB(i, color_off[0], color_off[2], color_off[1]);
            }
        }
        m_led.setData(m_ledBuffer);
    }

    public void rainbow() 
    {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            int hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 1;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
    }

    public void flashOrange() 
    {
        
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            
            if(flashBuffer < 5)
            {
                m_ledBuffer.setRGB(i, color_orange[0], color_orange[2], color_orange[1]);
            }
            else
            {
                m_ledBuffer.setRGB(i, color_off[0], color_off[2], color_off[1]);
            }
        }
        flashBuffer++;
        flashBuffer %= 10;
        m_led.setData(m_ledBuffer);
    }

    public void LED_Orange()
    {
        for(var i = 0; i < m_ledBuffer.getLength(); i++)
        {
            m_ledBuffer.setRGB(i, color_orange[0], color_orange[2], color_orange[1]);
        }
        m_led.setData(m_ledBuffer);
    }

    public void LED_Red()
    {
        for(var i = 0; i < m_ledBuffer.getLength(); i++)
        {
            m_ledBuffer.setRGB(i, color_red[0], color_red[2], color_red[1]);
        }
        m_led.setData(m_ledBuffer);
    }
}
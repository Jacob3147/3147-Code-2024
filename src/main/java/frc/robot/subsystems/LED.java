package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase
{
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;
    int m_rainbowFirstPixelHue = 0;
    int val = 0;

    public LED()
    {
        m_led = new AddressableLED(LEDConstants.led_pwm_port);
        m_ledBuffer = new AddressableLEDBuffer(LEDConstants.led_length);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
    @Override
    public void periodic() 
    {
        flashOrange();
    }

    private void rainbow() {
        
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
          // Set the value
          m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 1;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
      }

      public void flashOrange() {
        
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          
          // Set the value
          m_ledBuffer.setHSV(i, 174, 255, val%255);
        }
        // Increase by to make the rainbow "move"
        val +=20;
        m_led.setData(m_ledBuffer);
      }

    public void LED_Orange()
    {
        for(var i = 0; i < m_ledBuffer.getLength(); i++)
        {
            m_ledBuffer.setRGB(i, LEDConstants.color_orange[0], LEDConstants.color_orange[2], LEDConstants.color_orange[1]);
        }
        m_led.setData(m_ledBuffer);
    }
    public void LED_Red()
    {
        for(var i = 0; i < m_ledBuffer.getLength(); i++)
        {
            m_ledBuffer.setRGB(i, LEDConstants.color_red[0], LEDConstants.color_red[2], LEDConstants.color_red[1]);
        }
        m_led.setData(m_ledBuffer);
    }
}

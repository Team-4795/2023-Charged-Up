package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateManager;

public class LEDs extends SubsystemBase {
    private final int LED_LENGTH = 27;
    
    public final class RGB {
        public int r, g, b;
        public RGB (int r, int g, int b) {
            this.r = r;
            this.g = g; 
            this.b = b;
        }
    }
    
    public final class HSV {
        public int h, s, v;
        public HSV (int h, int s, int v) {
            this.h = h;
            this.s = s; 
            this.v = v;
        }
    }

    //set the led port(pwm)
    private AddressableLED led;
    private AddressableLEDBuffer buffer;
    

    public LEDs() {
        led = new AddressableLED(2);
        buffer = new AddressableLEDBuffer(LED_LENGTH);
        
        led.setLength(buffer.getLength());
        this.init();
    }

    private void init() {
        this.setRGB(255, 255, 255);
    }

    public int getLength() {
        return buffer.getLength();
    }

    private void setColor(int a0, int a1, int a2, boolean color_model, double portion) /* false: RGB; true: HSV */ {
        portion = MathUtil.clamp(portion, 0, 1);

        for (int i = 0; i < buffer.getLength(); i++) {
            if (i < Math.round(buffer.getLength() * portion)) {
                if (color_model) buffer.setHSV(i, a0, a1, a2);
                else buffer.setRGB(i, a0, a1, a2);
            } else buffer.setRGB(i, 0, 0, 0);
        }

        led.setData(buffer);
        led.start();
        
    }
    private void setBottomColor(int a0, int a1, int a2, boolean color_model) /* false: RGB; true: HSV */ {

        for (int i = 0; i < (buffer.getLength()/2); i++) {
            if (i < Math.round(buffer.getLength())) {
                if (color_model) buffer.setHSV(i, a0, a1, a2);
                else buffer.setRGB(i, a0, a1, a2);
            } else buffer.setRGB(i, 0, 0, 0);
        }

        led.setData(buffer);
        led.start();
        
    }
    private void setTopColor(int a0, int a1, int a2, boolean color_model) /* false: RGB; true: HSV */ {

        for (int i = (buffer.getLength()/2); i < buffer.getLength(); i++) {
            if (i < Math.round(buffer.getLength())) {
                if (color_model) buffer.setHSV(i, a0, a1, a2);
                else buffer.setRGB(i, a0, a1, a2);
            } else buffer.setRGB(i, 0, 0, 0);
        }

        led.setData(buffer);
        led.start();
        
    }
    
    public void setRGB(int r, int g, int b, double portion) {
        setColor(r, g, b, false, portion);
    }
    
    public void setRGB(int r, int g, int b) {
    
        setColor(r, g, b, false, 1);
        
    }
    
    public void setRGB(RGB rgb, double portion) {
        setColor(rgb.r, rgb.g, rgb.b, false, portion);
    }
    
    public void setRGB(RGB rgb) {
        setColor(rgb.r, rgb.g, rgb.b, false, 1);
    }
    
    public void setHSV(int h, int s, int v, double portion) {
        setColor(h, s, v, true, portion);
    }
        
    public void setHSV(int h, int s, int v) {
        setColor(h, s, v, true, 1);
    }

    public void setHSVIndex(int index, int h, int s, int v) {
        buffer.setHSV(index, h, s, v);
    }

    public void setOutput() {
        led.setData(buffer);
        led.start();
    }
    
    public void setHSV(HSV hsv, double portion) {
        setColor(hsv.h, hsv.s, hsv.v, true, portion);
    }
    
    public void setHSV(HSV hsv) {
        setColor(hsv.h, hsv.s, hsv.v, true, 1);
    }

    public void setBottomRGB(int r, int g, int b) {
        setBottomColor(r, g, b, false);
    }

    public void setTopRGB(int r, int g, int b) {
        setTopColor(r, g, b, false);
    }

    public void reset() {
        setColor(0, 0, 0, false, 1);
    }

    public void setLEDState(StateManager.LED state) {
        switch (state) {
            case Cube: setRGB(127, 0, 255); break;
            case Cone: setRGB(255, 255, 0); break;
            default: break;
        }
    }
    
}
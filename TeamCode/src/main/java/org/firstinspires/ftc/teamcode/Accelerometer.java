package org.firstinspires.ftc.teamcode;


import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Accelerometer implements SensorEventListener {

    public enum PhoneRotation {
        UP, LEFT, DOWN, RIGHT;

        @Override
        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase(); // Return the name with the first letter capitalized and the rest lowercase (I.e. Down)
        }
    }

    private Sensor accelerometer;
    private SensorManager sensorManager;

    private float x, y, z;

    public Accelerometer(HardwareMap hardwareMap) {
        setup(hardwareMap);
    }

    private void setup(HardwareMap hardwareMap) {
        // Create our SensorManager
        sensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);

        // Get the accelerometer
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);

        // Register sensor listener
        sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_GAME);
    }

    public void stop() {
        sensorManager.unregisterListener(this, accelerometer);
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    public float getZ() {
        return z;
    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        x = sensorEvent.values[0];
        y = sensorEvent.values[1];
        z = sensorEvent.values[2];
    }

    public PhoneRotation getPhoneRotation() {
        float[] readings = {getX(), getY()};
        int maxReadingIndex = largerMagnitude(readings[0], readings[1]) ? 0 : 1;

        float maxReading = readings[maxReadingIndex];

        if (maxReading >= 0) {
            // Either up or left
            return (maxReadingIndex == 0) ? PhoneRotation.LEFT : PhoneRotation.UP;
        } else {
            // Either down or right
            return (maxReadingIndex == 0) ? PhoneRotation.RIGHT : PhoneRotation.DOWN;
        }
    }


    private boolean largerMagnitude(float bigger, float smaller) {
        return Math.abs(bigger) >= Math.abs(smaller);
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {
        // Not in use, just here to remove implementation error
    }
}

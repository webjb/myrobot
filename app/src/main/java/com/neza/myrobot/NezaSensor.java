package com.neza.myrobot;

/**
 * Created by bobsang on 3/23/2016.
 */
import android.app.Activity;
import android.content.Context;
import android.util.Log;
import android.widget.Toast;

import android.hardware.SensorManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;

public class NezaSensor {
    private SensorManager mSensorManager;
    private Sensor mPressure;
    private Sensor mAccelerometer;
    private Sensor mProximity;

    public void init(Activity activity) {
        mSensorManager = (SensorManager) activity.getSystemService(Context.SENSOR_SERVICE);
        //List<Sensor> deviceSensors = mSensorManager.getSensorList(Sensor.TYPE_ALL);
        mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        if (mAccelerometer != null) {
            mSensorManager.registerListener(mySensorEventListener, mAccelerometer,
                    SensorManager.SENSOR_DELAY_NORMAL);
            Log.i("Compass MainActivity", "Registerered for ORIENTATION Sensor");
        } else {
            Log.e("Compass MainActivity", "Registerered for ORIENTATION Sensor");
//            Toast.makeText(this, "ORIENTATION Sensor not found",Toast.LENGTH_LONG).show();
        }

        if (mAccelerometer == null) {

        }
        //
        mProximity = mSensorManager.getDefaultSensor(Sensor.TYPE_PROXIMITY);
        if (mProximity != null) {
            mSensorManager.registerListener(mySensorProximityEventListener, mProximity,
                    SensorManager.SENSOR_DELAY_NORMAL);
            Log.i("Compass MainActivity", "Registerered for ORIENTATION Sensor");
        } else {
            Log.e("Compass MainActivity", "Registerered for ORIENTATION Sensor");
//            Toast.makeText(this, "ORIENTATION Sensor not found", Toast.LENGTH_LONG).show();
        }

        if (mProximity == null) {

        }
    }

    private SensorEventListener mySensorProximityEventListener = new SensorEventListener() {

        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {
        }

        @Override
        public void onSensorChanged(SensorEvent event) {
            // angle between the magnetic north direction
            // 0=North, 90=East, 180=South, 270=West
            float azimuth = event.values[0];
            azimuth += 1;
//		      compassView.updateData(azimuth);
        }
    };

    private SensorEventListener mySensorEventListener = new SensorEventListener() {

        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {
        }

        @Override
        public void onSensorChanged(SensorEvent event) {
            // angle between the magnetic north direction
            // 0=North, 90=East, 180=South, 270=West
            float azimuth = event.values[0];
            azimuth += 1;
//		        compassView.updateData(azimuth);
        }
    };
}


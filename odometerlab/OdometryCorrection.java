/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometerlab;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;



public class OdometryCorrection extends Thread {
  private static final long CORRECTION_PERIOD = 7;
  private Odometer odometer;

  // Instantiates the light sensor
  private static Port portColor = LocalEV3.get().getPort("S1");
  private static SensorModes myColor = new EV3ColorSensor(portColor);
  private static SampleProvider myColorSample = myColor.getMode("Red");
  static float[] sampleColor = new float[myColor.sampleSize()];

  // constructor
  public OdometryCorrection(Odometer odometer) {
    this.odometer = odometer;
  }

  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd, sleepTime;
    int numSamplesY = 0; // number of black lines viewed in Y direction.
    int numSamplesX = 0;

    while (true) {
      correctionStart = System.currentTimeMillis();
      // TODO Place correction implementation here
      myColorSample.fetchSample(sampleColor, 0); // Fetches reading from sensor

      if ((sampleColor[0] * 1000 < 300) && (sampleColor[0] * 1000 > 5)) { // detection of line from
                                                                          // 5 to 300
        Sound.beep();

        if (odometer.getTheta() >= 0 && odometer.getTheta() < 85 || odometer.getTheta() >= 358) {
          odometer.setY(numSamplesY * 30.48); // update expected Y distance at black lines
          numSamplesY++;

        } else if (odometer.getTheta() >= 85 && odometer.getTheta() < 175) { // Second orientation
          odometer.setX(numSamplesX * 30.48); // update expected X distance at black lines
          numSamplesX++;
        } else if (odometer.getTheta() >= 175 && odometer.getTheta() < 265) {
          odometer.setY((numSamplesY - 1) * 30.48);
          numSamplesY--; // In opposite direction our Y values should decrease
        } else if (odometer.getTheta() >= 265 && odometer.getTheta() < 358) {
          odometer.setX((numSamplesX - 1) * 30.48);
          numSamplesX--;
        }
      }


      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          sleepTime = (CORRECTION_PERIOD - (correctionEnd - correctionStart));
          Thread.sleep(sleepTime);
        } catch (InterruptedException e) {
          // there is nothing to be done here because it is not
          // expected that the odometry correction will be
          // interrupted by another thread
        }
      }
    }
  }
}


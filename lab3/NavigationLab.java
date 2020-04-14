// Lab2.java

package ca.mcgill.ecse211.lab3;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class NavigationLab {

  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  private static final EV3MediumRegulatedMotor sensorMotor =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));

  private static final Port usPort = LocalEV3.get().getPort("S4");

  public static final double WHEEL_RADIUS = 2.1;
  public static final double TRACK = 15.4;

  public static void main(String[] args) {
    int buttonChoice;

    final TextLCD t = LocalEV3.get().getTextLCD();
    Odometer odometer = new Odometer(leftMotor, rightMotor);
    OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);


    @SuppressWarnings("resource") // Because we don't bother to close this resource
    SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
    SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from

    float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
    Navigation navigation = new Navigation(odometer, leftMotor, rightMotor, sensorMotor, usDistance,
        usData, WHEEL_RADIUS, TRACK); // returned

    do {

      // clear the display
      t.clear();

      // ask the user whether the motors should drive in a square or float
      t.drawString("< Left | Right >", 0, 0);
      t.drawString("       |        ", 0, 1);
      t.drawString(" Float | Drive  ", 0, 2);
      t.drawString("motors | in a   ", 0, 3);
      t.drawString("       | Map ", 0, 4);

      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    if (buttonChoice == Button.ID_LEFT) {

      leftMotor.forward();
      leftMotor.flt();
      rightMotor.forward();
      rightMotor.flt();

      odometer.start();
      odometryDisplay.start();

    } else {
      // clear the display
      t.clear();

      // ask the user whether the motors should drive in a square or float
      t.drawString("< Left | Right >", 0, 0);
      t.drawString("  No   | for   ", 0, 1);
      t.drawString(" corr- | Nav.  ", 0, 2);
      t.drawString(" ection| Avoid ", 0, 3);
      t.drawString("       |        ", 0, 4);

      buttonChoice = Button.waitForAnyPress();

      odometer.start();
      odometryDisplay.start();

      if (buttonChoice == Button.ID_RIGHT) {
        navigation.start();
      }


    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}

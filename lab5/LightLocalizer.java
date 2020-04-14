/*
*/
package ca.mcgill.ecse211.lab5;

import java.util.LinkedList;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;


/**
 * This class perform the light localization and places the robot at the point (0,0).
 */
public class LightLocalizer extends Thread {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private double leftRadius;
  private double rightRadius;
  private double width;
  private static final int ROTATE_SPEED = 70;
  private static final int FORWARD_SPEED = 115;
  private static final int normAccl = 1500; // Acceleration values
  private static final int slowAccl = 500;
  private final double sensorOffset = 14.0;
  public final float TWO_STDV = (float) 0.3247;
  public final float LOWPASS_FILTER = (float) 0.20;
  private static final long ODOMETER_PERIOD = 25; /* odometer update period, in ms */
  private static final double cartesian = 90;
  private EV3ColorSensor colorSensorLeft;
  private EV3ColorSensor colorSensorRight;

  private Navigation navigation;
  int size = 10;
  private EV3LargeRegulatedMotor usMotor;
  private double theta;
  private double previous = 0;
  private double average;
  private double finalX;
  private double finalY;

  // in tile
  // private double XCoord;
  // private double YCoord;
  private int sc;
  private final double tile = 30.48;

  private boolean firstLineCrossed = false;
  private boolean secondLineCrossed = false;



  // Creator
  public LightLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius, double width,
      int sc) {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.leftRadius = leftRadius;
    this.rightRadius = rightRadius;
    this.width = width;
    this.sc = sc;


    colorSensorLeft = new EV3ColorSensor(SensorPort.S3);
    colorSensorRight = new EV3ColorSensor(SensorPort.S4);


  }

  public void run() {
    long correctionStart, correctionEnd;
    // Sound.beep();


    // Go close to the 0,0 point
    // navigation.travelTo(0.2, 0.2);
    // turnTo(-360);
    int lineIndexLeft = 0;
    int lineIndexRight = 0;

    // double thetaX1 = 0;
    // double thetaX2 = 0;
    double thetaY1 = 0;
    double thetaY2 = 0;
    boolean xSetLeft = false;
    boolean ySetLeft = false;
    boolean xSetRight = false;
    boolean ySetRight = false;
    boolean rightFirst = false;
    boolean leftFirst = false;
    boolean leftOn = false;
    boolean rightOn = false;

    SensorLogger logger = new SensorLogger();

    // TODO: while loop to fill the buffer of the filter


    float intensityLeft[] = new float[1]; // used to be size 3
    float intensityRight[] = new float[1];

    boolean condition = true;
    normMotors(); // Sets the motors speed and acceleration
    leftMotor.synchronizeWith(new RegulatedMotor[] {rightMotor});
    leftMotor.startSynchronization();
    leftMotor.forward();
    rightMotor.forward();
    leftMotor.endSynchronization();
    while (condition) {
      long updateStart = System.currentTimeMillis();
      System.out.println("Knock");

      colorSensorLeft.getRedMode().fetchSample(intensityLeft, 0);
      colorSensorRight.getRedMode().fetchSample(intensityRight, 0);

      logger.writeToFile(Float.toString(intensityLeft[0]));



      // First line is crossed
      if (firstLineCrossed == true) {
        drive(2.0); // 2.0 cm is distance between light sensor and wheel axis
        if (secondLineCrossed == true) {

          turnTo(-cartesian, false);


          // quit while loop
          condition = false;
          firstLineCrossed = false;// Reset Paramaters
          secondLineCrossed = false;

        } else {// First time the first line is crossed
          firstLineCrossed = false;

          turnTo(cartesian, false);


          thetaY1 = 0;
          thetaY2 = 0;
          // drive(30);

          secondLineCrossed = true;// SO that next line crossed will trigger second line code
        }
        normMotors();
        continue;
      }



      // If left sensor detects a line
      if (intensityLeft[0] < TWO_STDV && intensityLeft[0] > LOWPASS_FILTER) {


        if (!rightFirst && !leftFirst) { // If neither sensor hit yet.

          // Synchronized stop motors
          leftMotor.startSynchronization();
          leftMotor.stop(true);
          rightMotor.stop(false);
          leftMotor.endSynchronization();

          leftFirst = true;
          thetaY1 = odometer.getTheta();

          // if right is also on line
          if (intensityRight[0] < TWO_STDV && intensityRight[0] > LOWPASS_FILTER) {

          } else { // Turn the right motor forward until it also hits the line
            slowMotors();
            rightMotor.forward();
            while (!(intensityRight[0] < TWO_STDV && intensityRight[0] > LOWPASS_FILTER)) {
              // Just keep turning while right motor looks for line
              colorSensorRight.getRedMode().fetchSample(intensityRight, 0);
              try { // Sleep to try to detect line center of the line, not the edge
                Thread.sleep(150);
              } catch (InterruptedException e) {
              }
            }
            rightMotor.stop(false);// when right is also on the line
          }
          firstLineCrossed = true; // As both are on the same line
          leftFirst = false;
          continue;

        }
        /*
         * else if (rightFirst && !leftFirst) { //Right has already hit //Check to see if both on
         * line leftMotor.stop(); leftMotor.setSpeed(FORWARD_SPEED); leftMotor.rotate(0);
         * rightMotor.rotate(0); thetaY2 = odometer.getTheta();
         * 
         * }
         */


      }
      // If right line detected
      if (intensityRight[0] < TWO_STDV && intensityRight[0] > LOWPASS_FILTER) {
        // rightMotor.setAcceleration(0);
        // Sound.beep();
        // Sound.playTone(880, 30);


        if (!rightFirst && !leftFirst) { // If neither sensor hit yet.

          // Synchronized stop motors
          leftMotor.startSynchronization();
          rightMotor.stop(true);
          leftMotor.stop(false);
          leftMotor.endSynchronization();

          rightFirst = true;
          thetaY1 = odometer.getTheta();
          // if left is also on line
          if (intensityLeft[0] < TWO_STDV && intensityLeft[0] > LOWPASS_FILTER) {
            Sound.twoBeeps();
          } else { // Turn the left motor forward until it also hits the line
            slowMotors();
            leftMotor.forward();

            while (!(intensityLeft[0] < TWO_STDV && intensityLeft[0] > LOWPASS_FILTER)) {
              // Just keep turning while left motor looks for line
              colorSensorLeft.getRedMode().fetchSample(intensityLeft, 0);
              // Poor mans timer
              try {
                Thread.sleep(150);
              } catch (InterruptedException e) {
              }
            }
            leftMotor.stop(false);// when right is also on the line
          }
          firstLineCrossed = true; // As both are on the same line
          rightFirst = false;
          continue;

        }


      }



      long updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here because it is not
          // expected that the odometer will be interrupted by
          // another thread
        }
      }


    }

    leftMotor.startSynchronization();
    leftMotor.stop(true);
    rightMotor.stop(false);
    leftMotor.endSynchronization();
    try {
      Thread.sleep(500);
    } catch (InterruptedException e) {

    }


    switch (sc) {
      case 0:
        odometer.setX(tile);
        odometer.setY(tile);
        odometer.setTheta(0.0);
        break;
      case 1:
        odometer.setX(7 * tile);
        odometer.setY(tile);
        odometer.setTheta(270.0);
        break;
      case 2:
        odometer.setX(7 * tile);
        odometer.setY(tile);
        odometer.setTheta(180.0);
        break;
      case 3:
        odometer.setX(tile);
        odometer.setY(7 * tile);
        odometer.setTheta(90.0);
        break;

    }
    System.out.printf("X: %f \n", odometer.getX());
    System.out.printf("Y: %f \n", odometer.getY());
    System.out.printf("Theta: %f \n", odometer.getTheta());
    return;

  }

  /**
   * This method determines at which angle the robot should go to. It should do so with a MINIMAL
   * angle.
   * 
   * @param theta if positive then the robot will turn to the right, else it will rotate the the
   *        left
   * @param fallThrough
   */
  public void turnTo(double theta, boolean fallThrough) {

    System.out.println("Turning");
    slowMotors();

    if (theta < 0) { // if angle is negative, turn to the left
      leftMotor.rotate(-convertAngle(leftRadius, width, -theta), true);
      rightMotor.rotate(convertAngle(rightRadius, width, -theta), fallThrough);
    } else { // angle is positive, turn to the right
      leftMotor.rotate(convertAngle(leftRadius, width, theta), true);
      rightMotor.rotate(-convertAngle(rightRadius, width, theta), fallThrough);
    }
    normMotors();

    leftMotor.startSynchronization();
    leftMotor.forward();
    rightMotor.forward();
    leftMotor.endSynchronization();
  }

  /**
   * This method slows down the motors
   */
  public void slowMotors() {
    leftMotor.setAcceleration(slowAccl);
    rightMotor.setAcceleration(slowAccl);

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
  }

  /**
   * This method speeds up the motors to regular speed
   */
  public void normMotors() {
    leftMotor.setAcceleration(normAccl);
    rightMotor.setAcceleration(normAccl);

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
  }

  /**
   * This method synchronizes stopping the motor
   */
  public void syncStop() {
    leftMotor.startSynchronization();
    rightMotor.stop(true);
    leftMotor.stop(false);
    leftMotor.endSynchronization();
  }

  /**
   * This method drives the robot a given distance.
   * 
   * @param distance the robot will travel
   */

  public void drive(double distance) {

    normMotors();
    leftMotor.rotate(convertDistance(leftRadius, distance), true);
    rightMotor.rotate(convertDistance(rightRadius, distance), false);
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
    }
    // Sound.buzz();
  }

  /**
   * This method converts the angle into the number of rotations it must perform
   * 
   * @param radius of the wheel
   * @param width of the cart
   * @param angle to turn to
   * @return number of rotations
   */
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  /**
   * This method converts the distance into the number of rotations
   * 
   * @param radius of the wheel
   * @param distance to travel
   * @return number of rotations
   */
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }
}

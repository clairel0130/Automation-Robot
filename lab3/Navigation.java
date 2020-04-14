/*
 * Navigation.java
 */
package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.robotics.SampleProvider;

public class Navigation extends Thread {
  private static final int FORWARD_SPEED = 200;
  private static final int ROTATE_SPEED = 100;
  private double[] map = new double[10];
  private int pointMap;
  private double Theta, X, Y; // Temp variables, passed from Odometer
  private Odometer odometer;
  private EV3LargeRegulatedMotor leftMotor, rightMotor;
  private EV3MediumRegulatedMotor sensorMotor;
  private double wheelRadius, width;
  private SampleProvider us;
  private float[] usData;
  private boolean wallFollow;
  private final int bandCenter = 20;
  private final int bandWidth = 5; // Get from bangbang
  private final int minBand = bandCenter - bandWidth;
  private final int maxBand = bandCenter + bandWidth;

  private int countOut; // counter for filtering outlier values
  private int countCheck; // Counter to regulate countOut values;
  private int countRev; // Counter to regulate reverse control


  public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, EV3MediumRegulatedMotor sensorMotor, SampleProvider us,
      float[] usData, double wheelRad, double width) {
    this.odometer = odometer;
    this.pointMap = 0;
    // Map 1
//     this.map[0] = 0.0;
//     this.map[1] = 2.0;
//     this.map[2] = 1.0;
//     this.map[3] = 1.0;
//     this.map[4] = 2.0;
//     this.map[5] = 2.0;
//     this.map[6] = 2.0;
//     this.map[7] = 1.0;
//     this.map[8] = 1.0;
//     this.map[9] = 0.0;



    // Map 2
    // this.map[0] = 1.0;
    // this.map[1] = 1.0;
    // this.map[2] = 0.0;
    // this.map[3] = 2.0;
    // this.map[4] = 2.0;
    // this.map[5] = 2.0;
    // this.map[6] = 2.0;
    // this.map[7] = 1.0;
    // this.map[8] = 1.0;
    // this.map[9] = 0.0;

    // Map 3
    this.map[0] = 1.0;
    this.map[1] = 0.0;
    this.map[2] = 2.0;
    this.map[3] = 1.0;
    this.map[4] = 2.0;
    this.map[5] = 2.0;
    this.map[6] = 0.0;
    this.map[7] = 2.0;
    this.map[8] = 1.0;
    this.map[9] = 1.0;

    // Map 4
//     this.map[0] = 0.0;
//     this.map[1] = 1.0;
//     this.map[2] = 1.0;
//     this.map[3] = 2.0;
//     this.map[4] = 1.0;
//     this.map[5] = 0.0;
//     this.map[6] = 2.0;
//     this.map[7] = 1.0;
//     this.map[8] = 2.0;
//     this.map[9] = 2.0;

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.sensorMotor = sensorMotor;
    this.wheelRadius = wheelRad;
    this.width = width;
    this.us = us;
    this.usData = usData;
    this.wallFollow = true;

    this.countOut = 0;
    this.countCheck = 0;
    this.countRev = 0;
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {this.leftMotor,
        this.rightMotor}) {
      motor.stop();
      motor.setAcceleration(3000);
    }
    sensorMotor.stop();
    sensorMotor.setAcceleration(1000);
  }

  public void run() {
    int dist;
    double enteringAngle = 0.0;
    double endingAngle = 0.0;
    boolean firstDetection = true;

    while (this.pointMap < this.map.length) {
      this.Theta = this.odometer.getTheta();
      this.X = this.odometer.getX();
      this.Y = this.odometer.getY();
      us.fetchSample(usData, 0); // acquire US data
      dist = (int) (usData[0] * 100.0); // extract from buffer, cast to int

      if (dist < 20) { // Too close to obstacle correct

        if (firstDetection == true) {// For the first time
          firstDetection = false;
          enteringAngle = this.Theta;
          endingAngle = enteringAngle - 55; // stop condition of the avoidance mode
          if (endingAngle < 0) {
            endingAngle += 360;
          }
          sensorMotor.rotate(-45, true);
          turnTo((this.Theta + 45) % 360); // Turn 45 degrees right
        }

        // Meet an obstacle, start the wall following mode
        while (this.wallFollow) { // While flag is true wallfollow

          this.Theta = this.odometer.getTheta();
          this.X = this.odometer.getX();
          this.Y = this.odometer.getY();
          us.fetchSample(usData, 0); // acquire US data
          dist = (int) (usData[0] * 100.0);// extract from buffer, cast to int
          dist = (dist * 7071) / 10000; // Sensor is now at 45 degree angle thus *sqrt(2)/2
          processUSData(dist, endingAngle); // now take action depending on distance
        }
        firstDetection = true; // Since the while loop only ends when the robot is far enough away
        this.wallFollow = true; // So that next block in the way will be avoided.
      } else { // No object detected, so travel to the next point in map.
        travelTo(this.map[this.pointMap] * 30.48, this.map[this.pointMap + 1] * 30.48);
      }

    }
    leftMotor.stop();
    rightMotor.stop();
  }

  // the method below compute the angle from current position to estimated point
  private double expectedTheta(double nextX, double nextY) {
    double expTheta;
    expTheta = Math.atan2((nextX - this.X), (nextY - this.Y));
    expTheta *= 180 / Math.PI; // Converts expTheta to degrees

    if (expTheta < 0) {
      expTheta += 360;
    }
    return expTheta;
  }

  private void travelTo(double nextX, double nextY) {
    double expTheta;
    double distance;


    expTheta = expectedTheta(nextX, nextY); // calculate the theta needed to turn
    distance = Math.sqrt(Math.pow((nextX - this.X), 2.0) + Math.pow((nextY - this.Y), 2.0)); // distance

    if (distance <= 2.5) { // If Max is within circle of 2.5 cm from destination consider arrived
      this.pointMap += 2; // update next points in map
      return;
    }
    double delta;
    delta = this.Theta - expTheta;
    if (delta < 0) {
      delta += 360;
    }
    if (delta > 180) {
      delta = 360 - delta;
    }
    if (delta < 5) {
      // if the direction is good, drive forward
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
      leftMotor.forward();
      rightMotor.forward();
    } else {
      // The direction is not good or good enough
      turnTo(expTheta);
    }
  }

  private void turnTo(double xpTheta) {
    double deltaTheta = xpTheta - this.Theta;
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    // angular correction, aims at doing the minimal angle
    if (deltaTheta <= 180.0 && deltaTheta >= -180.0) {
      leftMotor.rotate(convertAngle(wheelRadius, width, deltaTheta), true);
      rightMotor.rotate(-convertAngle(wheelRadius, width, deltaTheta), false);


    } else if (deltaTheta < -180.0) {
      deltaTheta += 360.0; // Updates the correction to minimum angle
      leftMotor.rotate(convertAngle(wheelRadius, width, deltaTheta), true);
      rightMotor.rotate(-convertAngle(wheelRadius, width, deltaTheta), false);
    } else {
      deltaTheta -= 360;
      leftMotor.rotate(convertAngle(wheelRadius, width, deltaTheta), true);
      rightMotor.rotate(-convertAngle(wheelRadius, width, deltaTheta), false);
    }
  }

  private void processUSData(int dist, double endingAngle) {
    // Gets called when you are too close to a wall directly infront of you
    // Follow the wall

    if (dist >= minBand && dist <= maxBand) { // distance is in good band
      leftMotor.setSpeed(FORWARD_SPEED); // Start robot moving forward
      rightMotor.setSpeed(FORWARD_SPEED);
      leftMotor.forward();
      rightMotor.forward();

      this.countCheck++;
      if (this.countCheck > 3) { // if in good band for 3 cycles
        this.countOut = 0;
        this.countCheck = 0;
        this.countRev = 0;
      }

    } else if (dist <= 15) { // Too close to inner wall
      this.countRev++;
      if (this.countRev > 5) { // Turn in place if too close for 5 cycles
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);
        leftMotor.forward();
        rightMotor.backward(); // turns right
      }

    } else if (dist < minBand) { // Turn right slightly, out of bandWidth.
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(ROTATE_SPEED); // Slow outer motor
      leftMotor.forward();
      rightMotor.forward();
    } else if (dist > maxBand && dist <= maxBand + 75) {// Turn Left slightly
      // if the distance of the US is less than maxBand+75, it's a bit away from the wall
      // 75 is an empirical value
      leftMotor.setSpeed(ROTATE_SPEED + 50); // Inner motor slow down
      rightMotor.setSpeed(FORWARD_SPEED);
      leftMotor.forward();
      rightMotor.forward();

    } else if (dist > maxBand + 75) { // either outlier or hard left
      this.countOut++;
      if (this.countOut > 25) {// if not outlier turn hard.
        // 25 is the cycles counted for filtering the error. if within 25 cycles, error.
        leftMotor.setSpeed(ROTATE_SPEED); // Inner motor slow down
        rightMotor.setSpeed(FORWARD_SPEED);
        leftMotor.forward();
        rightMotor.forward();

      } else { // Outlier thus go straight.
        leftMotor.setSpeed(FORWARD_SPEED); // Start robot moving forward
        rightMotor.setSpeed(FORWARD_SPEED);
        leftMotor.forward();
        rightMotor.forward();
      }
    }
    // check if theta meets the endingAngle.
    double delt;
    delt = this.Theta - endingAngle;
    if (delt < 0) {
      delt += 360;
    }
    if (delt > 180) {
      delt = 360 - delt;
    }
    if (delt < 5) {
      this.wallFollow = false; // Exits while loop
      this.sensorMotor.rotateTo(0, false);
      return;
    }
  }

  private boolean isNavigating() {

    return false;
  }

  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
}

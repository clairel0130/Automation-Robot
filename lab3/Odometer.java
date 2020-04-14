package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
  // robot position
  private double x;
  private double y;
  private double theta;
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private static final long ODOMETER_PERIOD = 25; /* odometer update period, in ms */

  private Object lock; /* lock object for mutual exclusion */

  // default constructor
  public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.x = 0.0;
    this.y = 0.0;
    this.theta = 0.0;
    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;
    lock = new Object();

  }

  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;
    int nowTachoL, nowTachoR; // Current tachometer readings
    int lastTachoL = 0; // Previous tachometer readings
    int lastTachoR = 0;
    double distL, distR; // Distance traveled per wheel
    double deltaD, deltaT, dX, dY; // Change in distance, Theta, X, and Y

    while (true) {
      updateStart = System.currentTimeMillis();
      // TODO put (some of) your odometer code here
      nowTachoL = leftMotor.getTachoCount(); // Get current tachometer readings
      nowTachoR = rightMotor.getTachoCount();

      distL = Math.PI * NavigationLab.WHEEL_RADIUS * (nowTachoL - lastTachoL) / 180; // Displacement
      distR = Math.PI * NavigationLab.WHEEL_RADIUS * (nowTachoR - lastTachoR) / 180;

      lastTachoL = nowTachoL; // Update previous Tachometer readings.
      lastTachoR = nowTachoR;

      deltaD = 0.5 * (distL + distR); // Average displacement
      deltaT = (distL - distR) / NavigationLab.TRACK;// Change in Theta, 14.7 is wheelbase
      deltaT = deltaT * 180 / Math.PI;// DEGREE

      synchronized (lock) {
        theta += deltaT;
        theta = theta % 360; // Set upper-bound of theta to 360
        if (theta < 0) {
          theta += 360; // Sets lower-bound of theta to 0 and wraps around.
        }
        dX = deltaD * Math.sin(theta * Math.PI / 180); // Displacement in X direction
        dY = deltaD * Math.cos(theta * Math.PI / 180);
        x = x + dX; // Update x
        y = y + dY;

        /**
         * Don't use the variables x, y, or theta anywhere but here! Only update the values of x, y,
         * and theta in this block. Do not perform complex math
         * 
         */
        // theta = -0.7376; // TODO replace example value
      }

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
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
  }

  public void getPosition(double[] position, boolean[] update) {
    // ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0])
        position[0] = x;
      if (update[1])
        position[1] = y;
      if (update[2])
        position[2] = theta;
    }
  }

  public double getX() {
    double result;

    synchronized (lock) {
      result = x;
    }

    return result;
  }

  public double getY() {
    double result;

    synchronized (lock) {
      result = y;
    }

    return result;
  }

  public double getTheta() {
    double result;

    synchronized (lock) {
      result = theta;
    }

    return result;
  }

  // mutators
  public void setPosition(double[] position, boolean[] update) {
    // ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0])
        x = position[0];
      if (update[1])
        y = position[1];
      if (update[2])
        theta = position[2];
    }
  }

  public void setX(double x) {
    synchronized (lock) {
      this.x = x;
    }
  }

  public void setY(double y) {
    synchronized (lock) {
      this.y = y;
    }
  }

  public void setTheta(double theta) {
    synchronized (lock) {
      this.theta = theta;
    }
  }

  /**
   * @return the leftMotorTachoCount
   */
  public int getLeftMotorTachoCount() {
    return leftMotorTachoCount;
  }

  /**
   * @param leftMotorTachoCount the leftMotorTachoCount to set
   */
  public void setLeftMotorTachoCount(int leftMotorTachoCount) {
    synchronized (lock) {
      this.leftMotorTachoCount = leftMotorTachoCount;
    }
  }

  /**
   * @return the rightMotorTachoCount
   */
  public int getRightMotorTachoCount() {
    return rightMotorTachoCount;
  }

  /**
   * @param rightMotorTachoCount the rightMotorTachoCount to set
   */
  public void setRightMotorTachoCount(int rightMotorTachoCount) {
    synchronized (lock) {
      this.rightMotorTachoCount = rightMotorTachoCount;
    }
  }
}


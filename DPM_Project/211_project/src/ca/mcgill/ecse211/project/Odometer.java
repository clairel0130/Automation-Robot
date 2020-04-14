package ca.mcgill.ecse211.project;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
  // robot position
  public static int lastTachoL; // Tacho L at last sample
  public static int lastTachoR; // Tacho R at last sample
  public static int nowTachoL; // Current tacho L
  public static int nowTachoR; // Current tacho R
  private double x;
  private double y;
  private double theta;
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  public static final int SINTERVAL = 50; // Period of sampling f (mSec)
  public static final int SLEEPINT = 500; // Period of display update (mSec)
  public static final double WB = 15.3; // Wheelbase (cm)
  public static final double WR = 2.1; // Wheel radius (cm)

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

    while (true) {
      updateStart = System.currentTimeMillis();
      // TODO put (some of) your odometer code here

      double distL, distR, deltaD, deltaT, dX, dY;

      nowTachoL = leftMotor.getTachoCount(); // get tacho counts
      nowTachoR = rightMotor.getTachoCount();
      distL = 3.14159 * WR * (nowTachoL - lastTachoL) / 180; // compute L and R wheel displacements
      distR = 3.14159 * WR * (nowTachoR - lastTachoR) / 180;
      lastTachoL = nowTachoL; // save tacho counts for next iteration
      lastTachoR = nowTachoR;
      deltaD = 0.5 * (distL + distR); // compute vehicle displacement
      deltaT = (distL - distR) / WB; // compute change in heading

      synchronized (lock) {
        /**
         * Don't use the variables x, y, or theta anywhere but here! Only update the values of x, y,
         * and theta in this block. Do not perform complex math
         * 
         */
        // TODO replace example value
        theta += deltaT; // update heading
        dX = deltaD * Math.sin(theta); // compute X component of displacement
        dY = deltaD * Math.cos(theta); // compute Y component of displacement
        x = x + dX; // update estimates of X and Y position
        y = y + dY;
      }

      // Logging the odometer data
      // LightSensorLogger logger = new LightSensorLogger();
      //
      // String data = "x: "+ x + "; y:" + y;
      // logger.writeToFile(data);s
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
        if (Math.toDegrees(theta) >= 360) {
          position[2] = (Math.toDegrees(theta) - 360); // cap max theta at 359.9
        } else {
          position[2] = Math.toDegrees(theta);
        }

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

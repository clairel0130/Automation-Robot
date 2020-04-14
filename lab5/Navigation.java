package ca.mcgill.ecse211.lab5;


import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.RegulatedMotor;

// Shift Alt J Will generate java doc for method

public class Navigation { // extends Thread {
  // Constants
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private static final int FORWARD_SPEED = 250;
  private static final int ROTATE_SPEED = 70;
  private static final int normAccl = 1500;
  private static final int slowAccl = 500;
  private static final double tile = 30.48;
  private double leftRadius;
  private double rightRadius;
  private double width;
  private static final double error = 0.3;
  private Odometer odometer;
  private static boolean isNavigating = false;



  public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius, double width) {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    this.leftRadius = leftRadius;
    this.rightRadius = rightRadius;
    this.width = width;
    this.leftMotor.synchronizeWith(new RegulatedMotor[] {rightMotor});
  }

  public void run() {
    drive(lab5.X0, lab5.Y0);

    return;
  }

  public void drive(int x, int y) {

    syncStop();
    // wait 1 second


    travelTo(x, y);



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

  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  public void turnTo(double theta) {
    syncStop();
    slowMotors();

    System.out.printf("Turn To: %f \n", theta);

    isNavigating = true;

    leftMotor.rotate(convertAngle(leftRadius, width, theta), true);
    rightMotor.rotate(-convertAngle(rightRadius, width, theta), false);

    isNavigating = false;
    normMotors();
  }

  /**
   * @param distance
   */
  public void roll(double distance) {
    normMotors();

    // when the wheel is about to turn, set isNavigating to true
    isNavigating = true;

    leftMotor.rotate(convertDistance(leftRadius, (tile * distance)), true);
    rightMotor.rotate(convertDistance(rightRadius, (tile * distance)), false);

    // when the wheels stopped, set isNavigating back to false
    isNavigating = false;
  }

  public void travelTo(double x, double y) {

    turnTo(-odometer.getTheta()); // Turn to 0deg


    // get the current location
    double startX = odometer.getX() / tile;
    double startY = odometer.getY() / tile;
    System.out.printf("startX: %f , startY: %f", startX, startY);
    // calculate the difference between the source and destination x and y
    double distX = x - startX;
    double distY = y - startY;
    System.out.printf("distX: %f , distY: %f", distX, distY);


    if (lab5.SC == 1 || lab5.SC == 2) {
      turnTo(90);
    }
    roll(distX);
    // if the x coordinate is different
    if (Math.abs(distY) > error) {
      double angle = odometer.getTheta();
      // point the robot to 90 degrees
      turnTo(-angle); // turn by 90 degrees //(90 - angle);
      // move forward or backward to x value
      roll(distY);
    }

    // turn to the true 0 after reaching each point
    double turn = odometer.getTheta();
    turnTo(-turn);
    turnTo(90);
    syncStop();



  }

  public boolean isNavigating() {

    return isNavigating;
  }


}

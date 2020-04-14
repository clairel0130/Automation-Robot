package ca.mcgill.ecse211.project;

import java.util.LinkedList;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class will perform the ultrasonic localization using either - the rising edge routine or, -
 * the falling edge routine.
 */
public class UltrasonicLocalizer extends Thread {

  private Odometer odometer;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private double leftRadius;
  private double rightRadius;
  private double width;
  private static final int ROTATE_SPEED = 150;
  private static final int FORWARD_SPEED = 250;
  public EV3UltrasonicSensor usSensor;
  LinkedList<Integer> queue = new LinkedList<Integer>();
  int size = 10;
  private double average;
  double previous = 0;
  private double alpha = 0;
  private double THRESHOLD = 50;
  private double range = 2;
  private double beta = 0;
  private double deltaTheta;
  private boolean isFallingEdge;
  private boolean condition;
  private SensorLogger logger;
  // private static final Port usPort = LocalEV3.get().getPort("S1");

  // Creator
  public UltrasonicLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius, double width) {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.leftRadius = leftRadius;
    this.rightRadius = rightRadius;
    this.width = width;


    usSensor = new EV3UltrasonicSensor(SensorPort.S1);

  }


  public void run() {

    turnTo(-360.0);

    float intensity[] = new float[3];
    usSensor.getDistanceMode().fetchSample(intensity, 0);

    logger = new SensorLogger();
    condition = true;


    while (condition) {
      usSensor.getDistanceMode().fetchSample(intensity, 0);
      logger.writeToFile(Float.toString(intensity[0] * 100));



      fallingEdge(intensity[0] * 100);


      try {
        Thread.sleep(25);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
  }

  /**
   * This method performs the falling edge routine
   * 
   * @return true if the routine is finished
   */
  public boolean fallingEdge(float distance) {
    boolean done = false;

    if (distance < THRESHOLD && previous > 100 && alpha == 0) {
      alpha = Math.toDegrees(odometer.getTheta());
      Sound.beep();
      leftMotor.rotate(0);
      rightMotor.rotate(0);
      turnTo(360);
    } else if (distance < THRESHOLD && previous > 100 && alpha != 0) {
      beta = Math.toDegrees(odometer.getTheta());
      leftMotor.rotate(0);
      rightMotor.rotate(0);

      // quit the while loop in run()
      condition = false;

      // deltaTheta
      if (alpha < beta) {
        deltaTheta = 55 - (alpha - beta) / 2;
      } else {
        deltaTheta = 265 - (alpha - beta) / 2;
      }

      // Setting odometer
      odometer.setTheta(Math.toRadians(deltaTheta));
      turnTo(-deltaTheta);
    }

    previous = distance;

    done = true;
    return done;
  }



  /**
   * This method determines at which angle the robot should go to. It should do so with a MINIMAL
   * angle.
   * 
   * @param theta if positive then the robot will turn to the right, else it will rotate the the
   *        left
   */
  public void turnTo(double theta) {

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    if (theta < 0) { // if angle is negative, turn to the left
      leftMotor.rotate(-convertAngle(leftRadius, width, -theta), true);
      rightMotor.rotate(convertAngle(rightRadius, width, -theta), true);
    } else { // angle is positive, turn to the right
      leftMotor.rotate(convertAngle(leftRadius, width, theta), true);
      rightMotor.rotate(-convertAngle(rightRadius, width, theta), true);
    }
  }

  /**
   * This method returns the moving average
   * 
   * @param value is the current measured value
   * @return the corrected moving average
   */
  public double next(int value) {

    // getting a starting sample as a base line
    if (queue.size() < this.size) {
      queue.addLast(value);;
      int sum = 0;
      for (int i : queue) {
        sum = sum + i;
      }

      average = (double) sum / queue.size();
      return average;
      // calculating the moving average
      // average = average + Top of the stack - Bottom of the stack
      // current average = previous average + (current value - last value)/size
    } else {
      int head = queue.removeFirst();
      queue.addLast(value);
      this.average = average + (value - head) / size;
      return this.average;
    }
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

}

package ca.mcgill.ecse211.project;

import lejos.hardware.Battery;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;


public class MainController {

  public static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  public static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

  public static final EV3LargeRegulatedMotor topMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));


  public static final double WHEEL_RADIUS = 2.1;
  public static final double TRACK = 16.2;

  private static final double tile = 30.48;
  static int X0 = 6;
  static int Y0 = 2;
  static int XC = 2;
  static int YC = 6;
  static int SC = 0;
  static int XZ = 7;
  static int YZ = 6;


  /**
   * @param args
   */
  public static void main(String[] args) {

    // Initialize variables:
    int buttonChoice;
    final TextLCD t = LocalEV3.get().getTextLCD();
    SensorLogger logger = new SensorLogger(); // writes data to local file


    Odometer odometer = new Odometer(leftMotor, rightMotor);
    OdometryDisplay odoDisplay = new OdometryDisplay(odometer, t);
    final Navigation navigator =
        new Navigation(odometer, leftMotor, rightMotor, WHEEL_RADIUS, WHEEL_RADIUS, TRACK);

    UltrasonicLocalizer usLocalizer =
        new UltrasonicLocalizer(odometer, leftMotor, rightMotor, WHEEL_RADIUS, WHEEL_RADIUS, TRACK);

    final LightLocalizer ltLocalizer =
        new LightLocalizer(odometer, leftMotor, rightMotor, WHEEL_RADIUS, WHEEL_RADIUS, TRACK, SC);

    // Beep twice to let the user know battery is low
    if (Battery.getVoltage() < 7.8) {
      Sound.twoBeeps();
    }



    // If the "Log.txt" file exists then delete it.
    if (logger.getFile() != null) {
      logger.getFile().delete();
    }


    // // clear the display
    // t.clear();
    // // ask the user to input the value for x0 coordinate
    // t.drawString("enter the x0 value now", 0, 0);
    // buttonChoice = Button.waitForAnyPress();
    // // initial x = 0;
    // int x = 0;
    // // max x = 8
    //
    // while (buttonChoice != Button.ID_RIGHT) {
    // buttonChoice = Button.waitForAnyPress();
    // // increment x0 by pushing the up button
    // if (buttonChoice == Button.ID_UP && x <= 8) {
    // x++;
    // t.drawInt(x, 0, 1);
    // }
    // // decrement x0 by pushing the down button
    // if (buttonChoice == Button.ID_DOWN && x >= 1) {
    // x--;
    // t.drawInt(x, 0, 1);
    // }
    // }
    //
    //
    // // confirm the value by pushing the right button
    // // * and clear the screen for the next entry
    //
    // if (buttonChoice == Button.ID_RIGHT) {
    // X0 = x;
    // t.clear();
    // }
    //
    //
    // // ask the user to enter the value of y0 coodinates
    // t.drawString("enter the y0 value now", 0, 0);
    // buttonChoice = Button.waitForAnyPress();
    // int y = 0;
    // // max y = 8
    // while (buttonChoice != Button.ID_RIGHT) {
    // buttonChoice = Button.waitForAnyPress();
    // // increment y0 by pushing the up button
    // if (buttonChoice == Button.ID_UP) {
    // y++;
    // t.drawInt(y, 0, 1);
    // }
    // // decrement y0 by pushing the down button
    // if (buttonChoice == Button.ID_DOWN) {
    // y--;
    // t.drawInt(y, 0, 1);
    // }
    // }
    // // confirm the entry by pushing the right button
    // // * and clear the screen for the next entry
    //
    // if (buttonChoice == Button.ID_RIGHT) {
    // Y0 = y;
    // t.clear();
    // }
    //
    //
    // // ask the user to enter the value of Xc
    // t.drawString("enter the Xc value now", 0, 0);
    // buttonChoice = Button.waitForAnyPress();
    // int Xc = 0;
    // // max Xc = 8;
    // while (buttonChoice != Button.ID_RIGHT) {
    // buttonChoice = Button.waitForAnyPress();
    // // increment Xc by pushing the up button
    // if (buttonChoice == Button.ID_UP) {
    // Xc++;
    // t.drawInt(Xc, 0, 1);
    // }
    // // decrement Xc by pushing the down button
    // if (buttonChoice == Button.ID_DOWN) {
    // Xc--;
    // t.drawInt(Xc, 0, 1);
    // }
    // }
    // // confirm the entry by pushing the right button
    // // * and clear the screen for the next entry
    //
    // if (buttonChoice == Button.ID_RIGHT) {
    // XC = Xc;
    // t.clear();
    // }
    //
    // // ask the user to enter the value of Yc
    // t.drawString("enter the Yc value now ", 0, 0);
    // buttonChoice = Button.waitForAnyPress();
    // int Yc = 0;
    // while (buttonChoice != Button.ID_RIGHT) {
    // buttonChoice = Button.waitForAnyPress();
    // // increment Yc by pushing the up button
    // if (buttonChoice == Button.ID_UP) {
    // Yc++;
    // t.drawInt(Yc, 0, 1);
    // }
    // // decrement Yc by pushing the down button
    // if (buttonChoice == Button.ID_DOWN) {
    // Yc--;
    // t.drawInt(Yc, 0, 1);
    // }
    // }
    // // confirm the entry by pushing the right button
    // // * and clear the screen for the next entry
    //
    // if (buttonChoice == Button.ID_RIGHT) {
    // YC = Yc;
    // t.clear();
    // }
    //
    // // ask the user to enter SC
    // t.drawString("Select SC", 0, 0);
    // buttonChoice = Button.waitForAnyPress();
    // if (buttonChoice == Button.ID_UP) {
    // SC = 0;
    // } else if (buttonChoice == Button.ID_DOWN) {
    // SC = 1;
    // } else if (buttonChoice == Button.ID_RIGHT) {
    // SC = 2;
    // } else if (buttonChoice == Button.ID_RIGHT) {
    // SC = 3;
    // } else {
    // SC = 0; // default start
    // }

    // if (buttonChoice == Button.ID_ENTER) {
    // t.clear();
    // }

    // clear the display for the odometry info
    t.clear();
    odometer.start();
    odoDisplay.start();

    // Start Ultrasonic Localization by pushing any button except ESCAPE
    // buttonChoice = Button.waitForAnyPress();
    // if (buttonChoice != Button.ID_ESCAPE) {
    // usLocalizer.start();
    // } else {
    // System.exit(0);
    // }
    //
    // // Start Light Localization after Ultrasonic by pushing any button except ESCAPE
    // buttonChoice = Button.waitForAnyPress();
    // if (buttonChoice != Button.ID_ESCAPE) {
    // ltLocalizer.start();
    // } else {
    // System.exit(0);
    // }
    //
    // buttonChoice = Button.waitForAnyPress();
    // if (buttonChoice != Button.ID_ESCAPE) {
    // (new Thread() {
    // public void run() {
    // navigator.drive(X0, Y0); // Drive to X0, Y0
    // return;
    // }
    // }).start();
    // } else {
    // System.exit(0);
    // }

    buttonChoice = Button.waitForAnyPress();
    if (buttonChoice != Button.ID_ESCAPE) { // After Reaching X0 Y0
      topMotor.setSpeed(150); // Start Top Motors to traverse zipline
      topMotor.backward();
      (new Thread() {
        public void run() {
          XZ = X0 + 6;
          navigator.roll((XZ - X0));
          // Roll bottom motors until XZ the end
          navigator.turnTo(30);
          return;
        }
      }).start();
    }

    if (buttonChoice == Button.ID_ESCAPE) {
      System.exit(0);
    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}

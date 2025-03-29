// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.util.Color;

/**
 * Methods to help work with color detection.
 */
public class ColorHelpers {

  /**
   * Calculates the hue of the given color.
   * Source:
   * https://www.geeksforgeeks.org/program-change-rgb-color-model-hsv-color-model/
   * 
   * @param color The desired color.
   * @return The hue value of the desired color.
   */
  public static double getHue(Color color) {
    double r = color.red;
    double g = color.green;
    double b = color.blue;

    // R, G, B values are divided by 255
    // to change the range from 0..255 to 0..1
    r = r / 255.0;
    g = g / 255.0;
    b = b / 255.0;

    // h, s, v = hue, saturation, value
    double cmax = Math.max(r, Math.max(g, b)); // maximum of r, g, b
    double cmin = Math.min(r, Math.min(g, b)); // minimum of r, g, b
    double diff = cmax - cmin; // diff of cmax and cmin.
    double h = -1;

    // if cmax and cmax are equal then h = 0
    if (cmax == cmin)
      h = 0;

    // if cmax equal r then compute h
    else if (cmax == r)
      h = (60 * ((g - b) / diff) + 360) % 360;

    // if cmax equal g then compute h
    else if (cmax == g)
      h = (60 * ((b - r) / diff) + 120) % 360;

    // if cmax equal b then compute h
    else if (cmax == b)
      h = (60 * ((r - g) / diff) + 240) % 360;

    // Return hue value
    return h;
  }

  /**
   * Compares two colors using their hues and returns whether they match within a
   * given threshold.
   * 
   * @param color1    The first color to compare.
   * @param color2    The second color to compare.
   * @param threshold The allowed hue threshold.
   * @return Whether the colors are within the threshold.
   */
  public static boolean colorsMatch(Color color1, Color color2, double threshold) {
    double hue1 = getHue(color1);
    double hue2 = getHue(color2);

    if (Math.abs(hue1 - hue2) < threshold) {
      return true;
    }

    return false;
  }

}

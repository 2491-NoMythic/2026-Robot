// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

import java.util.ArrayList;
import org.opencv.core.Mat.Tuple2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class MythicalMath {
  /**
   * finds the absolute distance from the origin of any point on a 3d plane
   *
   * @param XfromOrigin x-coordinate of point
   * @param YfromOrigin y-coordinate of point
   * @param ZfromOrigin z-coordinate of point
   * @return distance from origin of point
   */
  public static double DistanceFromOrigin3d(
      double XfromOrigin, double YfromOrigin, double ZfromOrigin) {
    double Distance2d = Math.sqrt(Math.pow(YfromOrigin, 2) + Math.pow(XfromOrigin, 2));
    return Math.sqrt(Math.pow(Distance2d, 2) + Math.pow(ZfromOrigin, 2));
  }

  /**
   * mirrors a starting pose around a coordinate. very useful for changing a
   * coordinate from blue alliance to red alliance. The angle will also flip 180
   * degrees
   * 
   * @param startingPose
   * @param pivotX       the x coordinate of the point to mirror around
   * @param pivotY       the y coordinate of the point to mirror around
   * @return a new Pose2d, which is the starting pose mirrored around the pivot
   *         point
   */
  public static Pose2d mirrorPoseAround(Pose2d startingPose, double pivotX, double pivotY) {
    double mirroredX = 2 * pivotX - startingPose.getX();
    double mirroredY = 2 * pivotY - startingPose.getY();
    return new Pose2d(mirroredX, mirroredY, Rotation2d.fromRadians(-startingPose.getRotation().getRadians()));
  }

  public static Pose2d multiplyOnlyPos(Pose2d pose, Double scalar) {
    return new Pose2d(pose.getX() * scalar, pose.getY() * scalar, pose.getRotation());
  }

  public static Pose2d divideOnlyPos(Pose2d pose, Double scalar) {
    return new Pose2d(pose.getX() / scalar, pose.getY() / scalar, pose.getRotation());
  }

  /**
   * limits a number in both the positive and negative direction by a number
   * 
   * @param number the number to limit
   * @param cap    a POSITIVE number to be the limit in the negative and positive
   *               direction
   * @return
   */
  public static double absoluteCap(double number, double cap) {
    return cap(number, -cap, cap);
  }

  /**
   * limits a number with an upper and lower limit. Will return the upper or lower
   * limit if the number exceeds that.
   * 
   * @param number   the number to limit
   * @param lowerCap the number to return if NUMBER is less than it
   * @param upperCap the number to return if NUMBER is greater than it
   * @return
   */
  public static double cap(double number, double lowerCap, double upperCap) {
    if (number < lowerCap) {
      return lowerCap;
    }
    if (number > upperCap) {
      return upperCap;
    }
    return number;
  }

  /**
   * returns the minimum of two values, but treats any vaue that is 0 as 10,000
   * 
   * @param value1
   * @param value2
   * @return
   */
  public static double minNotZero(double value1, double value2) {
    if (value1 == 0) {
      value1 = 10000;
    }
    if (value2 == 0) {
      value2 = 10000;
    }
    return Math.min(value1, value2);
  }

  /**
   * @param pose1
   * @param pose2
   * @return poses added together, with pose1's rotation
   */
  public static Pose2d addOnlyPosTogether(Pose2d pose1, Pose2d pose2) {
    return new Pose2d(
        pose1.getX() + pose2.getX(), pose1.getY() + pose2.getY(), pose1.getRotation());
  }

  /**
   * calculates the distance between two Pose2d's using the
   * .getTranslation.getDistance(Translaiton2d) method
   * 
   * @param pose1
   * @param pose2
   * @return the distance in units, will always be positive
   */
  public static double distanceBetweenTwoPoses(Pose2d pose1, Pose2d pose2) {
    return pose1.getTranslation().getDistance(pose2.getTranslation());
  }

  public static Double getSmallest(Double a, Double b, Double c) {
    // Replace null values with Double.MAX_VALUE (a very large number)
    double valA = (a != null) ? a : Double.MAX_VALUE;
    double valB = (b != null) ? b : Double.MAX_VALUE;
    double valC = (c != null) ? c : Double.MAX_VALUE;

    // Find the smallest value
    return Math.min(valA, Math.min(valB, valC));
  }

  /**
   * calculates the desired 3d launch angle to hit a target, given the coordinates of the target and robot, 
   * @param origin the 3d coordinates of the releast point of the gamepiece
   * @param target the 3d coordinates of the target
   * @param initialVelocity the velocity that the gamepiece leaves the robot at
   * @param inheritedVelocity the velocity of the robot when the shot is fired.
   * @return (pitch, yaw) where pitch is up and yaw is left/right in radians        //a 3d launch angle
   */
  public static Tuple2<Double> aimProjectileAtPoint(Translation3d origin, Translation3d target, float initialVelocity, Translation3d inheritedVelocity)
  {
    float radius = 0f;
    Translation3d velocity = inheritedVelocity;
    Translation3d position = origin;

    float timeStep = 0.1f; //Checks per second
    float maxTime = 5f; //In seconds
    int goalType = 1; //0 or 1, 0 shoots to hit the target while 1 shoots to fall into the target. For REBUILT we want type 1.

    var solutions = new ArrayList<Translation3d>();

    float gravity = -9.81f;
    Boolean lastTargetInsideSphereValue = false;

    for(int sample = 0; sample <= maxTime/timeStep; sample++){
      
      radius += initialVelocity * timeStep; //Calculate an expanding sphere to represent all possible shots simultaneously

      velocity = velocity.plus(new Translation3d(0, 0, gravity * timeStep)); //Apply gravity and acceleration to velocity of the sphere
      position = position.plus(velocity.times(timeStep)); //Apply velocity to position of the sphere

      Boolean targetInsideSphere = (position.getDistance(target) < radius); //Is the target point inside the sphere?
      
      if(targetInsideSphere != lastTargetInsideSphereValue){ //Are we on the surface of the sphere - have we just changed from being outside to inside or vice versa
        
        solutions.add(position); //Then this is a solution
        
        lastTargetInsideSphereValue = targetInsideSphere;
      }
    }

    if(solutions.size() > 1){ //Are there multiple solutions (should be 2)?
      
      Translation3d chosenSolution = solutions.get(goalType); //Grab the appropriate solution for how we want to hit the target
      Translation3d direction = target.minus(chosenSolution); //Dangerous, does this mutate target directly? IDK
      direction = direction.times(1/direction.getNorm());

      double pitch = Math.asin(direction.getZ());
      double yaw = Math.atan2(direction.getX(), direction.getY());


      //Rotation3d rotation = new Rotation3d(chosenSolution.toVector(), target.toVector()); //TODO: test this, does it do what we expect?
    
      return new Tuple2<Double>(pitch, yaw);
    }

    return null;

    //TODO: simulate found shot and make adjustments 
  }


  /**
   * calculates the length a linear actuator must extend by to angle the shooter 
   * @param desiredAngleInDegrees the desired angle, in degrees
   * @param actuatorBodyLength the length of the actuator WHEN NOT EXTENDED AT ALL
   * @param sideALength the length from the center of the bottom pivot point of the actuator where it is mounted, to the center of the pivot point of the shooter hood piece
   * @param sideBLength the length from the center of the top pivot point of the actuator where it is mounted, to the center of the pivot point of the shooter hood piece
   * @param lowerAngleToHorizontal the angle between side A and horizontal
   * @return the length, in whatever units were inputted, to extend the actuator by. ideally in cm.
   */
  public double ServoExtensionToReachHoodAngleInDegrees(double desiredAngleInDegrees, double actuatorBodyLength, double sideALength, double sideBLength, double lowerAngleToHorizontal){
    double a = sideALength; 
    double b = sideBLength;
    double totalAngle = desiredAngleInDegrees + lowerAngleToHorizontal;

    double totalActuatorSideLength = Math.sqrt( -2 * a * b * Math.cos(totalAngle) + a*a + b*b); 
    double extension = totalActuatorSideLength - actuatorBodyLength;
    //I don't want to document this, so I will just link a photo of the math. 
    //https://nomythic.slack.com/archives/CA2SX3M8T/p1771377556413989
    //TLDR, we know 3 sides, two of which are fixed, so this is solving the Law Of Cosines for the angle, 
    //and then finding only the portion above horizontal.

    return extension;
  }

  /**
   * calculates expected acceleration due to air resistance base on exit velocity of ball
   * @param currentVelocity
   * @return acceleration
   */
  public double airResistance(double currentVelocity) {
    return currentVelocity; //eventulaly this will return the acceleration on the fuel due to air resistance, haven't done the calculations yet
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.videoplayer;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.bytedeco.javacv.FFmpegFrameGrabber;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.FFmpegFrameGrabber.Exception;
import org.littletonrobotics.junction.Logger;
import org.bytedeco.javacv.OpenCVFrameConverter;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SimulationVideoConstants;

public class VideoPlayer extends SubsystemBase {
  private final String videoPath;
  private final List<Boolean[][]> frames = new ArrayList<>();
  private final List<Pose2d> poses = new ArrayList<>();

  /**
   * Creates a new VideoPlayer.
   * 
   * @param videoPath The path of the video file relative to the deploy folder.
   * @apiNote The video file MUST be located in the deploy folder.
   */
  public VideoPlayer(String videoPath) {
    this.videoPath = videoPath;
    if (isAllowed()) {
      processVideo();
      resetPoses();
    }
  }

  @Override
  public void periodic() {
    if (isAllowed()) {
      Pose2d[] poseArray = new Pose2d[poses.size()];
      poses.toArray(poseArray);
      Logger.recordOutput("VideoPlayer/Display/Chunks", poseArray);
    }
  }

  private void processVideo() {
    frames.clear();

    double startTime = Timer.getFPGATimestamp();
    File videoFile = new File(Filesystem.getDeployDirectory().getPath() + "\\" + videoPath);

    if (!videoFile.exists() || !videoFile.canRead()) {
      System.err.println("Video " + videoPath + " doesn't exist or cannot be read.");
      return;
    }

    FFmpegFrameGrabber frameGrabber = new FFmpegFrameGrabber(videoFile);

    int chunkCount = 0;
    int frameCount = 0;
    int validFrameCount = 0;
    int nullFrameCount = 0;
    try (OpenCVFrameConverter.ToMat converter = new OpenCVFrameConverter.ToMat()) {
      frameGrabber.start();
      Frame frame;

      // Iterate through each frame
      while ((frame = frameGrabber.grab()) != null) {
        frameCount++;
        Mat inputMat = converter.convertToOrgOpenCvCoreMat(frame);
        if (inputMat == null) {
          nullFrameCount++;
          continue;
        }
        validFrameCount++;
        System.out.println("[" + frameCount + "] Reading Mat...");

        Mat processedMat = new Mat();
        Imgproc.cvtColor(inputMat, processedMat, Imgproc.COLOR_BGR2GRAY);
        Imgproc.threshold(
            processedMat,
            processedMat,
            128,
            255,
            Imgproc.THRESH_BINARY);

        Boolean[][] pixels = new Boolean[SimulationVideoConstants.kDisplayHeight][SimulationVideoConstants.kDisplayWidth];

        int imgWidth = inputMat.cols();
        int imgHeight = inputMat.rows();
        int chunkWidth = inputMat.cols() / SimulationVideoConstants.kDisplayWidth;
        int chunkHeight = inputMat.rows() / SimulationVideoConstants.kDisplayHeight;

        // Dividing the frame data into x and y amount of chunks
        for (int y = 0; y < SimulationVideoConstants.kDisplayHeight; y++) {
          for (int x = 0; x < SimulationVideoConstants.kDisplayWidth; x++) {
            Mat chunk = processedMat.submat(
                y * chunkHeight,
                Math.min((y + 1) * (chunkHeight), imgHeight),
                x * chunkWidth,
                Math.min((x + 1) * (chunkWidth), imgWidth));

            int brightPixels = Core.countNonZero(chunk);
            int totalPixels = chunk.rows() * chunk.cols();
            pixels[y][x] = brightPixels > (totalPixels / 2);

            chunkCount++;
          }
        }

        frames.add(pixels);
      }

    } catch (Exception e) {
      e.printStackTrace();
    }
    try {
      frameGrabber.close();
    } catch (org.bytedeco.javacv.FrameGrabber.Exception ignore) {
    }
    System.out.println(String.format(
        "========================================================\n" +
            "====================================================================================\n" +
            "====================================================================================\n" +
            "VIDEO PROCESSING RESULTS:\n" +
            "Number of valid frames: %s\n" +
            "Number of null frames: %s\n" +
            "Total number of frames read: %s\n" +
            "Number of chunks: %s\n" +
            "Processing time: %s seconds\n" +
            "====================================================================================\n" +
            "====================================================================================\n" +
            "====================================================================================\n\n\n",
        validFrameCount, nullFrameCount, frameCount, chunkCount,
        Math.round((Timer.getFPGATimestamp() - startTime) * 1000.0) / 1000.0));
    System.out.println(frames.get(0)[0].length + " x " + frames.get(0).length);

    Logger.recordOutput("VideoPlayer/ValidFrameCount", validFrameCount);
    Logger.recordOutput("VideoPlayer/NullFrameCount", nullFrameCount);
    Logger.recordOutput("VideoPlayer/TotalFrameCount", frameCount);
    Logger.recordOutput("VideoPlayer/ChunkCount", chunkCount);
  }

  /**
   * Resets all poses to their default location
   */
  public void resetPoses() {
    poses.clear();
    for (int x = 0; x < SimulationVideoConstants.kDisplayWidth; x++) {
      for (int y = 0; y < SimulationVideoConstants.kDisplayHeight; y++) {
        poses.add(calculateDefaultPose(x, y));
      }
    }
  }

  /**
   * Calculate the default position of a pose when it's enabled, given it's
   * position relative to the display.
   * 
   * @param x X-axis
   * @param y Y-axis
   * @return New {@code Pose2d} object
   */
  public Pose2d calculateDefaultPose(int x, int y) {
    return new Pose2d(
        (x * SimulationVideoConstants.kDisplayGap) + SimulationVideoConstants.kDisplayOffsetX,
        FieldConstants.kFieldWidthY
            - ((y * SimulationVideoConstants.kDisplayGap) + SimulationVideoConstants.kDisplayOffsetY),
        Rotation2d.kZero);
  }

  /**
   * Get a specidifc frame from the processed video
   * 
   * @param index The frame index
   * @return The frame
   */
  public Boolean[][] getFrame(int index) {
    if (index >= frames.size()) {
      System.err.println("Index " + index + " is out of bounds. There is no such frame!");
      return new Boolean[SimulationVideoConstants.kDisplayHeight][SimulationVideoConstants.kDisplayWidth];
    }
    return frames.get(index);
  }

  /**
   * Get the total frame count of the processed video
   * 
   * @return THe total frame count
   */
  public int getFrameCount() {
    return frames.size();
  }

  /**
   * Get the list of poses being plotted on the field.
   * 
   * @return {@code ArrayList} of {@code Pose2d}s
   */
  public List<Pose2d> getPoses() {
    return poses;
  }

  /**
   * Safeguard in case someone runs this on the real robot.
   * 
   * @return {@cod true} if playback is allowed, {@code false} if not.
   */
  public boolean isAllowed() {
    return RobotBase.isSimulation();
  }
}

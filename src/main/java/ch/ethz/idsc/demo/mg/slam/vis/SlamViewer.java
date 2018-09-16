// code by mg
package ch.ethz.idsc.demo.mg.slam.vis;

import java.util.Timer;
import java.util.TimerTask;

import ch.ethz.idsc.demo.mg.slam.SlamContainer;
import ch.ethz.idsc.demo.mg.slam.algo.PeriodicSlamStep;
import ch.ethz.idsc.demo.mg.slam.algo.prc.SlamCurveContainer;
import ch.ethz.idsc.demo.mg.slam.config.SlamConfig;
import ch.ethz.idsc.gokart.core.pos.GokartPoseInterface;
import ch.ethz.idsc.retina.util.StartAndStoppable;
import ch.ethz.idsc.retina.util.math.Magnitude;

/** SLAM algorithm visualization wrapper. PeriodicSlamStep is implemented to have access to a time stamp for saving of frames */
public class SlamViewer extends PeriodicSlamStep implements StartAndStoppable {
  private final GokartPoseInterface gokartLidarPose;
  private final SlamCurveContainer slamCurveContainer;
  private final SlamMapFrame[] slamMapFrames;
  private final SlamMapGUI slamMapGUI;
  private final SlamSaveFrame slamSaveFrame;
  // ---
  private final Timer timer = new Timer();
  private final TimerTask visualizationTask = new TimerTask() {
    @Override
    public void run() {
      visualizationTask();
    }
  };
  private final long visualizationInterval;

  public SlamViewer(SlamConfig slamConfig, SlamContainer slamContainer, SlamCurveContainer slamCurveContainer, GokartPoseInterface gokartLidarPose) {
    super(slamContainer, slamConfig.savingInterval);
    this.gokartLidarPose = gokartLidarPose;
    this.slamCurveContainer = slamCurveContainer;
    slamMapGUI = new SlamMapGUI(slamConfig);
    slamMapFrames = new SlamMapFrame[2];
    for (int i = 0; i < slamMapFrames.length; ++i)
      slamMapFrames[i] = new SlamMapFrame(slamConfig);
    slamSaveFrame = new SlamSaveFrame(slamConfig, slamMapFrames);
    // ---
    visualizationInterval = Magnitude.MILLI_SECOND.toLong(slamConfig.visualizationInterval);
  }

  @Override // from StartAndStoppable
  public void start() {
    timer.schedule(visualizationTask, 0, visualizationInterval);
  }

  @Override // from StartAndStoppable
  public void stop() {
    timer.cancel();
    slamMapGUI.dispose();
  }

  private void visualizationTask() {
    slamMapGUI.setFrames(StaticHelper.constructFrames(slamMapFrames, slamContainer, slamCurveContainer, gokartLidarPose.getPose()));
  }

  @Override // from PeriodicSlamStep
  protected void periodicTask(int currentTimeStamp, int lastComputationTimeStamp) {
    slamSaveFrame.saveFrame(currentTimeStamp);
  }
}

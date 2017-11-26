// code by jph
package ch.ethz.idsc.retina.lcm.lidar;

import ch.ethz.idsc.retina.dev.lidar.LidarAngularFiringCollector;
import ch.ethz.idsc.retina.dev.lidar.LidarRotationProvider;
import ch.ethz.idsc.retina.dev.lidar.LidarSpacialProvider;
import ch.ethz.idsc.retina.dev.lidar.VelodyneDecoder;
import ch.ethz.idsc.retina.dev.lidar.VelodyneModel;
import ch.ethz.idsc.retina.dev.lidar.vlp16.Vlp16Decoder;
import ch.ethz.idsc.retina.dev.lidar.vlp16.Vlp16SpacialProvider;

public class Vlp16LcmHandler {
  public static final int MAX_COORDINATES = 2304 * 32;
  // ---
  public final LidarAngularFiringCollector lidarAngularFiringCollector = //
      new LidarAngularFiringCollector(MAX_COORDINATES, 3);
  private final VelodyneLcmClient velodyneLcmClient;

  public Vlp16LcmHandler(String lidarId) {
    VelodyneModel velodyneModel = VelodyneModel.VLP16;
    VelodyneDecoder velodyneDecoder = new Vlp16Decoder();
    velodyneLcmClient = new VelodyneLcmClient(velodyneModel, velodyneDecoder, lidarId);
    LidarSpacialProvider lidarSpacialProvider = new Vlp16SpacialProvider();
    lidarSpacialProvider.addListener(lidarAngularFiringCollector);
    LidarRotationProvider lidarRotationProvider = new LidarRotationProvider();
    lidarRotationProvider.addListener(lidarAngularFiringCollector);
    velodyneDecoder.addRayListener(lidarSpacialProvider);
    velodyneDecoder.addRayListener(lidarRotationProvider);
    // ---
    velodyneLcmClient.startSubscriptions();
  }

  public void stopSubscriptions() {
    velodyneLcmClient.stopSubscriptions();
  }
}
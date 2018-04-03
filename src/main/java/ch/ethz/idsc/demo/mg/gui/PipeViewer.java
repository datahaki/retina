//code by jph (adapted)
package ch.ethz.idsc.demo.mg.gui;

import ch.ethz.idsc.retina.dev.davis.DavisDevice;
import ch.ethz.idsc.retina.dev.davis._240c.Davis240c;
import ch.ethz.idsc.retina.dev.davis.app.AbstractAccumulatedImage;
import ch.ethz.idsc.retina.dev.davis.app.AccumulatedEventsGrayImage;
import ch.ethz.idsc.retina.lcm.davis.DavisImuLcmClient;
import ch.ethz.idsc.retina.lcm.davis.DavisLcmClient;
import ch.ethz.idsc.retina.util.StartAndStoppable;

// draft for pipeline
public class PipeViewer implements StartAndStoppable {
 private final DavisLcmClient davisLcmClient;
 private final DavisImuLcmClient davisImuLcmClient;
 public final PipeViewerFrame pipeViewerFrame;

 public PipeViewer(String cameraId) {
   DavisDevice davisDevice = Davis240c.INSTANCE;
   davisLcmClient = new DavisLcmClient(cameraId);
   AbstractAccumulatedImage abstractAccumulatedImage = AccumulatedEventsGrayImage.of(davisDevice);
   abstractAccumulatedImage.setInterval(25_000);
   pipeViewerFrame = new PipeViewerFrame(davisDevice, abstractAccumulatedImage);
   // handle dvs
   davisLcmClient.davisDvsDatagramDecoder.addDvsListener(abstractAccumulatedImage);
//   davisLcmClient.davisDvsDatagramDecoder.addDvsListener(davisViewerFrame.davisTallyProvider.dvsListener);
//   davisLcmClient.davisDvsDatagramDecoder.addDvsListener(davisViewerFrame.dvsTallyProvider);
//   // handle aps
//   davisLcmClient.davisSigDatagramDecoder.addListener(davisViewerFrame.davisViewerComponent.sigListener);
//   davisLcmClient.davisSigDatagramDecoder.addListener(davisViewerFrame.davisTallyProvider.sigListener);
//   // handle aps
//   davisLcmClient.davisRstDatagramDecoder.addListener(davisViewerFrame.davisViewerComponent.rstListener);
//   davisLcmClient.davisRstDatagramDecoder.addListener(davisViewerFrame.davisTallyProvider.rstListener);
//   // handle dif
//   DavisImageBuffer davisImageBuffer = new DavisImageBuffer();
//   davisLcmClient.davisRstDatagramDecoder.addListener(davisImageBuffer);
//   SignalResetDifference signalResetDifference = SignalResetDifference.amplified(davisImageBuffer);
//   davisLcmClient.davisSigDatagramDecoder.addListener(signalResetDifference);
//   signalResetDifference.addListener(davisViewerFrame.davisViewerComponent.difListener);
   // handle imu
   davisImuLcmClient = new DavisImuLcmClient(cameraId);
   davisImuLcmClient.addListener(pipeViewerFrame.pipeViewerComponent);
 }

 @Override
 public void start() {
   // start to listen
   davisLcmClient.startSubscriptions();
   davisImuLcmClient.startSubscriptions();
 }

 @Override
 public void stop() {
   davisImuLcmClient.stopSubscriptions();
   davisLcmClient.stopSubscriptions();
   pipeViewerFrame.jFrame.setVisible(false);
   pipeViewerFrame.jFrame.dispose();
 }
}
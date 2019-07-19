// code by am and jph
package ch.ethz.idsc.gokart.core.adas;

import java.util.Objects;
import java.util.Optional;

import ch.ethz.idsc.gokart.core.man.ManualConfig;
import ch.ethz.idsc.gokart.dev.steer.SteerColumnTracker;
import ch.ethz.idsc.gokart.dev.steer.SteerGetEvent;
import ch.ethz.idsc.gokart.dev.steer.SteerGetListener;
import ch.ethz.idsc.gokart.dev.steer.SteerPutEvent;
import ch.ethz.idsc.gokart.dev.steer.SteerPutProvider;
import ch.ethz.idsc.gokart.dev.steer.SteerSocket;
import ch.ethz.idsc.gokart.gui.GokartLcmChannel;
import ch.ethz.idsc.gokart.lcm.BinaryBlobPublisher;
import ch.ethz.idsc.gokart.lcm.VectorFloatBlob;
import ch.ethz.idsc.owl.ani.api.ProviderRank;
import ch.ethz.idsc.retina.joystick.ManualControlInterface;
import ch.ethz.idsc.retina.joystick.ManualControlProvider;
import ch.ethz.idsc.retina.util.math.SI;
import ch.ethz.idsc.retina.util.sys.AbstractModule;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Flatten;
import ch.ethz.idsc.tensor.io.Timing;
import ch.ethz.idsc.tensor.opt.Pi;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.sca.Sin;

public class SteerVibrationModule extends AbstractModule implements SteerPutProvider, SteerGetListener {
  private final ManualControlProvider manualControlProvider = ManualConfig.GLOBAL.getProvider();
  private final SteerColumnTracker steerColumnTracker = SteerSocket.INSTANCE.getSteerColumnTracker();
  private SteerGetEvent steerGetEvent;
  private final Timing timing = Timing.started();
  private final BinaryBlobPublisher binaryBlobPublisher = new BinaryBlobPublisher(GokartLcmChannel.STEER_VIBRATE);

  @Override
  protected void first() {
    SteerSocket.INSTANCE.addPutProvider(this);
  }

  @Override
  protected void last() {
    SteerSocket.INSTANCE.removePutProvider(this);
  }

  @Override
  public ProviderRank getProviderRank() {
    return ProviderRank.MANUAL;
  }

  @Override
  public Optional<SteerPutEvent> putEvent() {
    Optional<ManualControlInterface> optional = manualControlProvider.getManualControl();
    if (steerColumnTracker.isCalibratedAndHealthy() && optional.isPresent() && Objects.nonNull(steerGetEvent)) {
      ManualControlInterface manualControlInterface = optional.get();
      System.out.println(steerColumnTracker.getSteerColumnEncoderCentered() + " " + steerGetEvent.tsuTrq());
      binaryBlobPublisher.accept(VectorFloatBlob.encode(Flatten.of(Tensors.of( //
          time2torque(Quantity.of(timing.seconds(), SI.SECOND)), steerColumnTracker.getSteerColumnEncoderCentered(), steerGetEvent.tsuTrq()))));
      if (manualControlInterface.isAutonomousPressed())
        return Optional.of(SteerPutEvent.createOn(time2torque(Quantity.of(timing.seconds(), SI.SECOND))));
    }
    return Optional.empty();
  }/* package */ Scalar time2torque(
      Scalar time) {
    Scalar frequency = HapticSteerConfig.GLOBAL.vibrationFrequency;
    Scalar amplitude = HapticSteerConfig.GLOBAL.vibrationAmplitude;
    Scalar radian = frequency.multiply(time).multiply(Pi.TWO);
    return Sin.FUNCTION.apply(radian).multiply(amplitude);
  }

  @Override
  public void getEvent(SteerGetEvent getEvent) {
    this.steerGetEvent = getEvent;
  }
}

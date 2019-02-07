// code by jph
package ch.ethz.idsc.gokart.offline.tab;

import java.nio.ByteBuffer;

import ch.ethz.idsc.gokart.offline.api.OfflineTableSupplier;
import ch.ethz.idsc.gokart.offline.channel.SingleChannelInterface;
import ch.ethz.idsc.retina.util.math.Magnitude;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.io.TableBuilder;

public class SingleChannelTable implements OfflineTableSupplier {
  public static OfflineTableSupplier of(SingleChannelInterface singleChannelInterface) {
    return new SingleChannelTable(singleChannelInterface);
  }

  // ---
  private final TableBuilder tableBuilder = new TableBuilder();
  private final SingleChannelInterface singleChannelInterface;
  private final String channel;

  private SingleChannelTable(SingleChannelInterface singleChannelInterface) {
    this.singleChannelInterface = singleChannelInterface;
    channel = singleChannelInterface.channel();
  }

  @Override // from OfflineLogListener
  public final void event(Scalar time, String channel, ByteBuffer byteBuffer) {
    if (channel.equals(this.channel))
      tableBuilder.appendRow( //
          time.map(Magnitude.SECOND), //
          singleChannelInterface.row(byteBuffer));
  }

  @Override // from OfflineTableSupplier
  public final Tensor getTable() {
    return tableBuilder.toTable();
  }
}

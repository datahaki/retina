// code by jph
package ch.ethz.idsc.retina.lcm.davis;

import ch.ethz.idsc.retina.dev.davis.DavisApsType;
import junit.framework.TestCase;

public class DavisApsBlockPublisherTest extends TestCase {
  public void testSimple() {
    assertEquals(DavisApsBlockPublisher.channel("overview", DavisApsType.SIG), "davis240c.overview.sig");
    assertEquals(DavisApsBlockPublisher.channel("overview", DavisApsType.RST), "davis240c.overview.rst");
  }
}

// code by jph
package ch.ethz.idsc.demo.em;

import java.io.File;
import java.io.IOException;

import ch.ethz.idsc.demo.GokartLogFile;
import ch.ethz.idsc.gokart.calib.SensorsConfig;
import ch.ethz.idsc.gokart.calib.vmu931.PlanarVmu931Type;
import ch.ethz.idsc.gokart.offline.gui.GokartLcmLogCutter;
import ch.ethz.idsc.gokart.offline.gui.GokartLogFileIndexer;

/* package */ enum GokartLogCutter {
  ;
  public static void main(String[] args) throws IOException {
    SensorsConfig.GLOBAL.planarVmu931Type = PlanarVmu931Type.ROT90.name();
    GokartLogFile gokartLogFile = GokartLogFile._20190701T163225_12dcbfa8;
    File file = new File("C:\\Users\\Enrico\\Documents\\ethz binary file", gokartLogFile.getFilename());
    // file = new File("/media/datahaki/media/ethz/gokart/topic/racing2r", "20180820T143852_1.lcm");
    GokartLogFileIndexer gokartLogFileIndexer = GokartLogFileIndexer.create(file);
    new GokartLcmLogCutter( //
        gokartLogFileIndexer, //
        new File("C:\\Users\\Enrico\\Documents\\ethz binary file\\cutsfile"), //
        gokartLogFile.getTitle());
  }
}

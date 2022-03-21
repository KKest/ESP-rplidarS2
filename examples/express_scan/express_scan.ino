#include "rpLidar.h"
#include "rpLidarTypes.h"

rpLidar lidar(&Serial2,1000000);


void setup() {
  Serial.begin(115200);
  lidar.resetDevice(); //reset the device to be sure that the status is good
  lidar.setAngleOfInterest(5,175); //Set the field of view that is saved to Data
  lidar.start(express); //start the express scan of the lidar
}

void loop()
{
  lidar.readMeasurePoints();// reads a full scan and save it to Data
  for(int i=0;i<sizeof(lidar.Data)/sizeof(point_t);i++)
  {
    Serial.print(lidar.Data[i].angle);
    Serial.print("deg\t|\t");
    Serial.print(lidar.Data[i].distance);
    Serial.println("mm");
  }
}
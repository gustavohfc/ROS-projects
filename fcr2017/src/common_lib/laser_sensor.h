#ifndef LASER_SENSOR_H
#define LASER_SENSOR_H

enum laserSectionsKeys {laserFrontLeft, laserFrontRight};

// Configura secoes para a leitura da distancia pelo sensor laser
struct laserConfSections
{
  laserSectionsKeys key;
  double angle1, angle2;
  int step;
  laserConfSections(laserSectionsKeys _key, double _angle1, double _angle2, int _step) : key(_key), angle1(_angle1), angle2(_angle2), step(_step) {}
};


#endif

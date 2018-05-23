#include "controller.h"
#include "stabilizer_types.h"

#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define NUM_DATA_POINTS 10000

void init_setPoint(setpoint_t *setPoint) {
  setPoint->attitude.roll = 0;
  setPoint->attitude.pitch = 0;
  setPoint->attitude.yaw = 0;

  setPoint->attitudeRate.roll = 0;
  setPoint->attitudeRate.pitch = 0;
  setPoint->attitudeRate.yaw = 0;

  setPoint->attitudeQuaternion.x = 0;
  setPoint->attitudeQuaternion.y = 0;
  setPoint->attitudeQuaternion.z = 0;
  setPoint->attitudeQuaternion.w = 1.0;

  // the setpoint position is always at (0, 0, 0)
  setPoint->position.x = 0;
  setPoint->position.y = 0;
  setPoint->position.z = 0;

  setPoint->velocity.x = 0;
  setPoint->velocity.y = 0;
  setPoint->velocity.z = 0;

  setPoint->acceleration.x = 0;
  setPoint->acceleration.y = 0;
  setPoint->acceleration.z = 0;

  // assuming position control is used
  setPoint->mode.x = modeAbs;
  setPoint->mode.y = modeAbs;
  setPoint->mode.z = modeAbs;

  // assuming rate control is not used
  setPoint->mode.roll = modeAbs;
  setPoint->mode.pitch = modeAbs;
  setPoint->mode.yaw = modeAbs;
  setPoint->mode.quat = modeDisable;
}

float random_float(float lower, float upper) {
  int random_int = rand();
  float random_float = random_int/(float)RAND_MAX;  // between 0 and 1
  float range = upper - lower;
  return lower + range * random_float;
}

// the sqrt of each component of a unit quaternion sum to 1
void random_unit_quat(quaternion_t *quat) {
  float x = random_float(-50.0, 50.0);  // 50.0 is randomly chosen
  float y = random_float(-50.0, 50.0);  // it shouldn't have a big affect on the
  float z = random_float(-50.0, 50.0);  // distribution of the quaternion
  float w = random_float(-50.0, 50.0);

  // normalize
  float norm = sqrt(x*x+y*y+z*z+w*w);
  quat->x = x/norm;
  quat->y = y/norm;
  quat->z = z/norm;
  quat->w = w/norm;

  // from quaternion to rotational matrix
  float qYqY = quat->y * quat->y;
  float qXqX = quat->x * quat->x;
  float qZqZ = quat->z * quat->z;
  float qXqY = quat->x * quat->y;
  float qZqW = quat->z * quat->w;
  float qXqZ = quat->x * quat->z;
  float qYqW = quat->y * quat->w;
  float qYqZ = quat->y * quat->z;
  float qXqW = quat->x * quat->w;

  float rotM[9] = {1-2*qYqY-2*qZqZ, 2*qXqY-2*qZqW, 2*qXqZ+2*qYqW,
                   2*qXqY+2*qZqW, 1-2*qXqX-2*qZqZ, 2*qYqZ-2*qXqW,
                   2*qXqZ-2*qYqW, 2*qYqZ+2*qXqW, 1-2*qXqX-2*qYqY};
}

int main() {
  // set random number seed
  srand(time(NULL));

  control_t control;
  setpoint_t setPoint;
  sensorData_t sensorData;
  state_t state;

  init_setPoint(&setPoint);

  // randomly generate input/output pairs
  for(int i = 0; i < NUM_DATA_POINTS*2; i += 2) {
    // Angular velocity
    // unit: deg/s (limited to +-25 deg/s for now)
    sensorData.gyro.x = random_float(-25.0, 25.0);
    sensorData.gyro.y = random_float(-25.0, 25.0);
    sensorData.gyro.z = random_float(-25.0, 25.0);

    // Orientation
    random_unit_quat(&state.attitudeQuaternion);

    // Position
    // limit the state to 0.2 meters within the setPoint
    state.position.x = random_float(-0.2, 0.2);
    state.position.y = random_float(-0.2, 0.2);
    state.position.z = random_float(-0.2, 0.2);

    // Linear velocity
    // limit the velocity to 1 m/s
    state.velocity.x = random_float(-1.0, 1.0);
    state.velocity.y = random_float(-1.0, 1.0);
    state.velocity.z = random_float(-1.0, 1.0);

    stateController(&control, &setPoint, &sensorData, &state, i);
    printf("Angular vel: %3.2f %3.2f %3.2f; Position: %2.2f %2.2f %2.2f; Linear vel: %2.2f %2.2f %2.2f; Quat: %f %f %f %f\n",
                    sensorData.gyro.x, sensorData.gyro.y, sensorData.gyro.z,
                    state.position.x, state.position.y, state.position.z,
                    state.velocity.x, state.velocity.y, state.velocity.z,
                    state.attitudeQuaternion.x, state.attitudeQuaternion.y, state.attitudeQuaternion.z, state.attitudeQuaternion.w);
    printf("Controller outputs: %10d %10d %10d %10.8f\n", control.roll, control.pitch, control.yaw, control.thrust);
  }

  return 0;
}

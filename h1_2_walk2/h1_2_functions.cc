#include "mjpc/tasks/h1_2_walk2/h1_2_walk.h"

#include <iostream>
#include <algorithm>
#include <cmath>
#include <string>

#include <stdexcept>

#include <cstdio>

#include "mjpc/task.h"
#include "mjpc/utilities.h"
#include "mujoco/mujoco.h"

//#include "mjpc/tasks/h1_2_walk2/utility/dm_control_utils_rewards.h"

#include "mjpc/tasks/h1_2_walk2/h1_2_functions.h"

#include <vector>

// --- 1. Definizioni delle Costanti ---
const int NUM_ELEVATION_ANGLES = 6;
const int NUM_AZIMUTH_ANGLES = 5;

// Elevazione: 5, 10, 15, 20, 25, 30 gradi (Righe)
const double ELEVATION_DEGREES[NUM_ELEVATION_ANGLES] = {5.0, 10.0, 15.0, 20.0, 25.0, 30.0};
// Azimutale: -0.20, -0.10, 0.00, 0.10, 0.20 radianti (Colonne) 
//const double AZIMUTH_RADIANS[NUM_AZIMUTH_ANGLES] = {-0.20, -0.10, 0.00, 0.10, 0.20};

// Matrice per memorizzare gli ID dei sensori MuJoCo (Indice nell'array sensordata)
int sensor_indices[NUM_ELEVATION_ANGLES][NUM_AZIMUTH_ANGLES];

// Matrice per memorizzare gli ID dei SITI MuJoCo (necessari per trovare Posizione/Orientazione)
int site_indices[NUM_ELEVATION_ANGLES][NUM_AZIMUTH_ANGLES];


double sigmoid(double x, double value_at_1, std::string sigmoid_type) {
  if (sigmoid_type == "cosine" || sigmoid_type == "linear" ||
      sigmoid_type == "quadratic") {
    if (!(0 <= value_at_1 && value_at_1 <= 1)) {
      throw std::invalid_argument("Value at 1 must be in [0, 1].");
    }
  } else {
    if (!(0 < value_at_1 && value_at_1 < 1)) {
      throw std::invalid_argument("Value at 1 must be in (0, 1).");
    }
  }
  if (sigmoid_type == "gaussian") {
    double scale = std::sqrt(-2 * std::log(value_at_1));
    return std::exp(-0.5 * std::pow(x * scale, 2));
  } else if (sigmoid_type == "linear") {
    double scale = 1.0 - value_at_1;
    double scaled_x = x * scale;
    return std::abs(scaled_x) < 1.0 ? 1.0 - scaled_x : 0.0;
  } else if (sigmoid_type == "quadratic") {
    double scale = std::sqrt(1 - value_at_1);
    double scaled_x = x * scale;
    return std::abs(scaled_x) < 1.0 ? 1.0 - scaled_x * scaled_x : 0.0;
  } else {
    // in the python implementation there are some more sigmoid types, but they
    // are currently not used
    throw std::invalid_argument("Unknown sigmoid type.");
  }
}

double tolerance(double x, std::pair<double, double> bounds, double margin,
                 std::string sigmoid_type, double value_at_margin) {
  double lower = bounds.first;
  double upper = bounds.second;

  if (lower > upper) {
    throw std::invalid_argument("Lower bound must be <= than upper bound.");
  }
  if (margin < 0) {
    throw std::invalid_argument("Margin must be >= 0.");
  }
  bool in_bounds = lower <= x && x <= upper;
  double value;
  if (margin == 0) {
    value = in_bounds ? 1.0 : 0.0;
  } else {
    double distance =
        std::min(std::abs(x - lower), std::abs(x - upper)) / margin;
    if (in_bounds) {
      value = 1.0;
    } else {
      value = sigmoid(distance, value_at_margin, sigmoid_type);
    }
  }
  return value;
}


std::vector<std::vector<Point3D>> MapPointCloudMatrix(const mjModel* model, const mjData* data) {
    std::vector<std::vector<int>> sensor_indices(NUM_ELEVATION_ANGLES, std::vector<int>(NUM_AZIMUTH_ANGLES, -1));
    std::vector<std::vector<int>> site_indices(NUM_ELEVATION_ANGLES, std::vector<int>(NUM_AZIMUTH_ANGLES, -1));

    // Inizializzazione mapping sensori -> siti
    for (int i = 0; i < NUM_ELEVATION_ANGLES; ++i) {
        for (int j = 0; j < NUM_AZIMUTH_ANGLES; ++j) {
            int col_index = j + 1;
            int row_angle = (int)ELEVATION_DEGREES[i];

            char sensor_name[16];
            sprintf(sensor_name, "rf%d_%d", col_index, row_angle);

            int sensor_id = mj_name2id(model, mjOBJ_SENSOR, sensor_name);
            if (sensor_id >= 0) {
                sensor_indices[i][j] = model->sensor_adr[sensor_id];
                site_indices[i][j] = model->sensor_objid[sensor_id];
            }
        }
    }

    // Creiamo una matrice NUM_ELEVATION_ANGLES x NUM_AZIMUTH_ANGLES
    std::vector<std::vector<Point3D>> point_matrix(NUM_ELEVATION_ANGLES,
                                                   std::vector<Point3D>(NUM_AZIMUTH_ANGLES,
                                                                        {NAN, NAN, NAN})); // placeholder per punti non validi

    for (int i = 0; i < NUM_ELEVATION_ANGLES; ++i) {
        for (int j = 0; j < NUM_AZIMUTH_ANGLES; ++j) {
            int s = sensor_indices[i][j];
            int site = site_indices[i][j];
            if (s < 0 || site < 0) continue;

            double dist = data->sensordata[s];
            if (dist < 0) continue;

            const double* O = data->site_xpos + site*3;
            const double* R = data->site_xmat + site*9;

            double Zx = R[6], Zy = R[7], Zz = R[8];
            double Px = O[0] + dist * Zx;
            double Py = O[1] + dist * Zy;
            double Pz = O[2] + dist * Zz;

          //printf("Punto (%2d,%2d): rotation matrix: [%.3f, %.3f, %.3f;\n %.3f, %.3f, %.3f;\n %.3f, %.3f, %.3f]\n",                        (int)ELEVATION_DEGREES[i], (int)(AZIMUTH_RADIANS[j]*100),R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8] );
            if (Pz > 0.01) {
                point_matrix[i][j] = {Px, Py, Pz};

                // Azimutale: -0.20, -0.10, 0.00, 0.10, 0.20 radianti (Colonne)
                //const double AZIMUTH_RADIANS[NUM_AZIMUTH_ANGLES] = {-0.20, -0.10, 0.00, 0.10, 0.20};
                //printf("Punto (%2d,%2d): D=%.3f -> X=%.3f Y=%.3f Z=%.3f\n", (int)ELEVATION_DEGREES[i], (int)(AZIMUTH_RADIANS[j]*100), dist, Px, Py, Pz);
            }
        }
    }

    return point_matrix;
}
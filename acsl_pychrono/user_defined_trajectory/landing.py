import numpy as np

class LandingPolynomials:
  def __init__(self):
    self.data = {
      # Polynomial coefficients for LANDING from (X, Y, -1) to (X, Y, 0) in 4 seconds
      "1meterIn4seconds": {
        "meta": {
          "height_meters": 1.0,
          "duration_seconds": 4.0
        },
        "position": {"z": np.array([
          -0.001220703125,
          0.01708984375,
          -0.0820312499999999,
          0.13671875,
          0.0,
          0.0,
          0.0,
          -1.0])
        },
        "velocity": {"z": np.array([
          -0.008544921875,
          0.1025390625,
          -0.41015625,
          0.546875,
          0.0,
          0.0,
          0.0])
        },
        "acceleration": {"z": np.array([
          -0.05126953125,
          0.5126953125,
          -1.640625,
          1.640625,
          0.0,
          0.0])
        }
      }
      # Add other cases here
    }

    # Flip all polynomial coefficient arrays along axis 0 (reverse order)
    for traj in self.data.values():
      for key in ["position", "velocity", "acceleration"]:
        for axis_key, coeff_array in traj[key].items():
          traj[key][axis_key] = np.flip(coeff_array)

  def get(self, phase: str, type_: str, axis: str) -> np.ndarray:
    return self.data[phase][type_][axis]
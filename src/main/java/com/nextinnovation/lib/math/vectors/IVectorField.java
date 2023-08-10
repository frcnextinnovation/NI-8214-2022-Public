package com.nextinnovation.lib.math.vectors;

import com.nextinnovation.lib.geometry.Translation2d;

public interface IVectorField {
  Translation2d getVector(Translation2d here);
}

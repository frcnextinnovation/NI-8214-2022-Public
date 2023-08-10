package com.nextinnovation.lib.math.vectors;

import com.nextinnovation.lib.geometry.Translation2d;

import java.util.function.Function;

public interface ISurface {
  Function<Translation2d, Double> f();

  Function<Translation2d, Double> dfdx();

  Function<Translation2d, Double> dfdy();
}

package com.nextinnovation.lib.math.vectors;

import com.nextinnovation.lib.geometry.Translation2d;

import java.util.function.Function;

public abstract class BaseSurface implements ISurface {

  public abstract Function<Translation2d, Double> f();

  public abstract Function<Translation2d, Double> dfdx();

  public abstract Function<Translation2d, Double> dfdy();
}

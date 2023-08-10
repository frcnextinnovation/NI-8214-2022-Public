package com.nextinnovation.lib.geometry;

public interface ICurvature<S> extends IState<S> {
  double getCurvature();

  double getDCurvatureDs();
}

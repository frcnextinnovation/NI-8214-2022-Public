package com.nextinnovation.lib.trajectory;

import com.nextinnovation.lib.geometry.IState;

public interface ITrajectoryView<S extends IState<S>> {
  TrajectorySamplePoint<S> sample(final double interpolant);

  double first_interpolant();

  double last_interpolant();

  Trajectory<S> trajectory();
}

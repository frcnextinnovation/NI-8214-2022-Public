package com.nextinnovation.lib.trajectory;

import com.nextinnovation.lib.geometry.IState;

public class TrajectoryPoint<S extends IState<S>> {
  protected final S state_;
  protected final int index_;

  public TrajectoryPoint(final S state, int index) {
    state_ = state;
    index_ = index;
  }

  public S state() {
    return state_;
  }

  public int index() {
    return index_;
  }
}

package com.nextinnovation.lib.geometry;

import com.nextinnovation.lib.utils.IInterpolable;
import com.nextinnovation.lib.utils.ICSVWritable;

public interface IState<S> extends IInterpolable<S>, ICSVWritable {
  double distance(final S other);

  boolean equals(final Object other);

  String toString();

  String toCSV();
}

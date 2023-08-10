package com.nextinnovation.lib.auto.actions;

import java.util.ArrayList;
import java.util.List;

public class ParallelAction extends BaseAction {
  private final ArrayList<BaseAction> actions;

  /**
   * Composite action, running all sub-actions at the same time All actions are started then updated
   * until all actions report being done.
   *
   * @param new_actions List of BaseAction objects
   */
  public ParallelAction(List<BaseAction> new_actions) {
    actions = new ArrayList<>(new_actions.size());

    // Avoid to use addAll() for safe.
    for (BaseAction action : new_actions) {
      actions.add(action);
    }
  }

  @Override
  public boolean isFinished() {
    boolean all_finished = true;
    for (BaseAction action : actions) {
      if (!action.isFinished()) {
        all_finished = false;
      }
    }
    return all_finished;
  }

  @Override
  public void update() {
    for (BaseAction action : actions) {
      action.update();
    }
  }

  @Override
  public void done() {
    for (BaseAction action : actions) {
      action.done();
    }
  }

  @Override
  public void start() {
    for (BaseAction action : actions) {
      action.start();
    }
  }
}

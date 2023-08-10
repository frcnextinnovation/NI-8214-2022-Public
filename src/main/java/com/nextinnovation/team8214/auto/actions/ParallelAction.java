package com.nextinnovation.team8214.auto.actions;

import com.nextinnovation.lib.auto.actions.BaseAction;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Composite action, running all sub-actions at the same time All actions are started then updated
 * until all actions report being done.
 */
public class ParallelAction extends BaseAction {
  private final ArrayList<BaseAction> actions;

  public ParallelAction(List<BaseAction> actions) {
    this.actions = new ArrayList<>(actions);
  }

  public ParallelAction(BaseAction... actions) {
    this(Arrays.asList(actions));
  }

  @Override
  public void start() {
    actions.forEach(BaseAction::start);
  }

  @Override
  public void update() {
    actions.forEach(BaseAction::update);
  }

  @Override
  public boolean isFinished() {
    for (BaseAction action : actions) {
      if (!action.isFinished()) {
        return false;
      }
    }
    return true;
  }

  @Override
  public void done() {
    actions.forEach(BaseAction::done);
  }
}

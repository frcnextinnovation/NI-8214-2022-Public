package com.nextinnovation.lib.io;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.XboxController;

public class StatefulXboxController extends XboxController {
  public enum ButtonId {
    BUTTON_A(1),
    BUTTON_B(2),
    BUTTON_X(3),
    BUTTON_Y(4),
    BUMPER_LEFT(5),
    BUMPER_RIGHT(6),
    BUTTON_BACK(7),
    BUTTON_START(8),
    CENTER_CLICK_LEFT(9),
    CENTER_CLICK_RIGHT(10),
    TRIGGER_LEFT(-1),
    TRIGGER_RIGHT(-2),
    POV_0(-3),
    POV_90(-4),
    POV_180(-5),
    POV_270(-6);

    public final int id;

    private ButtonId(int id) {
      this.id = id;
    }
  }

  public class Button {
    private final ButtonId buttonId;
    private boolean buttonActive = false;
    private boolean activationReported = true;
    private boolean deactivationReported = true;

    private Button(ButtonId buttonId) {
      this.buttonId = buttonId;
    }

    private void update() {
      if (buttonPressed()) {
        if (!buttonActive) {
          buttonActive = true;
          activationReported = false;
        }
      } else {
        if (buttonActive) {
          buttonActive = false;
          deactivationReported = false;
        }
      }
    }

    private boolean buttonPressed() {
      boolean buttonPressed = false;
      if (buttonId.id > 0) {
        buttonPressed = getRawButton(buttonId.id);
      } else {
        switch (buttonId) {
          case TRIGGER_LEFT:
            {
              buttonPressed = getLeftTrigger() >= triggerPressThreshold;
              break;
            }
          case TRIGGER_RIGHT:
            {
              buttonPressed = getRightTrigger() >= triggerPressThreshold;
              break;
            }
          case POV_0:
            {
              buttonPressed = getPOV() == 0;
              break;
            }
          case POV_90:
            {
              buttonPressed = getPOV() == 90;
              break;
            }
          case POV_180:
            {
              buttonPressed = getPOV() == 180;
              break;
            }
          case POV_270:
            {
              buttonPressed = getPOV() == 270;
              break;
            }
          default:
            {
              break;
            }
        }
      }
      return buttonPressed;
    }

    public ButtonId getButtonId() {
      return buttonId;
    }

    public boolean isBeingPressed() {
      return buttonActive;
    }

    public boolean wasPressed() {
      if (!activationReported) {
        activationReported = true;
        return true;
      }
      return false;
    }

    public boolean wasReleased() {
      if (!deactivationReported) {
        deactivationReported = true;
        return true;
      }
      return false;
    }
  }

  private double triggerPressThreshold;
  private Map<ButtonId, Button> buttons = new HashMap<ButtonId, Button>();

  public StatefulXboxController(int port, double triggerPressThreshold) {
    super(port);
    this.triggerPressThreshold = triggerPressThreshold;
    for (ButtonId buttonId : ButtonId.values()) {
      buttons.put(buttonId, new Button(buttonId));
    }
  }

  public void updateButtons() {
    for (ButtonId buttonId : ButtonId.values()) {
      buttons.get(buttonId).update();
    }
  }

  public double getLeftTrigger() {
    return getLeftTriggerAxis();
  }

  public double getRightTrigger() {
    return getRightTriggerAxis();
  }

  public Button getButton(ButtonId buttonId) {
    return buttons.get(buttonId);
  }
}

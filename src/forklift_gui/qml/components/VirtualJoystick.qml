import QtQuick
import Forklift.Core
import Forklift.Gui

// Sanal joystick (fare / dokunmatik). Bırakınca merkeze döner. Değer birim
// diske kırpılır; hız ve rampa C++ TeleopController'da hesaplanır.
Item {
    id: joy

    // Normalize konum (görsel); y ileri pozitif.
    property real nx: 0
    property real ny: 0

    implicitWidth: Theme.joystickSize
    implicitHeight: Theme.joystickSize
    opacity: joy.enabled ? 1.0 : 0.4

    function moveTo(px: real, py: real) {
        const r = joy.width / 2;
        let x = (px - r) / r;
        let y = (r - py) / r;
        const len = Math.sqrt(x * x + y * y);
        if (len > 1) {
            x /= len;
            y /= len;
        }
        joy.nx = x;
        joy.ny = y;
        Backend.teleop.setJoystick(x, y);
    }

    function release() {
        joy.nx = 0;
        joy.ny = 0;
        Backend.teleop.releaseJoystick();
    }

    Rectangle {
        anchors.fill: parent
        radius: width / 2
        color: Theme.raised
        border.width: 1
        border.color: Theme.border
    }

    Rectangle {
        anchors.centerIn: parent
        width: parent.width - 2 * Theme.spacing
        height: 1
        color: Theme.border
    }

    Rectangle {
        anchors.centerIn: parent
        width: 1
        height: parent.height - 2 * Theme.spacing
        color: Theme.border
    }

    Rectangle {
        id: knob

        width: Theme.joystickKnob
        height: Theme.joystickKnob
        radius: width / 2
        x: joy.width / 2 + joy.nx * (joy.width - width) / 2 - width / 2
        y: joy.height / 2 - joy.ny * (joy.height - height) / 2 - height / 2
        color: area.pressed ? Theme.accent : Theme.raisedHover
        border.width: 2
        border.color: area.pressed ? Qt.lighter(Theme.accent, 1.3) : Theme.borderStrong
    }

    MouseArea {
        id: area

        anchors.fill: parent
        preventStealing: true
        onPressed: function (mouse) {
            joy.moveTo(mouse.x, mouse.y);
        }
        onPositionChanged: function (mouse) {
            if (area.pressed)
                joy.moveTo(mouse.x, mouse.y);
        }
        onReleased: joy.release()
        onCanceled: joy.release()
    }
}

import QtQuick
import Forklift.Gui

// Dikey çatal göstergesi: ölçülen yükseklik (dolgu), hedef (çizgi) ve
// ±tolerans bandı aynı ölçekte. Değerler metre; etiketler mm.
Item {
    id: gauge

    property real minimum: 0
    property real maximum: 1
    property real target: 0
    property bool hasTarget: false
    property real measured: 0
    property bool hasMeasured: false
    property real tolerance: 0
    property int ticks: 3

    implicitWidth: Theme.gaugeWidth
    implicitHeight: 200

    function toY(value: real): real {
        const range = gauge.maximum - gauge.minimum;
        const clamped = Math.max(gauge.minimum, Math.min(gauge.maximum, value));
        return track.y + track.height * (1 - (clamped - gauge.minimum) / range);
    }

    Rectangle {
        id: track

        x: gauge.width - width
        y: Theme.fontXs
        width: gauge.width * 0.45
        height: gauge.height - 2 * Theme.fontXs
        radius: Theme.radius
        color: Theme.raised
        border.width: 1
        border.color: Theme.border
    }

    // Ölçülen yükseklik dolgusu
    Rectangle {
        x: track.x + 2
        width: track.width - 4
        y: gauge.toY(gauge.measured)
        height: track.y + track.height - y
        radius: Theme.radius
        color: Theme.measured
        opacity: 0.85
        visible: gauge.hasMeasured
    }

    // ±tolerans bandı
    Rectangle {
        x: track.x - 4
        width: track.width + 8
        y: gauge.toY(gauge.target + gauge.tolerance)
        height: Math.max(2, gauge.toY(gauge.target - gauge.tolerance) - y)
        color: Theme.toleranceBand
        visible: gauge.hasTarget
    }

    // Hedef çizgisi
    Rectangle {
        x: track.x - 6
        width: track.width + 12
        y: gauge.toY(gauge.target) - 1
        height: 3
        color: Theme.target
        visible: gauge.hasTarget
    }

    // Ölçek etiketleri (mm). Konumlar modelde hesaplanır; delegate dış
    // id'lere erişmez (Qt 6.4 qmllint, ComponentBehavior yok).
    readonly property var tickModel: {
        const out = [];
        for (let i = 0; i <= gauge.ticks; ++i) {
            const v = gauge.minimum + (gauge.maximum - gauge.minimum) * i / gauge.ticks;
            out.push({
                "label": (v * 1000).toFixed(0),
                "y": gauge.toY(v),
                "width": track.x - 4
            });
        }
        return out;
    }

    Repeater {
        model: gauge.tickModel

        Text {
            required property var modelData

            x: 0
            width: modelData.width
            y: modelData.y - height / 2
            horizontalAlignment: Text.AlignRight
            text: modelData.label
            color: Theme.textFaint
            font.family: Theme.monoFamily
            font.pixelSize: Theme.fontXs
        }
    }
}

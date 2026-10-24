import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import Forklift.Core
import Forklift.Gui

// Çatal: gösterge (hedef + ölçülen + tolerans), dikey slider, sayısal giriş,
// preset'ler ve "hedefe ulaşıldı" durumu (spesifikasyon §4.2).
PanelFrame {
    id: panel

    // Sayısal giriş mm; yerel ayardan bağımsız olarak "," ya da "." kabul.
    function applyTypedTarget() {
        const mm = Number(input.text.replace(",", "."));
        if (input.text.length > 0 && !isNaN(mm))
            Backend.fork.setTarget(mm / 1000);
        input.text = "";
        input.focus = false;
    }

    title: qsTr("Çatal")
    headerExtras: [
        Text {
            text: !Backend.fork.enabled ? (Backend.autonomy.autoMode ? qsTr("devre dışı (OTONOM)") : qsTr("devre dışı (E-STOP)")) : Backend.fork.timedOut ? qsTr("UYARI") : Backend.fork.reached ? qsTr("● HEDEFE ULAŞILDI") : (Backend.fork.hasTarget ? qsTr("hareket ediyor") : "")
            color: !Backend.fork.enabled ? Theme.danger : Backend.fork.timedOut ? Theme.warn : Backend.fork.reached ? Theme.ok : Theme.info
            font.pixelSize: Theme.fontXs
            font.bold: true
        }
    ]

    RowLayout {
        anchors.fill: parent
        spacing: Theme.spacing

        ForkGauge {
            Layout.fillHeight: true
            Layout.preferredWidth: Theme.gaugeWidth
            minimum: Backend.fork.minHeight
            maximum: Backend.fork.maxHeight
            target: Backend.fork.target
            hasTarget: Backend.fork.hasTarget
            measured: Backend.fork.measured
            hasMeasured: Backend.fork.hasMeasured
            tolerance: Backend.fork.tolerance
        }

        Slider {
            Layout.fillHeight: true
            orientation: Qt.Vertical
            from: Backend.fork.minHeight
            to: Backend.fork.maxHeight
            stepSize: Backend.fork.step
            snapMode: Slider.SnapAlways
            enabled: Backend.fork.enabled
            focusPolicy: Qt.NoFocus
            value: Backend.fork.hasTarget ? Backend.fork.target : Backend.fork.measured
            // Sürüklerken her adımda değil, bırakınca tek hedef (yalnız değişimde yayın).
            onPressedChanged: {
                if (!pressed)
                    Backend.fork.setTarget(value);
            }
        }

        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: Theme.spacingSm

            // hedef / ölçülen / fark tek satırda (dar panelde dikey yer kalsın)
            RowLayout {
                Layout.fillWidth: true
                spacing: Theme.spacingLg

                ValueTile {
                    label: qsTr("hedef")
                    value: Backend.fork.hasTarget ? Theme.fixed(Backend.fork.target * 1000, 1).padStart(5, " ") : "—"
                    unit: "mm"
                    valueColor: Theme.target
                }

                ValueTile {
                    label: qsTr("ölçülen")
                    value: Backend.fork.hasMeasured ? Theme.fixed(Backend.fork.measured * 1000, 1).padStart(5, " ") : "—"
                    unit: "mm"
                    valueColor: Theme.measured
                }

                ValueTile {
                    label: qsTr("fark")
                    value: Backend.fork.hasTarget && Backend.fork.hasMeasured ? Theme.signedFixed((Backend.fork.measured - Backend.fork.target) * 1000, 1) : "—"
                    unit: "mm"
                    valueColor: Backend.fork.reached ? Theme.ok : Theme.text
                }
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: Theme.spacingSm

                TextField {
                    id: input

                    Layout.fillWidth: true
                    placeholderText: qsTr("hedef [mm]")
                    enabled: Backend.fork.enabled
                    validator: RegularExpressionValidator {
                        regularExpression: /^[0-9]{0,3}([.,][0-9]?)?$/
                    }
                    onAccepted: panel.applyTypedTarget()
                    Keys.onEscapePressed: {
                        input.text = "";
                        input.focus = false;
                    }
                }

                Button {
                    text: qsTr("Git")
                    enabled: Backend.fork.enabled && input.text.length > 0
                    focusPolicy: Qt.NoFocus
                    onClicked: panel.applyTypedTarget()
                }
            }

            GridLayout {
                Layout.fillWidth: true
                columns: 2
                columnSpacing: Theme.spacingSm
                rowSpacing: Theme.spacingSm

                Repeater {
                    model: Backend.fork.presetLabels

                    delegate: Button {
                        required property int index
                        required property string modelData

                        Layout.fillWidth: true
                        font.pixelSize: Theme.fontSm
                        // İki satır: dar panelde etiket kesilmesin.
                        text: "[" + (index + 1) + "] " + modelData + "\n" + (Backend.fork.presetHeights[index] * 1000).toFixed(0) + " mm"
                        enabled: Backend.fork.enabled
                        focusPolicy: Qt.NoFocus
                        onClicked: Backend.fork.applyPreset(index)
                    }
                }
            }

            Item {
                Layout.fillHeight: true
            }

            Text {
                Layout.fillWidth: true
                text: qsTr("R / F: ±%1 mm · 1–%2: preset").arg((Backend.fork.step * 1000).toFixed(0)).arg(Backend.fork.presetLabels.length)
                color: Theme.textFaint
                font.pixelSize: Theme.fontXs
                elide: Text.ElideRight
            }
        }
    }
}

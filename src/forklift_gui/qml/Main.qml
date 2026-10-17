import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import QtQuick.Templates as T
import Forklift.Core
import Forklift.Gui

// Ana pencere: TopBar + iç içe SplitView'ler (ARCHITECTURE §7.10).
// Klavye kısayolları C++'ta (KeyRouter → Backend.handleKey) işlenir.
ApplicationWindow {
    id: window

    width: 1280
    height: 800
    minimumWidth: 1280
    minimumHeight: 800
    visible: true
    color: Theme.background
    title: Backend.mockMode ? qsTr("Forklift Operatör Paneli — MOCK") : qsTr("Forklift Operatör Paneli")

    // Basic stil kontrolleri için koyu palet (renkler Theme'den).
    palette.window: Theme.panel
    palette.windowText: Theme.text
    palette.base: Theme.raised
    palette.alternateBase: Theme.panelHeader
    palette.text: Theme.text
    palette.button: Theme.raised
    palette.buttonText: Theme.text
    palette.brightText: Theme.textOnDanger
    palette.highlight: Theme.accent
    palette.highlightedText: Theme.textOnDanger
    palette.light: Theme.raisedHover
    palette.midlight: Theme.raisedHover
    palette.mid: Theme.pressed
    palette.dark: Theme.borderStrong
    palette.shadow: Theme.background
    palette.placeholderText: Theme.textFaint
    palette.toolTipBase: Theme.panelHeader
    palette.toolTipText: Theme.text

    Component {
        id: splitHandle

        Rectangle {
            implicitWidth: Theme.splitHandle
            implicitHeight: Theme.splitHandle
            color: T.SplitHandle.pressed ? Theme.accent : (T.SplitHandle.hovered ? Theme.borderStrong : Theme.background)
        }
    }

    // Odak toplayıcı: boş alana tıklanınca metin alanı odağı bırakır ve
    // klavye kısayolları (W/A/S/D, R/F, 1–4, C) yeniden çalışır.
    Item {
        id: focusSink

        focus: true
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        TapHandler {
            onTapped: focusSink.forceActiveFocus()
        }

        TopBar {
            Layout.fillWidth: true
            Layout.preferredHeight: Theme.topBarHeight
        }

        SplitView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.margins: Theme.estopFrameWidth
            orientation: Qt.Vertical
            handle: splitHandle

            SplitView {
                SplitView.fillHeight: true
                orientation: Qt.Horizontal
                handle: splitHandle

                CameraPanel {
                    SplitView.fillWidth: true
                    SplitView.minimumWidth: 480
                }

                SplitView {
                    SplitView.preferredWidth: 400
                    SplitView.minimumWidth: 340
                    orientation: Qt.Vertical
                    handle: splitHandle

                    TelemetryPanel {
                        SplitView.fillHeight: true
                        SplitView.minimumHeight: 200
                    }

                    ForkPanel {
                        SplitView.preferredHeight: 255
                        SplitView.minimumHeight: 235
                    }
                }
            }

            SplitView {
                SplitView.preferredHeight: 215
                SplitView.minimumHeight: 190
                orientation: Qt.Horizontal
                handle: splitHandle

                DrivePanel {
                    SplitView.preferredWidth: 400
                    SplitView.minimumWidth: 340
                }

                BottomTabs {
                    SplitView.fillWidth: true
                    SplitView.minimumWidth: 400
                }
            }
        }
    }

    // E-stop etkinken tüm arayüzü saran kırmızı çerçeve; tıklamaları geçirir.
    Rectangle {
        anchors.fill: parent
        z: 1000
        color: "transparent"
        border.width: Theme.estopFrameWidth
        border.color: Theme.danger
        visible: Backend.safety.engaged
    }

    HelpOverlay {
        anchors.fill: parent
        z: 2000
        visible: Backend.helpVisible
    }
}

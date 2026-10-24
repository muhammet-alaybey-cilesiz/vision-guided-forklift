import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import Forklift.Core
import Forklift.Gui

// Otonomi: Manuel / Otonom mod ve görev düğmeleri (D-57). Mod görev yöneticisinde tutulur;
// panel istekte bulunur, durumu gösterir. Otonom modda çatal paneli pasif; sürüş girdisi
// (WASD / joystick) Manuel moda geçirir.
PanelFrame {
    id: panel

    readonly property var autonomy: Backend.autonomy

    title: qsTr("Otonomi")
    headerExtras: [
        Text {
            text: !panel.autonomy.online ? qsTr("çevrimdışı") : panel.autonomy.autoMode ? qsTr("● OTONOM") : qsTr("MANUEL")
            color: !panel.autonomy.online ? Theme.textFaint : panel.autonomy.autoMode ? Theme.ok : Theme.textDim
            font.pixelSize: Theme.fontXs
            font.bold: true
        }
    ]

    ColumnLayout {
        anchors.fill: parent
        spacing: Theme.spacingSm

        RowLayout {
            Layout.fillWidth: true
            spacing: Theme.spacingSm

            Button {
                Layout.fillWidth: true
                font.pixelSize: Theme.fontSm
                text: qsTr("Manuel")
                highlighted: panel.autonomy.online && !panel.autonomy.autoMode
                enabled: panel.autonomy.online && panel.autonomy.autoMode && !panel.autonomy.busy
                focusPolicy: Qt.NoFocus
                onClicked: panel.autonomy.setManual()
            }

            Button {
                Layout.fillWidth: true
                font.pixelSize: Theme.fontSm
                text: qsTr("Otonom")
                highlighted: panel.autonomy.autoMode
                enabled: panel.autonomy.canSetAuto && !panel.autonomy.autoMode
                focusPolicy: Qt.NoFocus
                onClicked: panel.autonomy.setAuto()
            }
        }

        GridLayout {
            Layout.fillWidth: true
            columns: 4
            columnSpacing: Theme.spacingSm
            rowSpacing: Theme.spacingSm

            Button {
                Layout.fillWidth: true
                font.pixelSize: Theme.fontSm
                text: qsTr("Görevi başlat")
                enabled: panel.autonomy.canStart
                focusPolicy: Qt.NoFocus
                onClicked: panel.autonomy.start()
            }

            Button {
                Layout.fillWidth: true
                font.pixelSize: Theme.fontSm
                text: qsTr("Duraklat")
                enabled: panel.autonomy.canPause
                focusPolicy: Qt.NoFocus
                onClicked: panel.autonomy.pause()
            }

            Button {
                Layout.fillWidth: true
                font.pixelSize: Theme.fontSm
                text: qsTr("Devam")
                enabled: panel.autonomy.canResume
                focusPolicy: Qt.NoFocus
                onClicked: panel.autonomy.resume()
            }

            Button {
                Layout.fillWidth: true
                font.pixelSize: Theme.fontSm
                text: qsTr("İptal")
                enabled: panel.autonomy.canCancel
                focusPolicy: Qt.NoFocus
                onClicked: panel.autonomy.cancel()
            }
        }

        Text {
            Layout.fillWidth: true
            text: panel.autonomy.online ? (panel.autonomy.state + " · " + panel.autonomy.detail) : qsTr("Görev yöneticisi çalışmıyor")
            color: panel.autonomy.state === "FAULT" ? Theme.danger : panel.autonomy.state === "PAUSED" ? Theme.warn : Theme.text
            font.family: Theme.monoFamily
            font.pixelSize: Theme.fontSm
            elide: Text.ElideRight
        }

        Text {
            Layout.fillWidth: true
            visible: panel.autonomy.online
            text: qsTr("Hedef mühür: %1 · Koli: %2 / %3").arg(panel.autonomy.targetColor.length > 0 ? panel.autonomy.targetColor : "—").arg(panel.autonomy.parcelsDone).arg(panel.autonomy.parcelsTotal) + (panel.autonomy.lastMessage.length > 0 ? " · " + panel.autonomy.lastMessage : "")
            color: Theme.textDim
            font.pixelSize: Theme.fontXs
            elide: Text.ElideRight
        }

        Item {
            Layout.fillHeight: true
        }
    }
}

import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import Forklift.Core
import Forklift.Gui

// Veri seti kaydı (spesifikasyon §4.5): sürekli kayıt aç/kapa, anlık kayıt
// [C], sayaç, oturum dizini ve son hata. Disk işi C++'ta ayrı iş parçacığında.
PanelFrame {
    id: panel

    title: qsTr("Veri seti")

    ColumnLayout {
        anchors.fill: parent
        spacing: Theme.spacing

        RowLayout {
            Layout.fillWidth: true
            spacing: Theme.spacing

            // Sürekli kayıt düğmesi solda: konumu düzen değişse de sabit.
            Button {
                text: Backend.dataset.recording ? qsTr("■ Sürekli kaydı durdur") : qsTr("● Sürekli kayıt (%1 Hz)").arg(Backend.dataset.continuousRate)
                focusPolicy: Qt.NoFocus
                palette.button: Backend.dataset.recording ? Theme.dangerDark : Theme.raised
                onClicked: Backend.dataset.toggleRecording()
            }

            Button {
                text: qsTr("Kare kaydet  [C]")
                focusPolicy: Qt.NoFocus
                onClicked: Backend.dataset.capture()
            }

            Item {
                Layout.fillWidth: true
            }

            Text {
                text: qsTr("kayıt %1").arg(String(Backend.dataset.captureCount).padStart(5, " "))
                color: Theme.text
                font.family: Theme.monoFamily
                font.pixelSize: Theme.fontMd
            }

            Text {
                text: qsTr("yazılıyor %1").arg(Backend.dataset.pendingWrites)
                color: Backend.dataset.pendingWrites > 0 ? Theme.info : Theme.textFaint
                font.family: Theme.monoFamily
                font.pixelSize: Theme.fontSm
            }
        }

        GridLayout {
            Layout.fillWidth: true
            columns: 2
            columnSpacing: Theme.spacingLg
            rowSpacing: Theme.spacingXs

            Text {
                text: qsTr("kök")
                color: Theme.textDim
                font.pixelSize: Theme.fontSm
            }

            Text {
                Layout.fillWidth: true
                text: Backend.dataset.rootDir
                color: Theme.text
                font.family: Theme.monoFamily
                font.pixelSize: Theme.fontSm
                elide: Text.ElideMiddle
            }

            Text {
                text: qsTr("oturum")
                color: Theme.textDim
                font.pixelSize: Theme.fontSm
            }

            Text {
                Layout.fillWidth: true
                text: Backend.dataset.sessionDir.length > 0 ? Backend.dataset.sessionDir : qsTr("— (ilk kayıtta oluşur)")
                color: Theme.text
                font.family: Theme.monoFamily
                font.pixelSize: Theme.fontSm
                elide: Text.ElideMiddle
            }

            Text {
                text: qsTr("son hata")
                color: Theme.textDim
                font.pixelSize: Theme.fontSm
            }

            Text {
                Layout.fillWidth: true
                text: Backend.dataset.lastError.length > 0 ? Backend.dataset.lastError : "—"
                color: Backend.dataset.lastError.length > 0 ? Theme.warn : Theme.textFaint
                font.pixelSize: Theme.fontSm
                elide: Text.ElideRight
            }
        }

        Item {
            Layout.fillHeight: true
        }

        Text {
            Layout.fillWidth: true
            text: qsTr("Her kayıt: tüm kameraların son karesi (PNG) ve metadata.jsonl satırı (sim/duvar zamanı, poz, çatal).")
            color: Theme.textFaint
            font.pixelSize: Theme.fontXs
            wrapMode: Text.WordWrap
        }
    }
}

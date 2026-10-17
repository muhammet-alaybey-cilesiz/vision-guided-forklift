import QtQuick
import Forklift.Gui

// Etiket + sabit genişlikli değer + birim; sayılar değişirken titremez.
Column {
    id: tile

    property string label
    property string value
    property string unit
    property color valueColor: Theme.text

    spacing: 0

    Text {
        text: tile.label
        color: Theme.textDim
        font.pixelSize: Theme.fontXs
    }

    Row {
        spacing: Theme.spacingSm

        Text {
            id: valueText

            text: tile.value
            color: tile.valueColor
            font.family: Theme.monoFamily
            font.pixelSize: Theme.fontLg
        }

        Text {
            anchors.baseline: valueText.baseline
            text: tile.unit
            color: Theme.textDim
            font.pixelSize: Theme.fontXs
        }
    }
}

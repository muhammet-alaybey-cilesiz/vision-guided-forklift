import QtQuick
import Forklift.Core
import Forklift.Gui

// F1 yardım katmanı: tüm kısayollar. Tıklama, Esc ya da F1 kapatır.
Rectangle {
    id: overlay

    readonly property var shortcuts: [
        {
            "key": "Space",
            "text": qsTr("Acil durdurma (yalnız etkinleştirir, sıfırlamaz)")
        },
        {
            "key": "W / S",
            "text": qsTr("İleri / geri — basılı tut")
        },
        {
            "key": "A / D",
            "text": qsTr("Sola / sağa dön — basılı tut")
        },
        {
            "key": "R / F",
            "text": qsTr("Çatal hedefi +5 mm / −5 mm")
        },
        {
            "key": "1 – 4",
            "text": qsTr("Çatal preset'leri (Yer, Temas, Taşıma, Maks)")
        },
        {
            "key": "C",
            "text": qsTr("Veri seti: iki kamera karesini kaydet")
        },
        {
            "key": "F1 / Esc",
            "text": qsTr("Yardımı aç / kapat")
        },
        {
            "key": qsTr("E-STOP"),
            "text": qsTr("Serbest bırakmak için butonu basılı tut")
        }
    ]

    color: Theme.scrim

    MouseArea {
        anchors.fill: parent
        onClicked: Backend.helpVisible = false
    }

    Rectangle {
        anchors.centerIn: parent
        width: 560
        height: column.implicitHeight + 4 * Theme.margin
        radius: Theme.radius * 2
        color: Theme.panel
        border.width: 1
        border.color: Theme.borderStrong

        Column {
            id: column

            anchors.fill: parent
            anchors.margins: 2 * Theme.margin
            spacing: Theme.spacing

            Text {
                text: qsTr("Klavye kısayolları")
                color: Theme.text
                font.pixelSize: Theme.fontXl
                font.bold: true
            }

            Repeater {
                model: overlay.shortcuts

                delegate: Row {
                    id: row

                    required property var modelData

                    spacing: Theme.spacingLg

                    Text {
                        width: 110
                        text: row.modelData.key
                        color: Theme.accent
                        font.family: Theme.monoFamily
                        font.pixelSize: Theme.fontMd
                        font.bold: true
                    }

                    Text {
                        text: row.modelData.text
                        color: Theme.text
                        font.pixelSize: Theme.fontMd
                    }
                }
            }

            Text {
                topPadding: Theme.spacing
                text: qsTr("Sürüş tuşları bırakılınca ya da pencere odağı kaybedilince robot durur.")
                color: Theme.textDim
                font.pixelSize: Theme.fontSm
            }
        }
    }
}

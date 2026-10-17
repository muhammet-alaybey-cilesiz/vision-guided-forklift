// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef APP__KEYROUTER_HPP_
#define APP__KEYROUTER_HPP_

#include <QObject>
#include <QPointer>
#include <QQuickWindow>

#include "forklift_gui/core/Backend.hpp"

namespace forklift_gui
{

// Pencere düzeyinde klavye süzgeci: tuş olayları odaktaki öğeye ulaşmadan
// Backend::handleKey'e gider (eşleme C++ core'da). Kurallar:
//  * Space her zaman e-stop'tur; metin alanı odaktayken bile.
//  * Metin girişi odaktaysa diğer kısayollar alana bırakılır; sürüş tuşlarının
//    BIRAKMA olayları yine iletilir ki takılı tuş kalmasın.
class KeyRouter : public QObject
{
  Q_OBJECT

public:
  KeyRouter(QQuickWindow * window, Backend * backend, QObject * parent = nullptr);

protected:
  bool eventFilter(QObject * watched, QEvent * event) override;

private:
  bool textInputFocused() const;

  QPointer<QQuickWindow> window_;
  Backend * backend_;
};

}  // namespace forklift_gui

#endif  // APP__KEYROUTER_HPP_

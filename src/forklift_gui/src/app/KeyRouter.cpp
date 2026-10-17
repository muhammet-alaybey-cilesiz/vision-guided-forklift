// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#include "KeyRouter.hpp"

#include <QKeyEvent>
#include <QQuickItem>

namespace forklift_gui
{

namespace
{

bool isDriveKey(int key)
{
  return key == Qt::Key_W || key == Qt::Key_A || key == Qt::Key_S || key == Qt::Key_D;
}

}  // namespace

KeyRouter::KeyRouter(QQuickWindow * window, Backend * backend, QObject * parent)
: QObject(parent), window_(window), backend_(backend)
{
  window->installEventFilter(this);
}

bool KeyRouter::eventFilter(QObject * watched, QEvent * event)
{
  if (event->type() != QEvent::KeyPress && event->type() != QEvent::KeyRelease) {
    return QObject::eventFilter(watched, event);
  }
  const auto * keyEvent = static_cast<QKeyEvent *>(event);
  const bool pressed = event->type() == QEvent::KeyPress;
  const int key = keyEvent->key();

  if (key == Qt::Key_Space) {
    // E-stop her koşulda; odaktaki düğme ya da alan Space'i yutmasın.
    backend_->handleKey(key, pressed, keyEvent->isAutoRepeat());
    return true;
  }
  if (textInputFocused()) {
    if (!pressed && isDriveKey(key)) {
      backend_->handleKey(key, false, keyEvent->isAutoRepeat());
    }
    return false;  // yazı alana gitsin
  }
  if (backend_->handleKey(key, pressed, keyEvent->isAutoRepeat())) {
    return true;
  }
  return QObject::eventFilter(watched, event);
}

bool KeyRouter::textInputFocused() const
{
  const QQuickItem * item = window_ ? window_->activeFocusItem() : nullptr;
  // TextInput / TextEdit bu bayrağı taşır.
  return item && item->flags().testFlag(QQuickItem::ItemAcceptsInputMethod);
}

}  // namespace forklift_gui

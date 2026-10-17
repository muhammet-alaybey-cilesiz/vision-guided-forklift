// Copyright 2026 muhammet61
// SPDX-License-Identifier: Apache-2.0

#ifndef FORKLIFT_SAFETY__GATELOGIC_HPP_
#define FORKLIFT_SAFETY__GATELOGIC_HPP_

#include <cstdint>

namespace forklift_safety
{

// cmd_vel_gate'in ROS'tan bağımsız durum makinesi (DECISIONS D-24).
// Zamanlar monoton saat nanosaniyesidir (duvar saati, D-25).
struct GateConfig
{
  double inputTimeout{0.0};  // [s] bu kadar girdi gelmezse sıfır
  double eStopTimeout{0.0};  // [s] /e_stop bu kadar gelmezse kilitli say
  double zeroRate{0.0};      // [Hz] dururken sıfırın tekrar hızı
};

class GateLogic
{
public:
  explicit GateLogic(const GateConfig & config);

  // Girdi (twist_mux çıkışı) geldi; iletilecekse true.
  bool onInput(int64_t nowNs);

  // /e_stop mesajı; yükselen kenarda hemen sıfır gerekiyorsa true.
  bool onEStop(bool engaged, int64_t nowNs);

  // Periyodik denetim; şimdi sıfır yayınlanmalıysa true (zero_rate ile kısılı).
  bool onTimer(int64_t nowNs);

  // Sıfır yayınlandı (onEStop ya da onTimer sonrası çağrılır).
  void markZeroPublished(int64_t nowNs) { lastZeroNs_ = nowNs; }

  // E-stop etkin, hiç gelmedi ya da eskidi: girdi iletilmez.
  bool blocked(int64_t nowNs) const;

  // Kilitli ya da girdi sessiz: robot sıfırla tutulur.
  bool stopped(int64_t nowNs) const;

private:
  GateConfig config_;
  bool eStop_{true};  // bilinmiyorsa kilitli kabul edilir
  int64_t lastEStopNs_{-1};
  int64_t lastInputNs_{-1};
  int64_t lastZeroNs_{-1};
};

}  // namespace forklift_safety

#endif  // FORKLIFT_SAFETY__GATELOGIC_HPP_

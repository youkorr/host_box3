#pragma once
#include "esphome.h"
#include "esphome/components/media_player/media_player.h"
#include "esphome/components/speaker/speaker.h"

#ifdef CONFIG_ESP32_S3_USB_OTG

namespace esphome {
namespace host_box3 {

// Enumérations
enum AudioPlayerType {
  AUDIO_PLAYER_I2S = 0,
  AUDIO_PLAYER_USB
};

enum AudioOutputMode {
  USB_HEADSET = 0,
  USB_SPEAKERS,
  USB_AUTO
};

enum MuteSetting {
  UNMUTED = 1,
  MUTED = 0
};

// Structure de contexte
struct AudioPlayerContext {
  bool is_playing;
  bool is_muted;
  float current_volume;
};

class USBAudioComponent : public Component,
                         public media_player::MediaPlayer,
                         public Parented<speaker::Speaker> {
 public:
  // Configuration
  void set_output_mode(AudioOutputMode mode) { output_mode_ = mode; }
  void set_priority(uint8_t priority) { priority_ = priority; }

  // Héritage Component
  void setup() override;
  void loop() override;
  void dump_config() override;

  // Héritage MediaPlayer
  void control(const media_player::MediaPlayerCall &call) override;
  media_player::MediaPlayerTraits get_traits() override;

  // Fonctions audio
  void play();
  void pause();
  void stop();
  void set_volume(float volume);
  void mute(bool mute);

  // Gestion USB
  bool is_usb_connected() const { return usb_connected_; }

 protected:
  // Variables d'état
  AudioOutputMode output_mode_{USB_AUTO};
  uint8_t priority_{50};
  bool usb_connected_{false};
  float current_volume_{0.8f};
  AudioPlayerContext context_;

  // Handles USB
  void* usb_handle_{nullptr};
  
  // Callbacks
  static void usb_event_callback(void* arg, esp_event_base_t event_base, 
                               int32_t event_id, void* event_data);
};


}  // namespace host_box3
}  // namespace esphome

#endif  // HOST_BOX3_H












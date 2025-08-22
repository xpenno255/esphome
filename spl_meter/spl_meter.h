#include "esphome.h"
#include <cmath>
extern "C" {
  #include "driver/i2s.h"
}

class I2SSPLMeter : public PollingComponent, public Sensor {
 public:
  I2SSPLMeter(gpio_num_t bclk, gpio_num_t lrck, gpio_num_t dout,
              int sample_rate_hz, int window_ms, float calibration_offset_db)
    : bclk_(bclk), lrck_(lrck), dout_(dout),
      fs_(sample_rate_hz), win_ms_(window_ms), cal_db_(calibration_offset_db) {}

  void setup() override {
    i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = fs_,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S),
      .intr_alloc_flags = 0,
      .dma_buf_count = 6,
      .dma_buf_len = 256,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
      .bck_io_num = bclk_,
      .ws_io_num = lrck_,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = dout_
    };

    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &pin_config));
    ESP_ERROR_CHECK(i2s_set_clk(I2S_NUM_0, fs_, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO));
  }

  void update() override {
    const uint32_t N = (uint32_t)((fs_ * win_ms_) / 1000);
    if (N < 64) return;

    buf_.resize(N);
    size_t br = 0;
    if (i2s_read(I2S_NUM_0, buf_.data(), N * sizeof(int32_t), &br, portMAX_DELAY) != ESP_OK ||
        br < sizeof(int32_t)) return;
    const uint32_t samples = br / sizeof(int32_t);

    double mean = 0.0;
    floatv_.clear();
    floatv_.reserve(samples);
    for (uint32_t i = 0; i < samples; i++) {
      int32_t s = buf_[i] >> 8;                 // top 24 bits
      if (s & 0x00800000) s |= 0xFF000000;      // sign-extend
      float f = (float)s / 8388608.0f;          // 2^23
      floatv_.push_back(f);
      mean += f;
    }
    mean /= (double)samples;

    double sumsq = 0.0;
    for (float f : floatv_) {
      const double x = (double)f - mean;
      sumsq += x * x;
    }
    double rms = sqrt(sumsq / (double)samples);

    // Convert to dBFS relative to full-scale sine RMS
    const double fs_rms = 1.0 / 1.41421356237;
    double dbfs = -120.0;
    if (rms > 1e-12) {
      dbfs = 20.0 * log10(rms / fs_rms);
      if (dbfs < -120.0) dbfs = -120.0;
      if (dbfs >   6.0)  dbfs =   6.0;
    }

    const double db_spl = dbfs + cal_db_;
    publish_state((float)db_spl);
  }

 private:
  gpio_num_t bclk_, lrck_, dout_;
  int fs_, win_ms_;
  float cal_db_;
  std::vector<int32_t> buf_;
  std::vector<float> floatv_;
};

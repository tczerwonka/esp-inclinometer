#pragma once

#define LGFX_USE_V1

#include <LovyanGFX.hpp>

// GC9A01
//cfg.pin_sclk = 10;
//cfg.pin_mosi = 11;
//cfg.pin_miso = -1;
//cfg.pin_dc = 3; 
//cfg.pin_cs = 9;
//cfg.pin_rst = 14;
//backlight 46

// encoder
// A pin 45
// B pin 42
// switch 41

//power light 40


class LGFX : public lgfx::LGFX_Device
{
    lgfx::Panel_GC9A01 _panel;
    lgfx::Bus_SPI _bus;

public:

    LGFX(void)
    {
        {
            auto cfg = _bus.config();

            cfg.spi_host = SPI2_HOST;

            cfg.spi_mode = 0;

            cfg.freq_write = 80000000;
            cfg.freq_read  = 16000000;
            cfg.freq_write = 40000000;
            cfg.freq_write = 30000000;
            cfg.freq_read  = 8000000;

            cfg.spi_3wire = false;

            cfg.use_lock = true;

            cfg.dma_channel = SPI_DMA_CH_AUTO;

            //cfg.pin_sclk = 13;
            //cfg.pin_mosi = 12;
            //cfg.pin_miso = -1; // not used

            //cfg.pin_dc = 10;

cfg.pin_sclk = 10;
cfg.pin_mosi = 11;
cfg.pin_miso = -1;
cfg.pin_dc = 3;

            _bus.config(cfg);

            _panel.setBus(&_bus);
        }

        {
            auto cfg = _panel.config();
cfg.pin_cs = 9;
cfg.pin_rst = 14;
cfg.pin_busy = -1;
            //cfg.pin_cs = 9;
            //cfg.pin_rst = 11;
            //cfg.pin_busy = -1;

            cfg.panel_width = 240;
            cfg.panel_height = 240;

            cfg.offset_x = 0;
            cfg.offset_y = 0;

            cfg.offset_rotation = 0;

            cfg.dummy_read_pixel = 8;
            cfg.dummy_read_bits = 1;

            cfg.readable = false;
            cfg.invert = true;

            cfg.rgb_order = false;

            cfg.dlen_16bit = false;

            cfg.bus_shared = false;

            _panel.config(cfg);
        }

        setPanel(&_panel);
    }
};
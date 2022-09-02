#pragma once

#ifndef menu_h
#define menu_h
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

struct MenuItem {
    uint8_t id;
    uint8_t parent_id;
    const char* caption;
};

class Menu {
    private:
        MenuItem _items[50];
        uint8_t _item_cntr {0};             // item counter. point on next free cell
        uint8_t _active_item_id {0};        // current active item
        uint8_t _focused_item_id {0};       // cu

    public:
        void add(uint8_t id, uint8_t parent_id, const char* caption);
        uint8_t enter();
        uint8_t exit();
        uint8_t next();
        uint8_t prev();
        uint8_t jump(uint8_t id);
        MenuItem getActiveItem();
        MenuItem getFocusedItem();
        void draw(LiquidCrystal_I2C& screen);
        void draw(LiquidCrystal_I2C& screen, char* user_str);
};

#endif
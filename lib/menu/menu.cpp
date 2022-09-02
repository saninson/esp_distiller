
#include "menu.h"

void Menu::add(uint8_t id, uint8_t parent_id, const char* caption){
    _items[_item_cntr++] = {id, parent_id, caption};
}

MenuItem Menu::getActiveItem(){
    return _items[_active_item_id];
}

MenuItem Menu::getFocusedItem(){
    return _items[_focused_item_id];
}

uint8_t Menu::enter(){
    _active_item_id = _focused_item_id;
    return _active_item_id;
}

uint8_t Menu::exit(){
    _active_item_id = _items[_active_item_id].parent_id;
    return _active_item_id;
}

uint8_t Menu::next(){
    uint8_t next_id = _item_cntr + 1;
    for(uint8_t n=0; n<_item_cntr; n++){
        if(_items[n].parent_id == _items[_focused_item_id].parent_id){  // check only same parent items
            if(_items[n].id > _focused_item_id && _items[n].id < next_id)
                next_id == _items[n].id;
        }
    }
    if(next_id != _item_cntr+1)
        _focused_item_id = next_id;
    return _focused_item_id;
}

uint8_t Menu::prev(){
    uint8_t prev_id = 0;
    for(uint8_t n=0; n<_item_cntr; n++){
        if(_items[n].parent_id == _items[_focused_item_id].parent_id){  // check only same parent items
            if(_items[n].id < _focused_item_id && _items[n].id > prev_id)
                prev_id == _items[n].id;
        }
    }
    if(prev_id != 0)
        _focused_item_id = prev_id;
    return _focused_item_id;
}

uint8_t Menu::jump(uint8_t id){
    _active_item_id = id;
    return _active_item_id;
}


void Menu::draw(LiquidCrystal_I2C& screen){
    screen.clear();
    screen.setCursor(0,0);
    screen.print(_items[_focused_item_id].caption);
    screen.setCursor(0,1);
}

void Menu::draw(LiquidCrystal_I2C& screen, const char* user_str){
    draw(screen);
    uint8_t left = 16 - screen.print(user_str);
    for ()
}
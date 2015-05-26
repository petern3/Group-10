

#include "display_core.h"

void init_display_core(void) {
  Serial.begin(BAUD_RATE);
}


void display_map(void) {
  int8_t index_x = 0;
  int8_t index_y = 0;
  Position_t curr = {0, 0};
  int8_t terrain_to_print = 0;
  char num[3] = {"\0"};

  for (index_y = -1; index_y <= NUM_NODES_Y; index_y++) {
    for (index_x = -1; index_x <= NUM_NODES_X; index_x++) {
      curr.x = index_x;
      curr.y = index_y;
      terrain_to_print = get_terrain(curr);
      sprintf(num, "%2d", terrain_to_print);
      
      #ifdef DEBUG_MAP_NUMBERS
        Serial.print(num);
      #else
        if (terrain_to_print == -1) {
          Serial.print(NODE_EDGE);
        } else if (terrain_to_print == 0) {
          Serial.print(NODE_OPEN);
        } else if (terrain_to_print == 1) {
          Serial.print(NODE_WALL);
        } else if (terrain_to_print == 2) {
          Serial.print(NODE_BASE);
        } else if (terrain_to_print == 3) {
          Serial.print(NODE_PACK);
        } else {
          Serial.print(num);
        }
      #endif
      }
    Serial.print("\n");
  }	
}

 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * map_core.cpp
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-26
 * Edited:  2015-05-26
 * 
 * Blurb
 * 
 *//////////////////////////////////////////////////////////////////////


/// INCLUDES ///
#include "map_core.h"
#include "exception_core.h"


/// GLOBALS ///
static uint16_t mymap[0][0]; //[NUM_NODES_Y][NUM_NODES_X];
File myFile;


/// FUNCTIONS ///
void init_map_core(void) {
  Serial.print("Initializing SD card...");

  if (!SD.begin(CHIP_SELECT_PIN)) {
    activate_exception(&SD_ERROR);
    Serial.println("initialization failed!");
    //report_exception(&SD_ERROR);
    return;
  }
  Serial.println("initialization done.");
    
  
  //mymap = (uint8_t**)calloc(NUM_NODES_X, sizeof(uint8_t*));
  //for(uint8_t i=0; i<NUM_NODES_X; i++) {
  //  (mymap)[i]=(uint8_t*)calloc(NUM_NODES_X, sizeof(uint8_t));
  //}
}


int8_t get_terrain(Position_t coord) {
  
  int8_t terrain_to_get = -1;

  if (coord.x < NUM_NODES_X && coord.x >= 0 &&
      coord.y < NUM_NODES_Y && coord.y >= 0) {
    terrain_to_get = mymap[coord.y][coord.x];
  }
  return terrain_to_get;
}


int8_t set_terrain(Position_t coord, uint8_t terrain_to_set) {
  
  int8_t set_terrain_success = -1;
  
  if (coord.x < NUM_NODES_X && coord.x >= 0 &&
      coord.y < NUM_NODES_Y && coord.y >= 0) {

    mymap[coord.y][coord.x] = terrain_to_set;
    set_terrain_success = 0;
  }
  return set_terrain_success;
}


void display_map(void) {
  int16_t index_x = 0;
  int16_t index_y = 0;
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

/*
void sd_test1(void) {
  
  double time1 = millis();
  
  // Write
  myFile = SD.open("map.tsv", FILE_WRITE);
  if (myFile) {
    Serial.print("Writing to map.tsv...");
    for (int i=0; i < 1000; i++) {
      for (int j=0; j < 1000; j++) {
        
        myFile.print(4);
        myFile.print('\t');
      }
      myFile.print('\n');
    }
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening map.tsv");
  }
  
  double time2 = millis();
  Serial.print("Method 1 took ");
  Serial.print(time2 - time1);
  Serial.println(" ms to write");
  
  //Read
  myFile = SD.open("map.txt", FILE_WRITE);
  if (myFile) {
    Serial.print("reading map.tsv...");
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening map.tsv");
  }
  
  double time3 = millis();
  Serial.print("Method 1 took ");
  Serial.print(time3 - time2);
  Serial.println(" ms to read");
}*/


/*void sd_test2(void) {
  
  double time1 = millis();
  String folder = "map/";
  String address = "000000000000";
  
  // Write
  Serial.print("Writing to card...");
  for (int i=0; i < 1000; i++) {
    for (int j=0; j < 1000; j++) {
      address = folder + String(i) + String(j);
      myFile = SD.open(address, FILE_WRITE);
      if (myFile) {
        myFile.print(4);
        myFile.close();
      }
    }
  }
  Serial.println("done.");
  
  double time2 = millis();
  Serial.print("Method 1 took ");
  Serial.print(time2 - time1);
  Serial.println(" ms to write");
  
  //Read
  Serial.print("reading...");
  for (int i=0; i < 1000; i++) {
    for (int j=0; j < 1000; j++) {
      address = folder + String(i) + String(j);
      myFile = SD.open(address);
      if (myFile) {
        while (myFile.available()) {
          Serial.write(myFile.read());
        }
        myFile.close();
      }
    }
  }
  
  double time3 = millis();
  Serial.print("Method 1 took ");
  Serial.print(time3 - time2);
  Serial.println(" ms to read");
}*/



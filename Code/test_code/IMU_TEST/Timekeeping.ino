/*
Manages the time library.
- Syncronises time over serial with PC
- Determines startup date/time with sub-second accuracy.
*/




time_t startupDateTime;
float  startupDecimals;




void time_init()
{
  time_set_console();
  time_calculate_startup();
  time_print_startup();
}




// Requests the time via the serial.

void time_set_console()
{
  Serial.println(F("Setting system time.\n"));
  Serial.println(F("What is the date & time?"));
  Serial.println(F("Format YYYY-MM-DD HH:MM:SS"));
  
  while(!Serial.available()) {}
  unsigned long t0 = micros();
  String inputTime = Serial.readString(); 
  Serial.println(inputTime);
  
  int yr = int_from_string(inputTime, 0, 3);
  int mo = int_from_string(inputTime, 5, 6);
  int dy = int_from_string(inputTime, 8, 9);
  int hr = int_from_string(inputTime, 11, 12);
  int mi = int_from_string(inputTime, 14, 15);
  int sc = int_from_string(inputTime, 17, 18);
  
  Serial.print("Year: "  ); Serial.println(yr);
  Serial.print("Month: " ); Serial.println(mo);
  Serial.print("Day: "   ); Serial.println(dy);
  Serial.print("Hour: "  ); Serial.println(hr);
  Serial.print("Minute: "); Serial.println(mi);
  Serial.print("Second: "); Serial.println(sc);
    
  float error = (float)(micros() - t0) / 1000000.0;
  setTime(hr, mi, (float)sc+error, dy, mo, yr);
  Serial.println(F("The time has been set."));
}




// This function extracts an integer from a segment within a string.
// Inputs:
//   string s - the whole string of interest.
//   int start - the index of the first character to contain the integer.
//   int finish - the index of the last character to contain the integer.
// Output:
//   The integer result.

int int_from_string(String s, int start, int finish)
{
  String temp = "";
  for(int I=start; I <= finish; I++)
  {
    temp += s[I];
  }
  return(temp.toInt());
}




// This function determines the startup date/time based on the input date/time & the onboard clock.

void time_calculate_startup()
{
  time_t lastNow = now();
  while(now() == lastNow) {} // Wait for the second to tick over to ensure now() is sub-second accurate.
  float timeSinceStartup = (float) millis() / 1000.0;
  startupDateTime = now() - round(ceil(timeSinceStartup)); // Ceil doesn't output integers for some reason, while round does.
  startupDecimals = ceil(timeSinceStartup) - timeSinceStartup;
}




// This function prints the startup date/time to the serial in a Matlab friendly format.

void time_print_startup()
{
  Serial.println(F("Startup time:"));
  Serial.print("["); Serial.print(  year(startupDateTime)); 
  Serial.print(" "); Serial.print( month(startupDateTime));
  Serial.print(" "); Serial.print(   day(startupDateTime));
  Serial.print(" "); Serial.print(  hour(startupDateTime));
  Serial.print(" "); Serial.print(minute(startupDateTime));
  Serial.print(" "); Serial.print(second(startupDateTime) + startupDecimals, 6);
  Serial.print("]"); Serial.print("\n");
}


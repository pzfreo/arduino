#include <avr/eeprom.h>

struct settings_t
{
  char refresh[40];
} settings, rs;

char refresh[] = "a186a1e2fd963dd7f2becf68cf91";
char bearer[] = "b941631c8f98a550f88637cc42d846d";

void setup()
{
  strncpy(settings.refresh, refresh, sizeof(refresh));
  eeprom_write_block((void*)&settings, (void*)0, sizeof(settings));  
  Serial.begin(9600);

}


void loop() {
  eeprom_read_block((void*)&rs, (void*)0, sizeof(rs));  
  Serial.println(rs.refresh);
  
  
}

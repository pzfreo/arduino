#include <SoftwareSerial.h>

#include <avr/eeprom.h>

/*
 Basic MQTT example with Authentication
 
  - connects to an MQTT server, providing username
    and password
  - publishes "hello world" to the topic "outTopic"
  - subscribes to the topic "inTopic"
*/

#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

struct settings_t
{
  char refresh[50];
  char bearer[50];
} settings;

// Update these with values suitable for your network.
//char bearer[256], refresh[256];
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
byte server[] = { 192, 168, 1, 68 };
//byte ip[]     = { 172, 16, 0, 100 };
char clientid[] = "arduinoclient-0123243";
char bearer[50];
int reconnect=0, publish = 0;
EthernetClient ethClient;
PubSubClient client(server, 1883, callback, ethClient);

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("received payload");
  memcpy(bearer, payload, 50);
  Serial.println(bearer);
  reconnect=1;
}



void setup()
{
  Serial.begin(9600);
  eeprom_read_block((void*)&settings, (void*)0, sizeof(settings));  
  strncpy(bearer, settings.bearer, 50);  
  Ethernet.begin(mac); //, ip);
  
  if (client.connect(clientid, settings.bearer, "dummy")) {
    publish = 1;
    // bearer is still valid
    client.publish("/pzf","hello world");
    
    
  }
  else 
  {
    if (client.connect(clientid, "refresh", "refresh")) {
      String topic = "/clients/"+ String(clientid);
      
      char topicchar[50];
      topic.toCharArray(topicchar,50);
      client.subscribe(topicchar);
      
      // publish two strings (255 delimited) - first client id, second refresh token
      char message[100];
      int phase = 0;
      int j=0, k=0;
      for (int i=0; i<100; i++) {
        if (phase == 0) {
          message[i] = clientid[j];
          if (clientid[j]==0) {
            message[i]=255;
            phase = 1;
          }
          else
          {
            j++;
          }
        }
        else
        {
          message[i]= settings.refresh[k];
          if (settings.refresh[k]==0) break;
          k++;
        }
      }
     
      client.publish("/refresh", message);
    }
  }
}

void loop()
{
  if (reconnect==2) {
    Serial.println("reconnecting");
    //client = client(server, 1883, callback, ethClient);
    if (client.connect(clientid, bearer, "dummy")) {
       publish = 1;
       reconnect=0;
    }
  } else
  if (reconnect==1) {
    reconnect=2;
    Serial.println("disconnecting");
    client.disconnect();
    delay(500);
   }
  client.loop();
  Serial.println("loop");
}


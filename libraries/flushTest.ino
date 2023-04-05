#include <SD.h>

File dataCollection;
int led = 13;
int dataSize = 100000;

void setup() {

  pinMode(led, OUTPUT);
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("READY");
  Serial.begin(9600);

  while (!SD.begin(BUILTIN_SDCARD))
    ;
  Serial.println("SD Card initialized");

  // delete file if previously initialized
  SD.remove("dataCollection.csv");
  dataCollection = SD.open("dataCollection.csv", FILE_WRITE | FILE_READ);


  int current = millis();
//  bool runLoop = true;
  //  while(runLoop){     
  
  Serial.println("Writing to the text file...");
for (int j = 0; j < 25; j++){
  for (double i = 0; i < dataSize; i++) {
    if (dataCollection)
    {
      dataCollection.println(i);
    } else
    {
      // if the file didn't open, report an error:
      Serial.println("error opening the text file!");
    }
//    if (myTime > 5000) runLoop = false;
  }

  int flushStart = micros();
  
  {dataCollection.flush();}
//  delay(1000);
  int flushEnd = micros();
  
//  Serial.println("starting flush at:" + flushStart);

//  Serial.println("ending flush at:" + flushEnd);
//  Serial.println("time taken:" + (flushEnd-flushStart));

  Serial.println(flushStart);
    Serial.println(flushEnd);
  Serial.println(flushEnd-flushStart);
}
  // re-open the text file for reading:
  dataCollection = SD.open("dataCollection.csv");
  if (dataCollection)
  {
    Serial.println("textFile.txt:=========================");

    // read all the text written on the file
    while (dataCollection.available())
    {
//      Serial.write(dataCollection.read());
    }
  } else
  {
    // if the file didn't open, report an error:
    Serial.println("error opening the text file!");
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}

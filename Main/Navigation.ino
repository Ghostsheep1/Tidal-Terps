void setup() {
  Enes100.begin("TidalTerp", WATER, 534, 1116, 8, 9);
  while (!Enes100.isConnected()) {
    Enes100.println("Waiting for connection");
    delay(1000);
  }

  Enes100.println("Connected to overhead vision system");
  Enes100.println("Starting mission");

}
void loop() {
}

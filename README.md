      ESP32C3 Wifi Influence Mine
      ===========================

      A SeedStudio Xiao ESP32C3 plus an LM303 magnetometer provide a
      battery powered wifi vehicle sensor with average current draw of around 1.5mA
      Hence a 32700 6000mA hour battery should last around 4000 hours, or c. 150 days

      Periodically recalibrates itself

      Requires a server endpoint that accepts its json message, and responds with updated parameters for its operation




     


        Wiring
        ======

                         =====================
                        =| G20           G21 |=
                        =| G8             G7 |= LM303 SCL
                        =| G9             G6 |= LM303 SDA
                        =| G10            G5 |=
        LiFePo4 + LM303 =| 3v3            G4 |=
        LiFePo4 + LM303 =| Gnd  |=====|   G3 |=
                        =| 5V   | USB |   G2 |= LM303 DataReady
                         =====================

<body>
  <h1>MT6826S ESP32 Tools</h1>

  <p>Two simple programs for the ESP32 to read and configure the <strong>Magntek MT6826S</strong> encoder.</p>

  <h2>Overview</h2>
  <ul>
    <li><strong>SPI Tool</strong>: Change PPR, enable ABZ output, save to EEPROM.</li>
    <li><strong>ABZ Reader</strong>: Monitor A/B/Z pins and estimate angle. Includes basic self-calibration.</li>
  </ul>

  <h2>How to Use</h2>
  <ol>
    <li>Flash the firmware to your ESP32.</li>
    <li>Power the encoder with <strong>3.3 V</strong>. If using 5 V, use level shifters for signals.</li>
    <li>Wire according to the code comments.</li>
    <li>Open a serial monitor . The menu will show up when the encoder gets detected.</li>
    <li>Change PPR and test. Use option <code>[3]</code> to save it permanently.</li>
  </ol>

  <p><em>Note: EEPROM save might timeout, but changes are usually applied. Power cycle and check if they persist.</em></p>

  <h2>Self-Calibration (ABZ Reader)</h2>
  <ul>
    <li>Send <code>S</code> over serial.</li>
    <li>Rotate the encoder one full turn, then press Enter.</li>
    <li>It will estimate the PPR from the pulses.</li>
  </ul>

  <h2>License</h2>
  <p>MIT — use freely, modify as needed.</p>
</body>
</html>
